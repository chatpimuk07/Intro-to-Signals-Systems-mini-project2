"""
Task 3 (v2) - common receive code shared by the Python tools.

Serial packet sent by the ESP32 bridge (see esp32/task3_esp32_bridge.ino):

    AA 55 AA 55 | FPGA frame (2056 B) | sum16 (uint16 LE, sum of frame bytes)

FPGA frame (produced by spi_slave_tx.vhd, all fields big-endian):

    A5 5A | block_seq u16 | N u16 (=1024) | status u16 | N x u16 samples (12-bit)

    status[15]    1 = DDS test tone, 0 = ADC
    status[14]    1 = MAX 10 on-chip ADC, 0 = MCP3201
    status[13:11] SW[3:1] tone select
    status[7:0]   FPGA overrun counter
"""
from __future__ import annotations

import glob
import queue
import sys
import threading
import time
from dataclasses import dataclass, field

import numpy as np

FS = 16_000                 # FPGA sampling rate (must match FS_HZ generic)
N = 1024                    # samples per block
HDR = 8
FRAME_BYTES = HDR + 2 * N   # 2056
SYNC = b"\xAA\x55\xAA\x55"
PACKET_BYTES = len(SYNC) + FRAME_BYTES + 2
ADC_BITS = 12
ADC_MID = 1 << (ADC_BITS - 1)
DEFAULT_BAUD = 2_000_000
TONES_HZ = [1000.0, 1007.8125, 440.0, 250.0, 2000.0, 3000.0, 5000.0, 7000.0]


@dataclass
class Block:
    seq: int
    status: int
    samples: np.ndarray            # uint16 codes 0..4095
    t_host: float = field(default_factory=time.perf_counter)

    @property
    def test_mode(self) -> bool:
        return bool(self.status & 0x8000)

    @property
    def max10_adc(self) -> bool:
        return bool(self.status & 0x4000)

    @property
    def tone_sel(self) -> int:
        return (self.status >> 11) & 0x7

    @property
    def overruns(self) -> int:
        return self.status & 0xFF

    def source_text(self) -> str:
        if self.test_mode:
            return f"FPGA DDS test tone {TONES_HZ[self.tone_sel]:g} Hz"
        return "MAX 10 on-chip ADC" if self.max10_adc else "MCP3201 ADC"

    def volts(self, vref: float | None = None) -> np.ndarray:
        """Convert codes to volts.  MCP3201: Vref (3.3 V); DE10-Lite header: 0..5 V."""
        if vref is None:
            vref = 5.0 if self.max10_adc else 3.3
        return self.samples.astype(np.float64) * (vref / (1 << ADC_BITS))


class PacketParser:
    """Byte-stream -> Block parser with resynchronisation and integrity checks."""

    def __init__(self):
        self.buf = bytearray()
        self.good = 0
        self.bad_checksum = 0
        self.bad_header = 0
        self.resync_bytes = 0

    def feed(self, data: bytes) -> list[Block]:
        self.buf += data
        out: list[Block] = []
        min_bytes = 2056  # v1 packet size: SYNC(4) + seq(2) + N(2) + data(2048)
        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                keep = len(SYNC) - 1
                self.resync_bytes += max(0, len(self.buf) - keep)
                del self.buf[:-keep or None]
                break
            if i > 0:
                self.resync_bytes += i
                del self.buf[:i]
            if len(self.buf) < min_bytes:
                break

            # Check if this is v2 format (starts with SYNC + 0xA5 0x5A)
            if len(self.buf) >= 6 and self.buf[4:6] == b"\xA5\x5A":
                if len(self.buf) < PACKET_BYTES:
                    break
                frame = bytes(self.buf[4:4 + FRAME_BYTES])
                chk = int.from_bytes(self.buf[4 + FRAME_BYTES:PACKET_BYTES], "little")
                n_val = int.from_bytes(frame[4:6], "big")
                if n_val != N:
                    self.bad_header += 1
                    del self.buf[:1]
                    continue
                if (sum(frame) & 0xFFFF) != chk:
                    self.bad_checksum += 1
                    del self.buf[:1]
                    continue
                del self.buf[:PACKET_BYTES]
                seq = int.from_bytes(frame[2:4], "big")
                status = int.from_bytes(frame[6:8], "big")
                samples = np.frombuffer(frame, dtype=">u2", offset=HDR, count=N).astype(np.uint16) & 0x0FFF
                out.append(Block(seq, status, samples))
                self.good += 1
            else:
                # Task 3 v1 format (2056 bytes: SYNC + seq + N + samples)
                pkt = bytes(self.buf[:2056])
                n_le = int.from_bytes(pkt[6:8], "little")
                n_be = int.from_bytes(pkt[6:8], "big")
                if n_le == N:
                    del self.buf[:2056]
                    seq = int.from_bytes(pkt[4:6], "little")
                    samples = np.frombuffer(pkt, dtype="<u2", offset=8, count=N).astype(np.uint16) & 0x0FFF
                    out.append(Block(seq, 0, samples))
                    self.good += 1
                elif n_be == N:
                    del self.buf[:2056]
                    seq = int.from_bytes(pkt[4:6], "big")
                    samples = np.frombuffer(pkt, dtype=">u2", offset=8, count=N).astype(np.uint16) & 0x0FFF
                    out.append(Block(seq, 0, samples))
                    self.good += 1
                else:
                    self.bad_header += 1
                    del self.buf[:1]
                    continue
        return out


def build_packet(seq: int, samples: np.ndarray, status: int = 0) -> bytes:
    """Create a packet exactly like the FPGA + ESP32 would (used by the simulator)."""
    frame = (b"\xA5\x5A" + (seq & 0xFFFF).to_bytes(2, "big") + N.to_bytes(2, "big")
             + (status & 0xFFFF).to_bytes(2, "big")
             + np.asarray(samples, dtype=">u2").tobytes())
    return SYNC + frame + (sum(frame) & 0xFFFF).to_bytes(2, "little")


def find_serial_port() -> str | None:
    pats = ["/dev/cu.usbserial*", "/dev/cu.SLAB_USBtoUART*", "/dev/cu.wchusbserial*",
            "/dev/cu.usbmodem*", "/dev/ttyUSB*", "/dev/ttyACM*"]
    for p in pats:
        hits = sorted(glob.glob(p))
        if hits:
            return hits[0]
    if sys.platform.startswith("win"):
        try:
            from serial.tools import list_ports
            ports = [p.device for p in list_ports.comports()]
            return ports[0] if ports else None
        except Exception:
            return None
    return None


class StreamStats:
    def __init__(self):
        self.blocks = 0
        self.lost = 0
        self.last_seq: int | None = None
        self.t_first: float | None = None
        self.t_last: float | None = None
        self.overruns = 0

    def update(self, b: Block):
        if self.last_seq is not None:
            gap = (b.seq - self.last_seq) & 0xFFFF
            if gap == 0:
                return
            self.lost += gap - 1
        if self.t_first is None:
            self.t_first = b.t_host
            self.seq_first = b.seq
        self.last_seq = b.seq
        self.t_last = b.t_host
        self.blocks += 1
        self.overruns = b.overruns

    def measured_fs(self) -> float | None:
        """Long-term sample rate seen by the host = samples generated / elapsed time.
        (Coarse: based on PC arrival times, good to ~0.1 % after a few seconds.)"""
        if self.t_first is None or self.t_last is None or self.t_last - self.t_first < 1.0:
            return None
        nblk = (self.last_seq - self.seq_first) & 0xFFFF
        return nblk * N / (self.t_last - self.t_first)


class _Source(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.q: queue.Queue[Block] = queue.Queue(maxsize=256)
        self.parser = PacketParser()
        self.running = True
        self.error: str | None = None

    def _push(self, blocks):
        for b in blocks:
            if self.q.full():
                try:
                    self.q.get_nowait()
                except queue.Empty:
                    pass
            self.q.put_nowait(b)

    def get_all(self) -> list[Block]:
        out = []
        while True:
            try:
                out.append(self.q.get_nowait())
            except queue.Empty:
                return out

    def stop(self):
        self.running = False


class SerialSource(_Source):
    def __init__(self, port: str, baud: int = DEFAULT_BAUD):
        super().__init__()
        import serial  # pyserial
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.ser.reset_input_buffer()

    def run(self):
        try:
            while self.running:
                data = self.ser.read(max(1, self.ser.in_waiting or 4096))
                if data:
                    self._push(self.parser.feed(data))
        except Exception as e:  # pragma: no cover
            self.error = str(e)
        finally:
            self.ser.close()


class SimSource(_Source):
    """Generates realistic packets in real time (no hardware needed).

    mode: 'tone'  - function-generator-like sine (DC 1.65 V) at sim_freq
          'mic'   - microphone-like signal: noise + voice-ish harmonics + claps
          'dds'   - FPGA test-tone mode (status bit 15 set)
    """

    def __init__(self, mode: str = "mic", freq: float = 1000.0, fs_true: float = FS,
                 vref: float = 3.3):
        super().__init__()
        self.mode, self.freq, self.fs_true, self.vref = mode, freq, fs_true, vref
        self.n0 = 0
        self.rng = np.random.default_rng(1)

    def _samples(self) -> np.ndarray:
        n = np.arange(self.n0, self.n0 + N)
        t = n / self.fs_true
        self.n0 += N
        lsb = self.vref / 4096
        if self.mode in ("tone", "dds"):
            v = 1.65 + 1.0 * np.sin(2 * np.pi * self.freq * t)
        else:
            v = 1.65 + 0.01 * self.rng.standard_normal(N)
            sec = t[0] % 6.0
            if 1.0 < sec < 3.0:      # "speech": gliding harmonic series
                f0 = 140 + 40 * np.sin(2 * np.pi * 0.7 * t)
                ph = 2 * np.pi * np.cumsum(f0) / self.fs_true
                v += sum(0.25 / k * np.sin(k * ph) for k in range(1, 8))
            if 4.0 < sec < 4.064:    # "hand clap": decaying noise burst
                v += 1.2 * self.rng.standard_normal(N) * np.exp(-(t - t[0]) * 80)
        codes = np.clip(np.round(v / lsb), 0, 4095).astype(np.uint16)
        return codes

    def run(self):
        seq = 0
        t_next = time.perf_counter()
        status = 0x8000 if self.mode == "dds" else 0
        while self.running:
            pkt = build_packet(seq, self._samples(), status)
            self._push(self.parser.feed(pkt))
            seq = (seq + 1) & 0xFFFF
            t_next += N / self.fs_true
            time.sleep(max(0.0, t_next - time.perf_counter()))


def open_source(args) -> _Source:
    """Create a data source from common CLI arguments (--sim, --port, --baud)."""
    if getattr(args, "sim", None):
        return SimSource(mode=args.sim, freq=getattr(args, "sim_freq", 1000.0),
                         fs_true=getattr(args, "sim_fs", FS))
    port = args.port or find_serial_port()
    if not port:
        raise SystemExit("No serial port found - use --port <dev> or --sim mic|tone|dds")
    print(f"opening {port} @ {args.baud} baud")
    return SerialSource(port, args.baud)


def add_source_args(ap):
    ap.add_argument("--port", help="serial port of the ESP32 (auto-detect if omitted)")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--sim", choices=["mic", "tone", "dds"],
                    help="run without hardware using a simulated stream")
    ap.add_argument("--sim-freq", type=float, default=1000.0, help="simulated tone frequency")
    ap.add_argument("--sim-fs", type=float, default=FS,
                    help="'true' Fs of the simulated hardware (to test Fs verification)")
