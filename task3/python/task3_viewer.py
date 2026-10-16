#!/usr/bin/env python3
"""
Task 3 (v2) - real-time viewer for the DE10-Lite FPGA audio sampler.

Shows, for every 1024-sample block (64 ms @ 16 kS/s):
  * time-domain waveform (volts)
  * single-sided FFT magnitude spectrum in dBFS - Hann vs. rectangular window
  * rolling spectrogram
  * metrics: dominant frequency (interpolated), DC, Vpp, RMS, SNR estimate,
    block sequence / lost blocks / FPGA overruns / checksum errors,
    long-term sample rate measured by the host

Keys:  space = pause/resume   r = start/stop WAV recording
       s     = save snapshot (PNG + NPZ)   w = toggle rectangular-window trace

Run:
    python task3_viewer.py                    # auto-detect ESP32 serial port
    python task3_viewer.py --port /dev/cu.usbserial-0001
    python task3_viewer.py --sim mic          # no hardware: simulated microphone
    python task3_viewer.py --sim tone --sim-freq 1234
"""
from __future__ import annotations

import argparse
import time
import wave
from collections import deque
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

from fpga_stream import ADC_MID, FS, N, StreamStats, add_source_args, open_source

SPEC_BLOCKS = 150                     # spectrogram history (150 x 64 ms = 9.6 s)
WIN_HANN = np.hanning(N)
WIN_RECT = np.ones(N)
FREQS = np.fft.rfftfreq(N, 1 / FS)


def spectrum_dbfs(codes: np.ndarray, win: np.ndarray) -> np.ndarray:
    """Amplitude spectrum in dBFS (0 dBFS = full-scale sine, amplitude 2048 codes)."""
    x = codes.astype(np.float64)
    x -= x.mean()
    X = np.fft.rfft(x * win)
    amp = np.abs(X) * 2.0 / win.sum()
    amp[0] /= 2.0
    return 20 * np.log10(np.maximum(amp, 1e-9) / ADC_MID)


def dominant_freq(db: np.ndarray) -> tuple[float, float]:
    """Peak bin + parabolic interpolation on the log magnitude (Hann spectrum)."""
    k = int(np.argmax(db[2:-1])) + 2          # skip DC / first bin
    a, b, c = db[k - 1], db[k], db[k + 1]
    den = a - 2 * b + c
    d = 0.5 * (a - c) / den if den != 0 else 0.0
    return (k + d) * FS / N, b - 0.25 * (a - c) * d


def snr_estimate(db: np.ndarray) -> float:
    """Power in +/-3 bins around the peak vs. the rest (excluding DC)."""
    p = 10 ** (db / 10)
    k = int(np.argmax(p[2:])) + 2
    sig = p[max(k - 3, 0):k + 4].sum()
    noise = p[2:].sum() - sig
    return 10 * np.log10(sig / noise) if noise > 0 else float("inf")


class Viewer:
    def __init__(self, src, vref: float | None):
        self.src = src
        self.vref = vref
        self.stats = StreamStats()
        self.paused = False
        self.show_rect = True
        self.last = None
        self.wav: wave.Wave_write | None = None
        self.wav_name = ""
        self.spec = np.full((len(FREQS), SPEC_BLOCKS), -120.0)
        self.rate_hist = deque(maxlen=50)

        plt.rcParams["toolbar"] = "None"
        self.fig = plt.figure(figsize=(13, 8.5))
        self.fig.canvas.manager.set_window_title("Task 3 v2 - FPGA audio sampler (16 kS/s, N = 1024)")
        gs = self.fig.add_gridspec(3, 3, height_ratios=[1, 1, 1.1], hspace=0.45, wspace=0.3)

        self.ax_t = self.fig.add_subplot(gs[0, :2])
        self.t_ms = np.arange(N) / FS * 1e3
        (self.l_wave,) = self.ax_t.plot(self.t_ms, np.full(N, 1.65), lw=1.8, color="#2563EB", label="v(t)")
        self.l_dc = self.ax_t.axhline(1.65, color="#F97316", ls="--", lw=1.2, label="DC Bias (~1.65V)")
        self.ax_t.set(xlim=(0, self.t_ms[-1]), ylim=(-0.1, 3.4), xlabel="time [ms]",
                      ylabel="voltage [V]", title="Time-domain waveform (N = 1024, Ts = 62.5 µs)")
        self.ax_t.grid(True, color="#E2E8F0", linestyle="-", linewidth=0.8)
        self.ax_t.legend(loc="upper right", fontsize=8.5, framealpha=0.9)

        self.ax_f = self.fig.add_subplot(gs[1, :2])
        (self.l_rect,) = self.ax_f.plot(FREQS, np.full(len(FREQS), -120.0), lw=1.2, ls="--",
                                        color="#DC2626", alpha=0.7, label="rectangular (no window)")
        (self.l_hann,) = self.ax_f.plot(FREQS, np.full(len(FREQS), -120.0), lw=2.0,
                                        color="#16A34A", label="Hann window (scaled)")
        (self.l_peak,) = self.ax_f.plot([], [], "rv", ms=8, label="peak tone")
        self.ax_f.set(xlim=(0, FS / 2), ylim=(-110, 5), xlabel="frequency [Hz]",
                      ylabel="magnitude [dBFS]",
                      title=f"FFT magnitude spectrum  (Δf = Fs/N = {FS / N:.3f} Hz)")
        self.ax_f.grid(True, color="#E2E8F0", linestyle="-", linewidth=0.8)
        self.ax_f.legend(loc="upper right", fontsize=8.5, framealpha=0.9)

        self.ax_s = self.fig.add_subplot(gs[2, :2])
        dur = SPEC_BLOCKS * N / FS
        self.im = self.ax_s.imshow(self.spec, origin="lower", aspect="auto", cmap="turbo",
                                   extent=(-dur, 0, 0, FS / 2), vmin=-80, vmax=0,
                                   interpolation="nearest")
        self.ax_s.set(xlabel="time [s]", ylabel="frequency [Hz]", title="Spectrogram (Hann, dBFS)")
        cbar = self.fig.colorbar(self.im, ax=self.ax_s, pad=0.01)
        cbar.set_label("dBFS", fontsize=8)

        self.ax_info = self.fig.add_subplot(gs[:, 2])
        self.ax_info.axis("off")
        self.txt = self.ax_info.text(0.0, 1.0, "waiting for data ...", va="top", ha="left",
                                     family="monospace", fontsize=9.5,
                                     transform=self.ax_info.transAxes)

        self.fig.canvas.mpl_connect("key_press_event", self.on_key)
        self.anim = FuncAnimation(self.fig, self.update, interval=30, cache_frame_data=False)

    # ------------------------------------------------------------------
    def on_key(self, ev):
        if ev.key == " ":
            self.paused = not self.paused
        elif ev.key == "w":
            self.show_rect = not self.show_rect
            self.l_rect.set_visible(self.show_rect)
        elif ev.key == "s" and self.last is not None:
            stamp = time.strftime("%Y%m%d_%H%M%S")
            self.fig.savefig(f"task3_snapshot_{stamp}.png", dpi=120)
            np.savez(f"task3_snapshot_{stamp}.npz", codes=self.last.samples,
                     seq=self.last.seq, status=self.last.status, fs=FS)
            print(f"saved task3_snapshot_{stamp}.png/.npz")
        elif ev.key == "r":
            self.toggle_recording()

    def toggle_recording(self):
        if self.wav is None:
            self.wav_name = time.strftime("task3_rec_%Y%m%d_%H%M%S.wav")
            self.wav = wave.open(self.wav_name, "wb")
            self.wav.setnchannels(1)
            self.wav.setsampwidth(2)
            self.wav.setframerate(FS)
            print(f"recording -> {self.wav_name}")
        else:
            self.wav.close()
            self.wav = None
            print(f"saved {self.wav_name}")

    # ------------------------------------------------------------------
    def update(self, _frame):
        blocks = self.src.get_all()
        if self.src.error:
            self.txt.set_text(f"SERIAL ERROR:\n{self.src.error}")
            return
        for b in blocks:
            self.stats.update(b)
            if self.wav is not None:          # 12-bit offset binary -> signed 16-bit
                pcm = ((b.samples.astype(np.int32) - ADC_MID) << 4).astype("<i2")
                self.wav.writeframes(pcm.tobytes())
            db = spectrum_dbfs(b.samples, WIN_HANN)
            self.spec = np.roll(self.spec, -1, axis=1)
            self.spec[:, -1] = db
        if not blocks or self.paused:
            return
        b = blocks[-1]
        self.last = b

        v = b.volts(self.vref)
        vref = self.vref or (5.0 if b.max10_adc else 3.3)
        db_h = spectrum_dbfs(b.samples, WIN_HANN)
        db_r = spectrum_dbfs(b.samples, WIN_RECT)
        f_dom, a_dom = dominant_freq(db_h)

        self.l_wave.set_ydata(v)
        self.l_dc.set_ydata([v.mean(), v.mean()])
        self.ax_t.set_ylim(0, vref)
        self.l_hann.set_ydata(db_h)
        self.l_rect.set_ydata(db_r)
        self.l_peak.set_data([f_dom], [a_dom + 3])
        self.im.set_data(self.spec)

        ac = v - v.mean()
        clip = np.mean((b.samples <= 2) | (b.samples >= 4093)) * 100
        fs_meas = self.stats.measured_fs()
        p = self.src.parser
        lines = [
            "TASK 3 v2  FPGA -> ESP32 -> PC",
            "-" * 34,
            f"source      : {b.source_text()}",
            f"Fs (FPGA)   : {FS} S/s  (Ts = {1e6 / FS:.1f} us)",
            f"Fs (host)   : " + (f"{fs_meas:9.1f} S/s" if fs_meas else "measuring..."),
            f"N / block   : {N}  ({N / FS * 1e3:.1f} ms)",
            "",
            "SIGNAL",
            f"dominant f  : {f_dom:9.2f} Hz",
            f"peak level  : {a_dom:9.1f} dBFS",
            f"SNR (est.)  : {snr_estimate(db_h):9.1f} dB",
            f"DC          : {v.mean():9.3f} V",
            f"Vpp         : {np.ptp(v):9.3f} V",
            f"Vrms (AC)   : {np.sqrt(np.mean(ac ** 2)):9.4f} V",
            f"clipping    : {clip:9.2f} %" + ("  <-- TOO LOUD" if clip > 0.5 else ""),
            "",
            "LINK",
            f"block seq   : {b.seq}",
            f"blocks rx   : {self.stats.blocks}",
            f"lost blocks : {self.stats.lost}",
            f"FPGA overrun: {b.overruns}",
            f"bad checksum: {p.bad_checksum}",
            f"bad header  : {p.bad_header}",
            f"resync bytes: {p.resync_bytes}",
            "",
            "REC  " + (f"* {self.wav_name}" if self.wav else "off  (press r)"),
            "",
            "keys: space pause  r record",
            "      s snapshot   w rect window",
        ]
        self.txt.set_text("\n".join(lines))

    def run(self):
        self.src.start()
        try:
            plt.show()
        finally:
            self.src.stop()
            if self.wav is not None:
                self.wav.close()
                print(f"saved {self.wav_name}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    add_source_args(ap)
    ap.add_argument("--vref", type=float, default=None,
                    help="ADC full-scale voltage (default: 3.3 V MCP3201, 5.0 V MAX10 header)")
    args = ap.parse_args()
    Viewer(open_source(args), args.vref).run()


if __name__ == "__main__":
    main()
