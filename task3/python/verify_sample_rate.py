#!/usr/bin/env python3
"""
Task 3 (v2) - procedure step 6: verify the FPGA sampling rate and show that
sampling is never interrupted by the data transfer.

Connect a function generator (sine, e.g. 1 kHz, 1 Vpp, 1.65 V offset) to the
ADC input, then run:

    python verify_sample_rate.py --f-ref 1000 --blocks 60

What it does
  1. Captures M *consecutive* blocks (checks block sequence numbers, so no
     block was lost between FPGA and PC) and the FPGA overrun counter.
  2. Concatenates them into one long record (60 x 1024 = 61 440 samples
     = 3.84 s) and fits a sine  A cos(2 pi f n) + B sin(2 pi f n) + C  by
     non-linear least squares -> frequency in cycles/sample (f_norm).
        Fs_measured = f_ref / f_norm
     (accuracy is limited by the function generator's and the 50 MHz
      oscillator's ppm tolerance - both are typically < 50 ppm).
  3. Fits the phase of each block separately.  If any sample were delayed,
     dropped or duplicated (e.g. because the ESP32 was reading the buffer),
     the phase of the following blocks would jump by 2 pi f Ts.  The script
     reports the timing error of every block in microseconds - with
     hardware-timed sampling it stays at the sub-microsecond noise level,
     far below one sampling period (62.5 us).
  4. Saves a plot  verify_fs_<timestamp>.png  for the lab report.

Also works without hardware:  python verify_sample_rate.py --sim tone --sim-fs 16001.6
"""
from __future__ import annotations

import argparse
import time

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import least_squares

from fpga_stream import FS, N, StreamStats, add_source_args, open_source


def capture(src, m: int, timeout: float):
    src.start()
    t_end = time.time() + timeout
    run, stats = [], StreamStats()
    print(f"capturing {m} consecutive blocks ({m * N / FS:.2f} s) ...")
    while time.time() < t_end:
        for b in src.get_all():
            stats.update(b)
            if run and ((b.seq - run[-1].seq) & 0xFFFF) != 1:
                print(f"  sequence gap {run[-1].seq} -> {b.seq}: restarting capture")
                run = []
            run.append(b)
            if len(run) == m:
                src.stop()
                return run, stats
        time.sleep(0.01)
    src.stop()
    raise SystemExit(f"timeout: only {len(run)} consecutive blocks received")


def fit_sine(x: np.ndarray, f0: float):
    n = np.arange(len(x))

    def resid(p):
        f, a, b, c = p
        w = 2 * np.pi * f * n
        return a * np.cos(w) + b * np.sin(w) + c - x

    # linear initial amplitudes for the coarse frequency
    w = 2 * np.pi * f0 * n
    M = np.column_stack([np.cos(w), np.sin(w), np.ones_like(w)])
    a, b, c = np.linalg.lstsq(M, x, rcond=None)[0]
    r = least_squares(resid, [f0, a, b, c], x_scale=[1e-6, 1, 1, 1], xtol=1e-15, ftol=1e-15)
    return r.x, resid(r.x)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    add_source_args(ap)
    ap.add_argument("--f-ref", type=float, default=None,
                    help="frequency set on the function generator [Hz]")
    ap.add_argument("--blocks", type=int, default=60, help="number of consecutive blocks")
    ap.add_argument("--timeout", type=float, default=30.0)
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()
    if args.sim and args.f_ref is None:
        args.f_ref = args.sim_freq

    blocks, stats = capture(open_source(args), args.blocks, args.timeout)
    codes = np.concatenate([b.samples for b in blocks]).astype(np.float64)
    x = codes - codes.mean()
    L = len(x)

    # ---- coarse frequency: zero-padded Hann FFT ----
    nfft = 1 << int(np.ceil(np.log2(L * 8)))
    spec = np.abs(np.fft.rfft(x * np.hanning(L), nfft))
    k = int(np.argmax(spec[1:])) + 1
    f_coarse = k / nfft

    # ---- fine frequency: least-squares sine fit over the whole record ----
    (f_norm, a, b, c), res = fit_sine(x, f_coarse)
    amp = np.hypot(a, b)
    sinad = 10 * np.log10((amp ** 2 / 2) / np.mean(res ** 2))
    enob = (sinad - 1.76) / 6.02

    print("\n================  RESULT  ================")
    print(f"blocks captured     : {len(blocks)}  (seq {blocks[0].seq} .. {blocks[-1].seq})")
    print(f"source              : {blocks[0].source_text()}")
    print(f"FPGA overrun count  : {blocks[-1].overruns}")
    print(f"record length       : {L} samples")
    print(f"fitted f / Fs       : {f_norm:.9f} cycles/sample")
    print(f"f  (assuming 16 kS/s): {f_norm * FS:.4f} Hz")
    print(f"amplitude           : {amp:.1f} codes, SINAD {sinad:.1f} dB, ENOB {enob:.2f} bit")
    fs_meas = None
    if args.f_ref:
        fs_meas = args.f_ref / f_norm
        ppm = (fs_meas - FS) / FS * 1e6
        print(f"f_ref (generator)   : {args.f_ref:.4f} Hz")
        print(f"==> Fs measured     : {fs_meas:.3f} S/s   ({ppm:+.1f} ppm vs {FS})")
        print(f"    Ts measured     : {1e6 / fs_meas:.5f} us  (nominal {1e6 / FS:.5f} us)")
    host_fs = stats.measured_fs()
    if host_fs:
        print(f"Fs from PC arrival  : {host_fs:.1f} S/s (coarse, USB timing)")

    # ---- per-block phase / timing continuity ----
    w = 2 * np.pi * f_norm
    phases = []
    for i in range(len(blocks)):
        n = np.arange(i * N, (i + 1) * N)
        M = np.column_stack([np.cos(w * n), np.sin(w * n), np.ones(N)])
        ab = np.linalg.lstsq(M, x[i * N:(i + 1) * N], rcond=None)[0]
        phases.append(np.arctan2(-ab[1], ab[0]))
    phases = np.unwrap(np.array(phases))
    dphi = phases - phases.mean()
    dt_us = dphi / w / FS * 1e6 if w > 0 else dphi * 0     # phase error -> time error
    worst = np.max(np.abs(dt_us))
    print(f"\nblock timing error  : max |dt| = {worst:.3f} us   (1 sample = {1e6 / FS:.1f} us)")
    ok = worst < 0.1 * 1e6 / FS and blocks[-1].overruns == 0
    print("continuity          : " + ("PASS - no sample lost, delayed or duplicated"
                                     if ok else "CHECK - timing discontinuity detected"))

    if args.no_plot:
        return
    fig, axs = plt.subplots(3, 1, figsize=(11, 9))
    nb = min(3, len(blocks) - 1)
    lo, hi = nb * N - 60, nb * N + 60
    nn = np.arange(lo, hi)
    fine = np.linspace(lo, hi, 2000)
    axs[0].plot(nn, x[lo:hi], "o", ms=3, label="FPGA samples")
    axs[0].plot(fine, a * np.cos(w * fine) + b * np.sin(w * fine) + c, lw=0.8,
                label="global sine fit")
    axs[0].axvline(nb * N - 0.5, color="r", ls="--", label="block boundary")
    axs[0].set(title="Samples across a block boundary (continuous = no gap)",
               xlabel="sample index", ylabel="code - mean")
    axs[0].legend(fontsize=8)
    axs[1].plot(np.arange(len(blocks)), dt_us, "o-", ms=3)
    axs[1].axhline(0, color="gray", lw=0.6)
    axs[1].set(title=f"Per-block timing error (max {worst:.3f} us, Ts = {1e6 / FS:.1f} us)",
               xlabel="block", ylabel="dt [us]")
    axs[2].plot(res, lw=0.4)
    axs[2].set(title=f"Sine-fit residual (SINAD {sinad:.1f} dB, ENOB {enob:.2f})",
               xlabel="sample index", ylabel="codes")
    for ax in axs:
        ax.grid(alpha=0.3)
    ttl = f"Fs verification: f/Fs = {f_norm:.9f}"
    if fs_meas:
        ttl += f"  ->  Fs = {fs_meas:.2f} S/s"
    fig.suptitle(ttl)
    fig.tight_layout()
    name = time.strftime("verify_fs_%Y%m%d_%H%M%S.png")
    fig.savefig(name, dpi=120)
    print(f"plot saved: {name}")
    plt.show()


if __name__ == "__main__":
    main()
