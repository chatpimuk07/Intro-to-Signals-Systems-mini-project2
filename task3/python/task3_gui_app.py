#!/usr/bin/env python3
"""
=============================================================================
Signals & Systems / Digital Signal Processing Practical Labs
Task 3 (v2): Audio Sampling using MAX 10 FPGA ADC & Analog Microphone
Mini-Project Advanced GUI Application:
"Acoustic Intelligence & Musical Pitch DSP Workstation"
=============================================================================
High-Contrast, Ultra-Clear Scientific UI for macOS, Windows, and Linux.

Features:
  1. High-Contrast, Professional Dashboard: Large fonts, crisp cards, clean layout
  2. Real-Time Time-Domain Waveform: Calibrated 0 - 3.3V (Ts = 62.5 µs, N = 1024)
  3. Single-Sided FFT Magnitude Spectrum: Hann vs. Rectangular Window comparison
  4. Real-Time Rolling Spectrogram: Turbo colormap with calibrated dBFS scale
  5. Precision Musical Pitch Tuner: HPS algorithm, Note name, Cents error gauge
  6. Quantitative DSP Metrics: Vrms, Vpp, DC bias, dBFS, SNR, THD, Centroid, ZCR
  7. Smart Acoustic Event Classifier: Silence, Speech, Tone, Hand Clap counter
  8. Hardware Telemetry & Diagnostics: Fs verification, FPGA overruns, packet rate
  9. Dual Source: Live Hardware (USB Serial @ 2 Mbaud) or Simulator (Mic/Tone/DDS)
 10. Report Snapshot (PNG + NPZ) & WAV Audio Recording
=============================================================================
"""

from __future__ import annotations

import math
import sys
import time
import wave
from pathlib import Path

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import numpy as np

import tkinter as tk
from tkinter import ttk, messagebox

from fpga_stream import (
    ADC_BITS,
    ADC_MID,
    DEFAULT_BAUD,
    FS,
    N,
    SerialSource,
    SimSource,
    StreamStats,
    find_serial_port,
)

# ----------------- Pitch Detection Constants -----------------
NOTE_NAMES = ["C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"]

def freq_to_note(freq: float) -> tuple[str, float, float]:
    """Calculates nearest musical note name, target frequency, and cents offset."""
    if freq < 25.0 or math.isnan(freq) or math.isinf(freq):
        return ("---", 0.0, 0.0)
    midi_num = 69.0 + 12.0 * math.log2(max(freq, 1.0) / 440.0)
    nearest_midi = int(round(midi_num))
    cents = (midi_num - nearest_midi) * 100.0
    note_idx = nearest_midi % 12
    octave = (nearest_midi // 12) - 1
    note_str = f"{NOTE_NAMES[note_idx]}{octave}"
    target_freq = 440.0 * (2.0 ** ((nearest_midi - 69.0) / 12.0))
    return (note_str, target_freq, cents)

def compute_hps_pitch(samples: np.ndarray, fs: float = FS) -> float:
    """Estimates fundamental frequency using Harmonic Product Spectrum (HPS)."""
    x = samples.astype(np.float64) - np.mean(samples)
    if np.max(np.abs(x)) < 15:
        return 0.0
    win = np.hanning(len(x))
    fft_spec = np.abs(np.fft.rfft(x * win))
    freqs = np.fft.rfftfreq(len(x), 1.0 / fs)

    order = 3
    hps = np.copy(fft_spec)
    for h in range(2, order + 1):
        decimated = fft_spec[::h]
        hps[:len(decimated)] *= decimated

    valid_mask = (freqs >= 55.0) & (freqs <= 2500.0)
    if not np.any(valid_mask):
        return 0.0

    valid_freqs = freqs[valid_mask]
    valid_hps = hps[valid_mask]
    peak_idx = int(np.argmax(valid_hps))
    best_freq = float(valid_freqs[peak_idx])

    if 0 < peak_idx < len(valid_hps) - 1:
        alpha = float(valid_hps[peak_idx - 1])
        beta = float(valid_hps[peak_idx])
        gamma = float(valid_hps[peak_idx + 1])
        denom = alpha - 2.0 * beta + gamma
        if abs(denom) > 1e-12:
            delta = 0.5 * (alpha - gamma) / denom
            best_freq += delta * (fs / len(x))

    return max(0.0, best_freq)

# ----------------- Bulletproof High-Contrast Button Widget -----------------
class HighContrastButton(tk.Label):
    """
    Bulletproof high-contrast 3D button widget for macOS Aqua and cross-platform Tkinter.
    Avoids native macOS NSButton bugs where background colors are stripped and white text becomes invisible.
    """
    def __init__(
        self,
        master,
        text="",
        command=None,
        bg="#16A34A",
        fg="#FFFFFF",
        font=("Helvetica", 11, "bold"),
        padx=14,
        pady=6,
        bd=3,
        relief="raised",
        **kwargs
    ):
        super().__init__(
            master,
            text=text,
            bg=bg,
            fg=fg,
            font=font,
            padx=padx,
            pady=pady,
            bd=bd,
            relief=relief,
            cursor="hand2",
            **kwargs
        )
        self.command = command
        self._bg_color = bg
        self._fg_color = fg
        self.bind("<Button-1>", self._on_press)
        self.bind("<ButtonRelease-1>", self._on_release)

    def _on_press(self, _event=None):
        self.config(relief="sunken")
        if self.command:
            self.command()

    def _on_release(self, _event=None):
        self.config(relief="raised")

    def config(self, **kwargs):
        if "bg" in kwargs:
            self._bg_color = kwargs["bg"]
        if "fg" in kwargs:
            self._fg_color = kwargs["fg"]
        super().config(**kwargs)

    configure = config

# ----------------- Main Workstation Application -----------------
class Task3AdvancedApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Task 3 (v2): FPGA Audio DSP Workstation & Musical Pitch Tuner")
        self.root.geometry("1380x940")
        self.root.minsize(1120, 760)
        self.root.configure(bg="#F1F5F9")  # Clean, crisp high-contrast slate background

        # Application State
        self.source = None
        self.stream_stats = StreamStats()
        self.is_running = False
        self.is_frozen = False
        self.auto_zoom = True  # Default to auto-zoom on so small mic signals are vivid
        self.wav_file: wave.Wave_write | None = None
        self.wav_filename = ""
        self.last_block = None

        # Classifier State
        self.clap_counter = 0
        self.prev_energy = 0.0
        self.clap_cooldown = 0

        # DSP Buffers
        self.spec_history_blocks = 70
        self.spec_matrix = np.full((N // 2 + 1, self.spec_history_blocks), -80.0)
        self.hann_window = np.hanning(N)
        self.win_scale = 1.0 / np.mean(self.hann_window)
        self.freq_axis = np.fft.rfftfreq(N, 1.0 / FS)
        self.time_axis_ms = np.arange(N) / FS * 1000.0

        # Build Clean, High-Contrast UI
        self._build_header()
        self._build_toolbar()
        self._build_cards()
        self._build_plots()

        # Start periodic GUI refresh (30 FPS)
        self.root.after(33, self._ui_update_loop)

    # -------------------------------------------------------------------------
    # 1. Top Header Bar
    # -------------------------------------------------------------------------
    def _build_header(self):
        header = tk.Frame(self.root, bg="#0F172A", height=60)
        header.pack(fill=tk.X, side=tk.TOP)

        title_box = tk.Frame(header, bg="#0F172A")
        title_box.pack(side=tk.LEFT, padx=18, pady=10)

        lbl_title = tk.Label(
            title_box,
            text="🎵 FPGA Audio DSP Workstation & Musical Pitch Tuner",
            font=("Helvetica", 17, "bold"),
            fg="#FFFFFF", bg="#0F172A"
        )
        lbl_title.pack(anchor="w")

        lbl_sub = tk.Label(
            title_box,
            text="Task 3 (v2)  |  DE10-Lite MAX 10 FPGA (VHDL)  |  Fs = 16.000 kS/s  |  N = 1024  |  Δf = 15.625 Hz",
            font=("Helvetica", 10),
            fg="#94A3B8", bg="#0F172A"
        )
        lbl_sub.pack(anchor="w")

        self.status_badge = tk.Label(
            header,
            text="● STREAM STOPPED",
            font=("Helvetica", 11, "bold"),
            fg="#EF4444", bg="#1E293B",
            padx=14, pady=6, relief="solid", bd=1
        )
        self.status_badge.pack(side=tk.RIGHT, padx=18, pady=12)

    # -------------------------------------------------------------------------
    # 2. Control Toolbar
    # -------------------------------------------------------------------------
    def _build_toolbar(self):
        bar = tk.Frame(self.root, bg="#FFFFFF", highlightbackground="#CBD5E1", highlightthickness=1, padx=14, pady=8)
        bar.pack(fill=tk.X, side=tk.TOP, padx=14, pady=(10, 6))

        # Source Selection
        tk.Label(bar, text="Source:", font=("Helvetica", 11, "bold"), fg="#0F172A", bg="#FFFFFF").pack(side=tk.LEFT, padx=(0, 6))

        self.source_mode_var = tk.StringVar(value="Simulator: Mic")
        modes = ["Simulator: Mic", "Simulator: 1kHz Tone", "Simulator: DDS Test Tone", "Live Hardware (Serial)"]
        self.combo_source = ttk.Combobox(bar, textvariable=self.source_mode_var, values=modes, state="readonly", width=20)
        self.combo_source.pack(side=tk.LEFT, padx=4)
        self.combo_source.bind("<<ComboboxSelected>>", self._on_source_changed)

        # Serial Port
        tk.Label(bar, text="Port:", font=("Helvetica", 11, "bold"), fg="#0F172A", bg="#FFFFFF").pack(side=tk.LEFT, padx=(12, 4))
        self.port_var = tk.StringVar(value=find_serial_port() or "No device found")
        self.port_entry = tk.Entry(bar, textvariable=self.port_var, width=20, font=("Helvetica", 10),
                                  bg="#F8FAFC", fg="#0F172A", relief="solid", bd=1)
        self.port_entry.pack(side=tk.LEFT, padx=3)
        self.port_entry.config(state="disabled")

        self.btn_refresh = HighContrastButton(
            bar, text="🔄 REFRESH", font=("Helvetica", 10, "bold"),
            fg="#FFFFFF", bg="#0F172A", padx=10, pady=5, bd=2, command=self._refresh_ports
        )
        self.btn_refresh.pack(side=tk.LEFT, padx=(3, 10))

        # Action Buttons (High-Visibility High-Contrast Palette)
        self.btn_stream = HighContrastButton(
            bar, text="▶ START STREAM", font=("Helvetica", 11, "bold"),
            fg="#FFFFFF", bg="#16A34A", padx=14, pady=5, bd=3, command=self.toggle_stream
        )
        self.btn_stream.pack(side=tk.LEFT, padx=4)

        self.btn_freeze = HighContrastButton(
            bar, text="⏸ FREEZE", font=("Helvetica", 11, "bold"),
            fg="#FFFFFF", bg="#EA580C", padx=12, pady=5, bd=3, command=self.toggle_freeze
        )
        self.btn_freeze.pack(side=tk.LEFT, padx=4)

        self.btn_zoom = HighContrastButton(
            bar, text="🔍 AUTO-ZOOM: ON", font=("Helvetica", 11, "bold"),
            fg="#FFFFFF", bg="#0284C7", padx=12, pady=5, bd=3, command=self.toggle_zoom
        )
        self.btn_zoom.pack(side=tk.LEFT, padx=4)

        self.btn_rec = HighContrastButton(
            bar, text="● REC WAV", font=("Helvetica", 11, "bold"),
            fg="#FFFFFF", bg="#7C3AED", padx=12, pady=5, bd=3, command=self.toggle_recording
        )
        self.btn_rec.pack(side=tk.LEFT, padx=4)

        self.btn_snap = HighContrastButton(
            bar, text="📷 SNAPSHOT", font=("Helvetica", 11, "bold"),
            fg="#FFFFFF", bg="#0F172A", padx=12, pady=5, bd=3, command=self.save_snapshot
        )
        self.btn_snap.pack(side=tk.RIGHT, padx=4)

    def _on_source_changed(self, _event=None):
        if "Live Hardware" in self.source_mode_var.get():
            self.port_entry.config(state="normal")
            self._refresh_ports()
        else:
            self.port_entry.config(state="disabled")

    def _refresh_ports(self):
        detected = find_serial_port()
        if detected:
            self.port_var.set(detected)
        elif not self.port_var.get() or self.port_var.get() == "No device found":
            self.port_var.set("No device found")

    # -------------------------------------------------------------------------
    # 3. High-Contrast Dashboard Cards Panel
    # -------------------------------------------------------------------------
    def _build_cards(self):
        panel = tk.Frame(self.root, bg="#F1F5F9", padx=14, pady=4)
        panel.pack(fill=tk.X, side=tk.TOP)

        def make_card(title: str):
            card = tk.Frame(panel, bg="#FFFFFF", highlightbackground="#CBD5E1", highlightthickness=1, padx=14, pady=10)
            lbl = tk.Label(card, text=title, font=("Helvetica", 12, "bold"), fg="#0F172A", bg="#FFFFFF")
            lbl.pack(anchor="w", pady=(0, 4))
            ttk.Separator(card, orient="horizontal").pack(fill=tk.X, pady=(0, 6))
            return card

        # Card 1: 🎯 Pitch Tuner
        card_tuner = make_card("🎯 Musical Pitch Tuner")
        card_tuner.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=4)

        tuner_inner = tk.Frame(card_tuner, bg="#FFFFFF")
        tuner_inner.pack(fill=tk.BOTH, expand=True)

        self.lbl_note = tk.Label(tuner_inner, text="---", font=("Helvetica", 36, "bold"), fg="#2563EB", bg="#FFFFFF")
        self.lbl_note.pack(side=tk.LEFT, padx=(4, 14))

        tuner_info = tk.Frame(tuner_inner, bg="#FFFFFF")
        tuner_info.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.lbl_pitch_f0 = tk.Label(tuner_info, text="f₀: -- Hz  |  Ref: -- Hz", font=("Helvetica", 11, "bold"), fg="#0F172A", bg="#FFFFFF")
        self.lbl_pitch_f0.pack(anchor="w")

        self.lbl_cents = tk.Label(tuner_info, text="Deviation: 0.0 cents", font=("Helvetica", 10, "bold"), fg="#D97706", bg="#FFFFFF")
        self.lbl_cents.pack(anchor="w", pady=(2, 4))

        # High-Contrast Cents Needle Canvas
        self.gauge_canvas = tk.Canvas(card_tuner, height=38, bg="#F8FAFC", highlightthickness=1, highlightbackground="#CBD5E1")
        self.gauge_canvas.pack(fill=tk.X, pady=(4, 0))
        self._draw_tuning_gauge(0.0)

        # Card 2: ⚡ Voltage & Levels
        card_volt = make_card("⚡ Voltage & Audio Level")
        card_volt.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=4)

        self.lbl_vrms = tk.Label(card_volt, text="Vrms: 0.000 V", font=("Helvetica", 13, "bold"), fg="#2563EB", bg="#FFFFFF")
        self.lbl_vrms.pack(anchor="w")

        self.lbl_vpp = tk.Label(card_volt, text="Vpp: 0.000 V  |  DC Bias: 1.650 V", font=("Helvetica", 10, "bold"), fg="#475569", bg="#FFFFFF")
        self.lbl_vpp.pack(anchor="w", pady=2)

        self.lbl_dbfs = tk.Label(card_volt, text="Sound Level: -80.0 dBFS", font=("Helvetica", 10, "bold"), fg="#B45309", bg="#FFFFFF")
        self.lbl_dbfs.pack(anchor="w")

        self.lbl_source_text = tk.Label(card_volt, text="Source: Simulator (Mic)", font=("Helvetica", 9), fg="#64748B", bg="#FFFFFF")
        self.lbl_source_text.pack(anchor="w", pady=(2, 0))

        # Card 3: 📊 Spectral Features & Purity
        card_dsp = make_card("📊 Spectral Features & Purity")
        card_dsp.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=4)

        self.lbl_peak_f = tk.Label(card_dsp, text="Dominant: 0.0 Hz", font=("Helvetica", 12, "bold"), fg="#0F172A", bg="#FFFFFF")
        self.lbl_peak_f.pack(anchor="w")

        self.lbl_centroid = tk.Label(card_dsp, text="Spectral Centroid: 0 Hz (Brightness)", font=("Helvetica", 10, "bold"), fg="#475569", bg="#FFFFFF")
        self.lbl_centroid.pack(anchor="w", pady=2)

        self.lbl_snr = tk.Label(card_dsp, text="SNR: 0.0 dB  |  ZCR: 0.000", font=("Helvetica", 10, "bold"), fg="#475569", bg="#FFFFFF")
        self.lbl_snr.pack(anchor="w")

        self.lbl_diag = tk.Label(card_dsp, text="FPGA Overruns: 0  |  Loss: 0", font=("Helvetica", 9), fg="#64748B", bg="#FFFFFF")
        self.lbl_diag.pack(anchor="w", pady=(2, 0))

        # Card 4: 👏 Smart Event Classifier
        card_event = make_card("👏 Acoustic Event Classifier")
        card_event.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=4)

        self.lbl_event_state = tk.Label(card_event, text="SILENCE", font=("Helvetica", 14, "bold"),
                                        fg="#0F172A", bg="#E2E8F0", padx=10, pady=3, relief="solid", bd=1)
        self.lbl_event_state.pack(anchor="w", pady=(0, 4))

        clap_row = tk.Frame(card_event, bg="#FFFFFF")
        clap_row.pack(fill=tk.X, pady=2)

        self.lbl_clap_count = tk.Label(clap_row, text="Clap Counter: 0", font=("Helvetica", 11, "bold"), fg="#EA580C", bg="#FFFFFF")
        self.lbl_clap_count.pack(side=tk.LEFT)

        btn_reset_claps = HighContrastButton(
            clap_row, text="↺ Reset", font=("Helvetica", 9, "bold"),
            fg="#FFFFFF", bg="#EA580C",
            padx=8, pady=2, bd=2, command=self._reset_claps
        )
        btn_reset_claps.pack(side=tk.RIGHT)

        self.lbl_fs_meas = tk.Label(card_event, text="Measured Fs: 16000.0 S/s", font=("Helvetica", 9), fg="#64748B", bg="#FFFFFF")
        self.lbl_fs_meas.pack(anchor="w", pady=(3, 0))

    def _reset_claps(self):
        self.clap_counter = 0
        self.lbl_clap_count.config(text="Clap Counter: 0")

    def _draw_tuning_gauge(self, cents: float):
        self.gauge_canvas.delete("all")
        w = self.gauge_canvas.winfo_width()
        if w < 20:
            w = 260
        h = 38
        cx = w / 2

        # Scale line
        self.gauge_canvas.create_line(15, h / 2, w - 15, h / 2, fill="#94A3B8", width=2)
        self.gauge_canvas.create_line(cx, 4, cx, h - 4, fill="#2563EB", width=2)
        self.gauge_canvas.create_text(cx, h - 6, text="0", fill="#2563EB", font=("Helvetica", 8, "bold"))
        self.gauge_canvas.create_text(25, h - 6, text="-50c", fill="#64748B", font=("Helvetica", 8))
        self.gauge_canvas.create_text(w - 25, h - 6, text="+50c", fill="#64748B", font=("Helvetica", 8))

        # Needle
        clamped = max(-50.0, min(50.0, cents))
        nx = cx + (clamped / 50.0) * (cx - 30)

        color = "#16A34A" if abs(cents) <= 5.0 else ("#D97706" if abs(cents) <= 20.0 else "#DC2626")
        self.gauge_canvas.create_oval(nx - 5, h / 2 - 5, nx + 5, h / 2 + 5, fill=color, outline="#0F172A", width=1.5)
        self.gauge_canvas.create_line(nx, 3, nx, h - 6, fill=color, width=3)

    # -------------------------------------------------------------------------
    # 4. Ultra-Clear High-Contrast Matplotlib Plots
    # -------------------------------------------------------------------------
    def _build_plots(self):
        container = tk.Frame(self.root, bg="#FFFFFF", highlightbackground="#CBD5E1", highlightthickness=1)
        container.pack(fill=tk.BOTH, expand=True, padx=14, pady=(4, 12))

        self.fig, (self.ax_time, self.ax_fft, self.ax_spec) = plt.subplots(
            3, 1, figsize=(11.5, 6.4), facecolor="#FFFFFF"
        )
        self.fig.subplots_adjust(top=0.94, bottom=0.08, left=0.065, right=0.92, hspace=0.45)

        # 1. Waveform Plot
        self.ax_time.set_facecolor("#FFFFFF")
        (self.line_time,) = self.ax_time.plot(self.time_axis_ms, np.full(N, 1.65), color="#2563EB", lw=1.8, label="Audio Signal v(t)")
        self.ax_time.axhline(1.65, color="#F97316", linestyle="--", lw=1.2, alpha=0.8, label="Quiescent DC Bias (~1.65V)")
        self.ax_time.set_title("1. Time-Domain Waveform (MAX 10 FPGA ADC @ 16 kS/s, N = 1024, Window = 64.0 ms)",
                               fontsize=11, fontweight="bold", color="#0F172A", pad=5)
        self.ax_time.set_xlabel("Time (ms)", fontsize=10, fontweight="bold", color="#1E293B")
        self.ax_time.set_ylabel("Voltage (V)", fontsize=10, fontweight="bold", color="#1E293B")
        self.ax_time.set_ylim(-0.1, 3.4)
        self.ax_time.set_xlim(0, max(self.time_axis_ms))
        self.ax_time.grid(True, color="#E2E8F0", linestyle="-", linewidth=0.8)
        self.ax_time.tick_params(colors="#0F172A", labelsize=9)
        self.ax_time.legend(loc="upper right", fontsize=8.5, framealpha=0.95)

        # 2. FFT Spectrum Plot
        self.ax_fft.set_facecolor("#FFFFFF")
        (self.line_fft_rect,) = self.ax_fft.plot(self.freq_axis, np.zeros(len(self.freq_axis)),
                                                color="#DC2626", lw=1.2, linestyle="--", alpha=0.7,
                                                label="Without Window (Rectangular)")
        (self.line_fft_hann,) = self.ax_fft.plot(self.freq_axis, np.zeros(len(self.freq_axis)),
                                                color="#16A34A", lw=2.0, label="With Periodic Hann Window (Scaled)")
        (self.peak_marker,) = self.ax_fft.plot([0], [0], "rv", markersize=8, label="Peak Tone")
        self.peak_text = self.ax_fft.text(400, 1.4, "", fontsize=10, fontweight="bold", color="#B91C1C",
                                          bbox=dict(boxstyle="round,pad=0.3", fc="#FEF2F2", ec="#F87171", lw=1))
        self.ax_fft.set_title("2. Single-Sided FFT Magnitude Spectrum (Δf = 15.625 Hz, Bandwidth: 0 - 8000 Hz)",
                              fontsize=11, fontweight="bold", color="#0F172A", pad=5)
        self.ax_fft.set_xlabel("Frequency (Hz)", fontsize=10, fontweight="bold", color="#1E293B")
        self.ax_fft.set_ylabel("Magnitude (V)", fontsize=10, fontweight="bold", color="#1E293B")
        self.ax_fft.set_xlim(0, FS / 2)
        self.ax_fft.set_ylim(0, 1.8)
        self.ax_fft.grid(True, color="#E2E8F0", linestyle="-", linewidth=0.8)
        self.ax_fft.tick_params(colors="#0F172A", labelsize=9)
        self.ax_fft.legend(loc="upper right", fontsize=8.5, framealpha=0.95)

        # 3. Rolling Spectrogram Waterfall
        self.ax_spec.set_facecolor("#FFFFFF")
        extent = [-self.spec_history_blocks * (N / FS), 0, 0, (FS / 2) / 1000.0]
        self.img_spec = self.ax_spec.imshow(
            self.spec_matrix, aspect="auto", origin="lower", extent=extent,
            cmap="turbo", vmin=-60.0, vmax=0.0, interpolation="nearest"
        )
        self.ax_spec.set_title("3. Real-Time Spectrogram Waterfall (dBFS History)",
                               fontsize=11, fontweight="bold", color="#0F172A", pad=5)
        self.ax_spec.set_xlabel("Time Relative to Present (s)", fontsize=10, fontweight="bold", color="#1E293B")
        self.ax_spec.set_ylabel("Frequency (kHz)", fontsize=10, fontweight="bold", color="#1E293B")
        self.ax_spec.tick_params(colors="#0F172A", labelsize=9)

        cbar_ax = self.fig.add_axes([0.93, 0.08, 0.015, 0.22])
        self.cbar = self.fig.colorbar(self.img_spec, cax=cbar_ax)
        self.cbar.set_label("dBFS", fontsize=8, color="#1E293B", fontweight="bold")
        self.cbar.ax.tick_params(labelsize=8, colors="#1E293B")

        self.canvas = FigureCanvasTkAgg(self.fig, master=container)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    # -------------------------------------------------------------------------
    # 5. Streaming & Event Handlers
    # -------------------------------------------------------------------------
    def toggle_stream(self):
        if not self.is_running:
            mode = self.source_mode_var.get()
            try:
                if "Live Hardware" in mode:
                    port = self.port_var.get().strip()
                    if not port or port == "No device found":
                        messagebox.showerror("Port Error", "Please select or type a valid serial port.")
                        return
                    self.source = SerialSource(port=port, baud=DEFAULT_BAUD)
                elif "1kHz Tone" in mode:
                    self.source = SimSource(mode="tone", freq=1000.0, fs_true=FS)
                elif "DDS Test Tone" in mode:
                    self.source = SimSource(mode="dds", freq=1000.0, fs_true=FS)
                else:
                    self.source = SimSource(mode="mic", freq=440.0, fs_true=FS)

                self.source.start()
                self.is_running = True
                self.is_frozen = False
                self.stream_stats = StreamStats()

                self.btn_stream.config(text="■ STOP STREAM", bg="#DC2626", fg="#FFFFFF")
                self.status_badge.config(text="● STREAMING LIVE", fg="#22C55E", bg="#1E293B")
            except Exception as e:
                messagebox.showerror("Connection Error", f"Failed to initialize stream:\n{e}")
        else:
            if self.source:
                self.source.stop()
                self.source = None
            if self.wav_file:
                self.toggle_recording()
            self.is_running = False
            self.btn_stream.config(text="▶ START STREAM", bg="#16A34A", fg="#FFFFFF")
            self.status_badge.config(text="● STREAM STOPPED", fg="#EF4444", bg="#1E293B")

    def toggle_freeze(self):
        self.is_frozen = not self.is_frozen
        if self.is_frozen:
            self.btn_freeze.config(text="▶ RESUME", bg="#16A34A", fg="#FFFFFF")
            self.status_badge.config(text="❚❚ DISPLAY FROZEN", fg="#F59E0B", bg="#1E293B")
        else:
            self.btn_freeze.config(text="⏸ FREEZE", bg="#EA580C", fg="#FFFFFF")
            self.status_badge.config(text="● STREAMING LIVE" if self.is_running else "● STREAM STOPPED",
                                     fg="#22C55E" if self.is_running else "#EF4444", bg="#1E293B")

    def toggle_zoom(self):
        self.auto_zoom = not self.auto_zoom
        if self.auto_zoom:
            self.btn_zoom.config(text="🔍 AUTO-ZOOM: ON", bg="#0284C7", fg="#FFFFFF")
        else:
            self.btn_zoom.config(text="🔍 FULL-SCALE (0-3.3V)", bg="#334155", fg="#FFFFFF")
            self.ax_time.set_ylim(-0.1, 3.4)
            self.ax_fft.set_ylim(0, 1.8)
            self.canvas.draw_idle()

    def toggle_recording(self):
        if self.wav_file is None:
            stamp = time.strftime("%Y%m%d_%H%M%S")
            self.wav_filename = f"task3_rec_{stamp}.wav"
            self.wav_file = wave.open(self.wav_filename, "wb")
            self.wav_file.setnchannels(1)
            self.wav_file.setsampwidth(2)
            self.wav_file.setframerate(FS)
            self.btn_rec.config(text="■ STOP REC", bg="#DC2626", fg="#FFFFFF")
        else:
            self.wav_file.close()
            self.wav_file = None
            self.btn_rec.config(text="● REC WAV", bg="#7C3AED", fg="#FFFFFF")
            messagebox.showinfo("Recording Saved", f"Audio stream saved to:\n{self.wav_filename}")

    def save_snapshot(self):
        if self.last_block is None:
            messagebox.showwarning("Snapshot", "No audio block available to capture.")
            return
        stamp = time.strftime("%Y%m%d_%H%M%S")
        img_file = f"task3_snapshot_{stamp}.png"
        npz_file = f"task3_snapshot_{stamp}.npz"
        self.fig.savefig(img_file, dpi=160, facecolor="#FFFFFF")
        np.savez(npz_file, samples=self.last_block.samples, seq=self.last_block.seq,
                 status=self.last_block.status, fs=FS)
        messagebox.showinfo("Snapshot Saved", f"Successfully saved:\n{img_file}\n{npz_file}")

    # -------------------------------------------------------------------------
    # 6. Real-Time Processing Loop
    # -------------------------------------------------------------------------
    def _ui_update_loop(self):
        if self.is_running and self.source:
            blocks = self.source.get_all()
            if self.source.error:
                messagebox.showerror("Stream Error", f"Stream error occurred:\n{self.source.error}")
                self.toggle_stream()
                return

            for b in blocks:
                self.stream_stats.update(b)
                if self.wav_file is not None:
                    pcm = ((b.samples.astype(np.int32) - ADC_MID) << 4).astype("<i2")
                    self.wav_file.writeframes(pcm.tobytes())

                # Spectrogram Waterfall update
                x_ac = b.samples.astype(np.float64) - np.mean(b.samples)
                fft_c = np.fft.rfft(x_ac * self.hann_window)
                spec_m = np.abs(fft_c) / (N / 2)
                spec_db = 20 * np.log10(np.maximum(spec_m / (2048.0), 1e-4))
                self.spec_matrix = np.roll(self.spec_matrix, -1, axis=1)
                self.spec_matrix[:, -1] = spec_db

            if blocks and not self.is_frozen:
                latest = blocks[-1]
                self.last_block = latest
                self._render_frame(latest)

        self.root.after(33, self._ui_update_loop)

    def _render_frame(self, block):
        raw = block.samples.astype(np.float64)
        vref = 5.0 if block.max10_adc else 3.3
        volts = raw * (vref / (1 << ADC_BITS))
        v_ac = volts - np.mean(volts)

        # 1. Pitch & Tuner
        f0 = compute_hps_pitch(raw, FS)
        note_str, target_f, cents = freq_to_note(f0)

        in_tune = f0 > 0 and abs(cents) <= 5.0
        self.lbl_note.config(
            text=note_str if f0 > 0 else "---",
            fg="#16A34A" if in_tune else ("#2563EB" if f0 > 0 else "#94A3B8")
        )
        self.lbl_pitch_f0.config(
            text=f"Pitch: {f0:.1f} Hz  |  Ref: {target_f:.1f} Hz" if f0 > 0 else "Pitch: -- Hz  |  Ref: -- Hz"
        )
        self.lbl_cents.config(
            text=f"Deviation: {cents:+.1f} cents [{'IN TUNE' if in_tune else ('FLAT' if cents < 0 else 'SHARP')}]"
            if f0 > 0 else "Deviation: 0.0 cents [NO PITCH]",
            fg="#16A34A" if in_tune else ("#D97706" if abs(cents) <= 20.0 else "#DC2626")
        )
        self._draw_tuning_gauge(cents if f0 > 0 else 0.0)

        # 2. Voltage Levels
        vpp = float(np.ptp(volts))
        vrms = float(np.sqrt(np.mean(v_ac ** 2)))
        dbfs = 20 * math.log10(max(vrms, 1e-6) / (vref / (2 * math.sqrt(2))))

        self.lbl_vrms.config(text=f"Vrms: {vrms:.4f} V")
        self.lbl_vpp.config(text=f"Vpp: {vpp:.3f} V  |  DC Bias: {np.mean(volts):.3f} V")
        self.lbl_dbfs.config(text=f"Sound Level: {dbfs:.1f} dBFS")
        self.lbl_source_text.config(text=f"Source: {block.source_text()}")

        # 3. FFT Analysis
        x_norm = v_ac
        fft_rect_m = np.abs(np.fft.rfft(x_norm)) / (N / 2)
        fft_rect_m[0] = 0.0
        fft_hann_m = np.abs(np.fft.rfft(x_norm * self.hann_window * self.win_scale)) / (N / 2)
        fft_hann_m[0] = 0.0

        peak_idx = int(np.argmax(fft_hann_m[2:])) + 2
        f_dom = float(self.freq_axis[peak_idx])
        peak_v = float(fft_hann_m[peak_idx])

        # Spectral Centroid & ZCR
        centroid = float(np.sum(self.freq_axis * fft_hann_m) / max(np.sum(fft_hann_m), 1e-9))
        zcr = float(np.mean(np.abs(np.diff(np.sign(v_ac)))) / 2.0)

        # SNR estimate
        p_sig = float(np.sum(fft_hann_m[max(0, peak_idx - 2):peak_idx + 3] ** 2))
        p_total = float(np.sum(fft_hann_m[2:] ** 2))
        p_noise = max(1e-12, p_total - p_sig)
        snr_est = 10 * math.log10(max(p_sig / p_noise, 1e-6))

        self.lbl_peak_f.config(text=f"Dominant: {f_dom:.1f} Hz (Peak: {peak_v:.3f} V)")
        self.lbl_centroid.config(text=f"Spectral Centroid: {centroid:.0f} Hz (Brightness)")
        self.lbl_snr.config(text=f"SNR: {snr_est:.1f} dB  |  ZCR: {zcr:.3f}")
        self.lbl_diag.config(text=f"FPGA Overruns: {block.overruns}  |  Packets: #{block.seq}")

        # 4. Classifier State
        energy = vrms ** 2
        if self.clap_cooldown > 0:
            self.clap_cooldown -= 1
        elif vpp > 0.8 and (energy / max(self.prev_energy, 1e-5)) > 5.0 and zcr > 0.15:
            self.clap_counter += 1
            self.lbl_clap_count.config(text=f"Clap Counter: {self.clap_counter}")
            self.lbl_event_state.config(text="👏 HAND CLAP DETECTED!", bg="#FEE2E2", fg="#B91C1C")
            self.clap_cooldown = 4
        elif vrms < 0.02:
            self.lbl_event_state.config(text="SILENCE / QUIET", bg="#E2E8F0", fg="#475569")
        elif snr_est > 18.0 and f0 > 80.0:
            self.lbl_event_state.config(text=f"🎵 MUSICAL TONE ({note_str})", bg="#DCFCE7", fg="#15803D")
        elif zcr > 0.08 and 250.0 < centroid < 3500.0:
            self.lbl_event_state.config(text="🗣 SPEECH / VOCAL", bg="#FEF3C7", fg="#B45309")
        else:
            self.lbl_event_state.config(text="🔊 AMBIENT SOUND", bg="#E0E7FF", fg="#3730A3")

        self.prev_energy = 0.8 * self.prev_energy + 0.2 * energy

        fs_meas = self.stream_stats.measured_fs()
        if fs_meas:
            self.lbl_fs_meas.config(text=f"Measured Fs: {fs_meas:.1f} S/s")

        # 5. Update Matplotlib
        self.line_time.set_ydata(volts)
        self.line_fft_rect.set_ydata(fft_rect_m)
        self.line_fft_hann.set_ydata(fft_hann_m)

        if self.auto_zoom:
            dc_val = float(np.mean(volts))
            span = max(0.08, float(vpp) * 0.75)
            self.ax_time.set_ylim(dc_val - span, dc_val + span)
            self.ax_fft.set_ylim(0, max(0.12, float(peak_v) * 1.35))

        if peak_v > 0.05:
            self.peak_marker.set_data([f_dom], [peak_v])
            self.peak_text.set_position((min(f_dom + 150, 6800), min(peak_v + 0.15, 1.7)))
            self.peak_text.set_text(f"{f_dom:.1f} Hz\n{peak_v:.3f} V")
            self.peak_text.set_visible(True)
        else:
            self.peak_marker.set_data([], [])
            self.peak_text.set_visible(False)

        self.img_spec.set_data(self.spec_matrix)
        self.canvas.draw_idle()

def main():
    root = tk.Tk()
    app = Task3AdvancedApp(root)
    root.mainloop()

if __name__ == "__main__":
    main()
