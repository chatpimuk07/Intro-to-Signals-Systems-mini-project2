%% =========================================================================
%% Signals & Systems / Digital Signal Processing Practical Labs
%% Task 1: Real-Time Audio Sampling & FFT Visualization (MATLAB)
%% =========================================================================
% Requirements:
%  - Sampling frequency: Fs = 10,000 Hz (10 kS/s)
%  - Block size: N = 1024 samples
%  - Binary stream over USB-Serial at 2,000,000 baud
%  - Header: [0xAA, 0x55, 0xAA, 0x55] (4 bytes), sequence (uint16), N (uint16)
%  - Real-time time-domain waveform, Hann windowing, and FFT magnitude spectrum
%% =========================================================================
clear; clc; close all;

%% User Configuration
portList = serialportlist("available");
disp("Available Serial Ports:");
disp(portList);
if isempty(portList)
    error("No serial ports found. Please connect ESP32 to USB.");
end

% Set port name: prioritize USB serial devices on macOS/Windows/Linux
usbIdx = find(contains(portList, "usb", "IgnoreCase", true) & ...
             ~contains(portList, "bluetooth", "IgnoreCase", true) & ...
             ~contains(portList, "debug-console", "IgnoreCase", true));
if ~isempty(usbIdx)
    serialPortName = portList(usbIdx(1));
else
    serialPortName = portList(1);
end
fprintf("Selected Port: %s\n", serialPortName);

baudRate = 2000000; % 2 Mbaud (must match ESP32 sketch)
Fs = 10000;         % Sampling rate: 10 kS/s
N  = 1024;          % Samples per block
Vref = 3.3;         % ESP32 ADC reference voltage (approx 3.3V)
ADC_MAX = 4095;     % 12-bit ADC max value

% Time and Frequency axes
t = (0:N-1) / Fs * 1000;     % Time axis in milliseconds
f = (0:N/2) * (Fs / N);       % Frequency axis for one-sided FFT (0 to Fs/2)
freqResolution = Fs / N;      % Frequency resolution = 10000 / 1024 = 9.7656 Hz
fprintf("FFT Frequency Resolution: %.4f Hz\n", freqResolution);

% Create Hann window (Column Vector: 1024 x 1)
hannWin = 0.5 * (1 - cos(2 * pi * (0:N-1)' / N));
winCorrection = 1 / mean(hannWin);

%% Open Serial Connection
fprintf("Opening serial connection on %s at %d baud...\n", serialPortName, baudRate);
try
    % Specify timeout directly in constructor to avoid configuration errors
    s = serialport(serialPortName, baudRate, "Timeout", 2);
    flush(s);
catch ME
    error("Failed to open serial port: %s", ME.message);
end

%% Setup Real-time Figure
fig = figure('Name', 'Task 1: ESP32 Audio Real-Time Sampling & FFT', ...
             'NumberTitle', 'off', 'Color', 'w', 'Position', [100, 100, 1000, 700]);

% Subplot 1: Time-Domain Waveform
subplot(2, 1, 1);
hWaveform = plot(t, zeros(N, 1), 'b-', 'LineWidth', 1.2);
grid on;
title(sprintf('Time-Domain Audio Signal (Fs = %d Hz, N = %d)', Fs, N), 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time (ms)', 'FontSize', 10);
ylabel('Voltage (V)', 'FontSize', 10);
ylim([0, 3.3]);
xlim([0, max(t)]);

% Subplot 2: Frequency Spectrum (FFT)
subplot(2, 1, 2);
hFFT_raw = plot(f, zeros(length(f), 1), 'r--', 'LineWidth', 1.0, 'DisplayName', 'Without Window (Rectangular)');
hold on;
hFFT_win = plot(f, zeros(length(f), 1), 'b-',  'LineWidth', 1.5, 'DisplayName', 'With Hann Window');
grid on;
title('Single-Sided FFT Magnitude Spectrum', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Frequency (Hz)', 'FontSize', 10);
ylabel('Magnitude (V)', 'FontSize', 10);
xlim([0, Fs/2]);
ylim([0, 1.5]);
legend('Location', 'northeast');

% Text display for peak dominant frequency
hPeakText = text(Fs/4, 1.2, '', 'FontSize', 11, 'FontWeight', 'bold', 'Color', [0.8 0 0]);

% Stop / Freeze toggle button
hStopBtn = uicontrol('Style', 'togglebutton', 'String', 'Stop / Freeze', ...
    'Units', 'normalized', 'Position', [0.42, 0.015, 0.16, 0.045], ...
    'BackgroundColor', [1 0.7 0.7], 'FontWeight', 'bold', 'FontSize', 10);

disp("Streaming started... Close the figure window or click Stop to freeze.");

%% Main Acquisition Loop
syncTarget = uint8([170, 85, 170, 85]); % [0xAA, 0x55, 0xAA, 0x55]
syncIdx = 1;
packetBytes = 4 + 2 + 2 + (N * 2);      % 2056 bytes per packet

try
    while ishandle(fig)
        % Check if user pressed Stop / Freeze
        if get(hStopBtn, 'Value') == 1
            set(hStopBtn, 'String', 'Resume', 'BackgroundColor', [0.7 1 0.7]);
            flush(s);
            drawnow;
            pause(0.05);
            continue;
        else
            set(hStopBtn, 'String', 'Stop / Freeze', 'BackgroundColor', [1 0.7 0.7]);
        end

        % Prevent lag: flush input buffer if backlog exceeds 2 packets
        if s.NumBytesAvailable > packetBytes * 2
            flush(s, "input");
        end

        % Search for 4-byte sync header
        while syncIdx <= 4
            if s.NumBytesAvailable == 0
                pause(0.002);
                continue;
            end
            b = read(s, 1, "uint8");
            if isempty(b)
                continue;
            end
            if b == syncTarget(syncIdx)
                syncIdx = syncIdx + 1;
            else
                if b == syncTarget(1)
                    syncIdx = 2;
                else
                    syncIdx = 1;
                end
            end
        end
        syncIdx = 1; % Reset for next block

        % Read Sequence number (2 bytes) and Block size (2 bytes)
        meta = read(s, 2, "uint16");
        pktSeq = meta(1);
        pktLen = meta(2);
        if pktLen ~= N
            continue; % Corrupted packet, re-sync
        end

        % Read N uint16 audio samples (2048 bytes)
        rawSamples = read(s, N, "uint16");
        if length(rawSamples) < N
            continue;
        end

        % Enforce column vector (1024 x 1)
        rawSamplesCol = double(rawSamples(:));

        % Convert raw 12-bit ADC values to Voltage (0V to 3.3V)
        voltage = rawSamplesCol * (Vref / ADC_MAX);

        % Remove DC offset for FFT analysis
        voltageAC = voltage - mean(voltage);

        % 1. FFT without windowing
        Y_raw = fft(voltageAC) / N;
        P1_raw = 2 * abs(Y_raw(1:N/2+1));
        P1_raw(1) = P1_raw(1) / 2; % DC component

        % 2. FFT with Hann windowing (column vector 1024 x 1 matching)
        voltageWin = voltageAC .* hannWin * winCorrection;
        Y_win = fft(voltageWin) / N;
        P1_win = 2 * abs(Y_win(1:N/2+1));
        P1_win(1) = P1_win(1) / 2;

        % Find dominant peak frequency (excluding DC bin)
        [peakVal, peakIdx] = max(P1_win(2:end));
        dominantFreq = f(peakIdx + 1);

        % Update plots
        set(hWaveform, 'YData', voltage);
        set(hFFT_raw,  'YData', P1_raw);
        set(hFFT_win,  'YData', P1_win);
        set(hPeakText, 'String', sprintf('Dominant Frequency: %.1f Hz (Peak: %.3f V)', dominantFreq, peakVal));

        drawnow limitrate;
    end
catch ME
    if ishandle(fig)
        fprintf("Acquisition stopped: %s\n", ME.message);
    end
end

%% Clean up
clear s;
disp("Serial port closed.");