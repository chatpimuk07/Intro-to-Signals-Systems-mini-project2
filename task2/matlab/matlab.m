% =========================================================================
% Signals & Systems / Digital Signal Processing Practical Labs
% Task 2: Real-Time Audio Sampling using ESP32 & I2S Microphone (INMP441)
% Real-Time Visualization: Time-Domain, FFT Spectrum, and Spectrogram
% =========================================================================

clear; clc; close all;

%% 1. DSP Parameters & Serial Port Configuration
portName = "/dev/cu.usbserial-58730042301"; % Serial port name (adjust for your OS/device)
baudRate = 2000000;                         % 2 Mbaud baud rate
Fs = 16000;                                 % Sampling Rate: 16 kS/s
N = 1024;                                   % Block Size: 1024 samples
syncHeader = uint8([170, 85, 170, 85]);     % 4-byte Sync Header: [0xAA, 0x55, 0xAA, 0x55]

% Time and frequency axes
t_axis = (0:N-1) / Fs * 1000;          % Time axis in ms (0 to ~63.94 ms)
f_axis = (0:N/2) * (Fs / N);           % Frequency axis in Hz (0 to 8000 Hz, 513 bins)
numBins = length(f_axis);

% Generate periodic Hann window as column vector (1024 x 1)
hann_win = 0.5 * (1 - cos(2 * pi * (0:N-1)' / N));
win_scale = 1.0 / mean(hann_win);      % Amplitude correction factor for Hann window

% Waterfall buffer for spectrogram (513 bins x 70 frames)
historyFrames = 70;
specMatrix = -80 * ones(numBins, historyFrames);
timeSpan = historyFrames * (N / Fs);   % Total visible time history in seconds

%% 2. Setup Real-time Figure & Plots
fig = figure('Name', 'Task 2: Real-Time Audio DSP (MATLAB)', ...
             'NumberTitle', 'off', 'Color', 'w', 'Position', [100, 80, 1000, 800]);

% Subplot 1: Time-Domain Waveform
subplot(3, 1, 1);
hLineTime = plot(t_axis, zeros(N, 1), 'Color', [0.1 0.45 0.76], 'LineWidth', 1.2);
title(sprintf('1. Time-Domain Waveform (Fs = %d Hz, N = %d, \\Deltat = %.1f ms)', Fs, N, N/Fs*1000), 'FontWeight', 'bold');
xlabel('Time (ms)'); ylabel('Amplitude (Norm)');
xlim([0, max(t_axis)]); ylim([-1.05, 1.05]); grid on;

% Subplot 2: FFT Magnitude Spectrum
subplot(3, 1, 2);
hLineFFT = plot(f_axis, zeros(numBins, 1), 'Color', [0.18 0.62 0.27], 'LineWidth', 1.3);
hTitleFFT = title(sprintf('2. Single-Sided FFT Spectrum (\\Deltaf = %.3f Hz)', Fs/N), 'FontWeight', 'bold');
xlabel('Frequency (Hz)'); ylabel('Magnitude (Linear)');
xlim([0, Fs/2]); ylim([0, 0.4]); grid on;

% Subplot 3: Real-Time Spectrogram Waterfall
subplot(3, 1, 3);
hImgSpec = imagesc([-timeSpan, 0], [0, (Fs/2)/1000], specMatrix);
set(gca, 'YDir', 'normal'); 
colormap('hot'); 
clim([-60, -10]); % Color range in dBFS adjusted for speech & pure tones
title('3. Real-Time Spectrogram (Power in dBFS)', 'FontWeight', 'bold');
xlabel('Time Relative to Present (s)'); ylabel('Frequency (kHz)');
hCbar = colorbar;
ylabel(hCbar, 'Magnitude (dBFS)');

%% 3. Open Serial Port Connection
fprintf('Connecting to %s at %d baud...\n', portName, baudRate);
try
    s = serialport(portName, baudRate, "Timeout", 0.5);
    flush(s);
catch ME
    error("Cannot open port: %s\nPlease check that no other application (e.g., Python / Arduino Serial Monitor) is using the port.", ME.message);
end

% Cleanup object to ensure serial port is closed when figure/script terminates
cleanupObj = onCleanup(@() delete(s));

%% 4. Zero-Delay Real-Time Acquisition & DSP Loop
fprintf('Streaming audio data... Close figure window to stop.\n');
packetSize = 4 + 2 + 2 + (N * 2); % 2056 bytes per packet

while ishandle(fig)
    % Flush OS buffer if backlog exceeds 2 packets to prevent latency/lag
    if s.NumBytesAvailable > packetSize * 2
        flush(s, "input");
    end
    
    % Search for 4-byte sync header (0xAA 0x55 0xAA 0x55)
    matched = false;
    while s.NumBytesAvailable >= 4
        b = read(s, 1, "uint8");
        if b == syncHeader(1)
            next3 = read(s, 3, "uint8");
            if isequal(next3, syncHeader(2:4))
                matched = true;
                break;
            end
        end
    end
    
    if ~matched
        pause(0.005);
        continue;
    end
    
    % Read header metadata (Sequence: 2 bytes, Block size: 2 bytes)
    meta = read(s, 2, "uint16");
    seq = meta(1);
    blockLen = meta(2);
    
    if blockLen ~= N
        continue; % Corrupted block length, re-sync
    end
    
    % Read 1024 int16 audio samples (2048 bytes)
    rawSamples = read(s, N, "int16");
    
    % --- DSP Processing ---
    % 1. Enforce column vector (1024 x 1) and normalize to [-1.0, 1.0]
    audioNorm = double(rawSamples(:)) / 32768.0;
    audioAC   = audioNorm - mean(audioNorm);
    
    % 2. Apply Hann window and calculate single-sided FFT
    audioWin   = audioAC .* hann_win * win_scale;
    fftComplex = fft(audioWin);
    fftMag     = abs(fftComplex(1:numBins)) / (N / 2);
    
    % Suppress DC bin (0 Hz)
    fftMag(1) = 0;
    
    % 3. Find dominant peak frequency
    [peakVal, maxIdx] = max(fftMag);
    domFreq = f_axis(maxIdx);
    
    % 4. Compute Spectrogram column in dBFS and scroll matrix
    fftDB = 20 * log10(max(fftMag, 1e-4));
    specMatrix(:, 1:end-1) = specMatrix(:, 2:end);
    specMatrix(:, end)     = fftDB;
    
    % --- Real-Time Graphics Update ---
    set(hLineTime, 'YData', audioNorm);
    set(hLineFFT,  'YData', fftMag);
    set(hImgSpec,  'CData', specMatrix);
    set(hTitleFFT, 'String', sprintf('2. Single-Sided FFT Spectrum (Dominant: %.1f Hz | Peak: %.3f)', domFreq, peakVal));
    
    drawnow limitrate;
end

fprintf('Serial port closed.\n');