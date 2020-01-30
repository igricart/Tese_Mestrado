% Function to calculate SNR given a signal and a noise deviation
% It may plot the signals if graph = 1
% --------------------------------------------------------------
% Data
% A 16dB SNR for data_step should have the given noise deviation [0.006 0.06 0.08 0.06]
% However the noise deviation used is proportional to quantization error

function [SNR, noisy_signal] = SNR_mod(signal,noise_const,graph)

if size(signal,2) ~= size(noise_const,2)
    error('Input dimensions are different in column size');
end

% Create noise signal
noise = noise_const.*randn(size(signal));

%------------------------------------------
% Calculate signal Energy
% Obs.:Signal avg power would be Energy/Nsamples
signalEn = rssq(signal).^2;
noiseEn  = rssq(noise).^2;

%------------------------------------------
% P_signal = calcPower(signal);
% noise = noise_const*randn(size(signal));
% P_noise = calcPower(noise);

% Both energies have equal number of samples, therefore one doesn't need to
% calculate SNR as power

%--------------------------
% Calculate SNR
SNR = 10 * log10(signalEn ./ noiseEn);
noisy_signal = signal + noise;

%-------------------------------
% Create a plot with raw signal and noisy signal
if graph == true
    f1 = figure();
    figure(f1);
    subplot (2,1,1);
    plot(signal);
    subplot (2,1,2);
    plot(signal+noise);
end

