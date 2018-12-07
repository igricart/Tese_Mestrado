function FFT_signal(u)

% Sampling frequency
fs = 1 / (data_steps(2,1) - data_steps(1,1)); % Hz

% Generate time and signals
t = data_steps(:, 1);

f1 = figure();
f2 = figure();

x = u;
y = x .* blackman(length(x));

figure(f1);
% Compute FFT
N = 2^ceil(log2(length(x)) + 3);
X = abs(fft(x, N));
freq_fft = (0:(N-1))/N * fs;
subplot(4,2,2);
stem(freq_fft, X)
title(['FFT from control signal']);
xlim([0 20]);
%
Y = abs(fft(y, N));
subplot(4,2,2);
stem(freq_fft, Y)
title(['FFT from control signal with blackman window']);
xlim([0 20]);

figure(f2);
% Plot signals
subplot(2,2,2);
plot(t,x,t,y);
end