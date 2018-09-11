function FFT_controlSignal(u)

% Sampling frequency
fs = 1 / (u.Time(2) - u.Time(1)); % Hz

% Generate time and signals
t = u.Time;

f1 = figure();
f2 = figure();
for k = 1:4
    x = u.Data(:,k);
    y = x .* blackman(length(x));

    figure(f1);
    % Compute FFT
    N = 2^ceil(log2(length(x)) + 3);
    X = abs(fft(x, N));
    freq_fft = (0:(N-1))/N * fs;
    subplot(4,2,2*k-1);
    stem(freq_fft, X)
    title(['FFT from control signal ' num2str(k) ' in human gait']);
    xlim([0 100]);
    %
    Y = abs(fft(y, N));
    subplot(4,2,2*k);
    stem(freq_fft, Y)
    title(['FFT from control signal ' num2str(k) ' with blackman window']);
    xlim([0 100]);

    figure(f2);
    % Plot signals
    subplot(2,2,k);
    plot(t,x,t,y);
    title(['Control signal ' num2str(k)]);
    
    grid on;
end