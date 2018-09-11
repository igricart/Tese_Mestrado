% Sampling frequency
fs = 1 / (data_steps(2,1) - data_steps(1,1)); % Hz

f1 = figure();
f2 = figure();
for k = 1:4
    x_r = data_steps(:,k+1);
    y = x .* blackman(length(x));
    
    figure(f1);
    % Compute Low pass filter response
    N = 2^ceil(log2(length(x)) + 3);
    X = abs(fft(x, N));
    freq_fft = (0:(N-1))/N * fs;
    subplot(4,2,2*k-1);
    stem(freq_fft, X)
    xlim([0 30]);
    %
    Y = abs(fft(y, N));
    subplot(4,2,2*k);
    stem(freq_fft, Y)
    xlim([0 30]);

    figure(f2);
    % Plot signals
    subplot(2,2,k);
    plot(t,x,t,y);
end