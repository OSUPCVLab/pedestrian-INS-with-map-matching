%% Lowpass Butterworth Transfer Function
% Design a 6th-order lowpass Butterworth filter with a cutoff frequency of
% 300 Hz, which, for data sampled at 1000 Hz, corresponds to $0.6\pi$
% rad/sample. Plot its magnitude and phase responses. Use it to filter a
% 1000-sample random signal.

%%

fc = 300;
fs = 1000;

[b,a] = butter(6,fc/(fs/2));
freqz(b,a)

dataIn = randn(1000,1);
dataOut = filter(b,a,dataIn);