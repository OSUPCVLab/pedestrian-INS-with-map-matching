clear all;
close all;
clc;

X = load('sensor_data.txt');
accX = X(:,4);
accY = X(:,5);
accZ = X(:,6);
timeStamps = X(:,7);
time = zeros(length(timeStamps),1);
time(1) = 0;
for i=2:length(time)
    time(i) = time(i-1) + timeStamps(i);
end

figure(1);
subplot(311);
plot(time, accX, 'k-');
grid on;
subplot(312);
plot(time, accY, 'k-');
grid on;
subplot(313);
plot(time, accZ, 'k-');
grid on;