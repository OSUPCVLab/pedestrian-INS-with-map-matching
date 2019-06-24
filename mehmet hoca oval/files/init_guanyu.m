clear all;
close all;

fileName = '../bolz_hall_NGIMU_2.csv';
data = csvread(fileName,1,0);
timestamp = data(:,1);
gyro_s = deg2rad(data(:,2:4)); % gyro data in sensor frame
worldGravity = 9.8;
acc_s =  data(:,5:7) * worldGravity;

n = 1600;
figure(1);
subplot(3,2,1);
plot(timestamp(1:n), acc_s((1:n),1),'k-');
grid on;
xlabel('Time (s)');
subplot(3,2,3);
plot(timestamp(1:n), acc_s((1:n),2),'k-');
grid on;
xlabel('Time (s)');
subplot(3,2,5);
plot(timestamp(1:n), acc_s((1:n),3),'k-');
grid on;
xlabel('Time (s)');
subplot(3,2,2);
plot(timestamp(1:n), gyro_s((1:n),1),'k-');
grid on;
xlabel('Time (s)');
subplot(3,2,4);
plot(timestamp(1:n), gyro_s((1:n),2),'k-');
grid on;
xlabel('Time (s)');
subplot(3,2,6);
plot(timestamp(1:n), gyro_s((1:n),3),'k-');
grid on;
xlabel('Time (s)');