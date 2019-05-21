clear;
close all;
clc;

% this code is a work done at the very beginning of the research

%accRange = 255; % [0-255] is the range of accelerometer ADXL345 placed on Razor IMU
worldGravity = 9.81; % 1g = 9.81m/s^2
GRAVITY = 256; % defined at Razor_AHRS.ino

%data = load('../sensor_data_home_parking_lot_rectangle2.txt');
%data = load('../sensor_data_home_parking_lot_zigzags.txt');

data = load('../sensor_data1.txt');
%data = load('sensor_data_caldwell_back_middle_region.txt');
n1 = length(data);
% step 1 is implemented on Razor IMU that is computing magnitude of
% accelerometer data a = sqrt(ax^2+ay^2+az^2)
%accMag = (data(:,1) / accRange) * worldGravity;
accMag = data(:,1) * worldGravity;
gyroMag = data(:,2);
G_Dt = data(:,3);
packetCount = data(:,4);
accX = (data(:,5) / GRAVITY) * worldGravity; % wrt inertial coordinate system
accY = (data(:,6) / GRAVITY) * worldGravity; % wrt inertial coordinate system

% recover missed samples artificially by using linear interpolation
[accMag, time] = construct_missed_samples(accMag, G_Dt, packetCount);
[gyroMag, time] = construct_missed_samples(gyroMag, G_Dt, packetCount);
[accX, time] = construct_missed_samples(accX, G_Dt, packetCount);
[accY, time] = construct_missed_samples(accY, G_Dt, packetCount);
n2 = length(accMag);

% step 2: local acceleration mean and variance; red and green plots
w = 15; % temporal window size
[localVariance, localMean] = local_variance_and_mean(accMag, w);

% step 3: thresholding to detect swing and stance phases
T1 = 2; % to detect swing phase
T2 = 1; % to detect stance phase
[B1, B2] = swing_and_stance_phase_detection(localVariance, T1, T2);
% step 4: step detection
% A step is detected in sample i when a swing phase ends and stance phase starts. 
% These two conditions must be satisfied: 1) a transition from high to low acceleration
% (B1(i-1) < B1(i)), and 2) there must be at least one low acceleration detection in a 
% window of size w ahead of current sample i, i.e.: max(B2i:i+w) = T2
steps = jimenez_2009_step_detection(B1,B2,w,T2);

% the Weinberg Stride Length Estimation Algorithm
% Lowpass Butterworth Transfer Function
% Design a 4th-order lowpass Butterworth filter with a cutoff frequency of
% 3 Hz, which, for data sampled at 50 Hz, corresponds to $0.6\pi$
% rad/sample. Plot its magnitude and phase responses. Use it to filter a
% 1000-sample random signal.
fc = 3;
fs = 50;

[b,a] = butter(4,fc/(fs/2));
%figure(1);
%freqz(b,a);
accMagFiltered = filter(b,a,accMag);

K = 1.75*0.364; % Weinberg stride estimation parameter
estimatedStrideLengths = weinberg_stride_length_estimation(accMagFiltered, steps, K, w, time);

% jimenez 2010 C1, C2, C3 acceleration magnitude, local acceleration
% variance, and gyro rate magnitude criterias to detect stance phase
figure(2); clf; h = set(gcf,'position',[285 378 1096 420]); l = 1.5;
plot(time, accMag, 'k-', 'linewidth', l); hold on;
plot(time, localMean, 'r-', 'linewidth', l);
plot(time, sqrt(localVariance), 'g-','linewidth', l);
plot(time, B1, 'b-','linewidth', l);
plot(time, B2, 'm-', 'linewidth', l);
plot(time(steps), zeros(1,length(steps)), 'ro', 'markerfacecolor','r');
% plot(time, B1, 'b-','linewidth', l);
% plot(time(steps), zeros(1,length(steps)), 'ro', 'markerfacecolor','r');
hold off; grid on;
xlabel('Time (s)', 'interpreter', 'latex', 'fontsize', 24);
% a = get(gca,'XTickLabel');
set(gca,'xtick',0:1:24,'XTickLabel',{'','','2','','4','','6','','8','','',...
    '','','','14','','16','','18','','20','','22','','24'},'FontSize',16);
ylabel('$m/s^2$', 'interpreter','latex','fontsize', 20);
h = legend('$a$','$\bar{a}$','$\sigma_{a}$','$B_1$ (swing phase)',...
    '$B_2$ (stance phase)','detected step');
set(h,'Interpreter','latex','location','northwest','fontsize',15);
set(gca,'gridlinestyle','--','position',[0.0611 0.1357 0.9279 0.8429]);
axis([0 21.2 0 60]);
%title('Step detection by using only accelerometer data');

%%%%%%% WEINBERG STRIDE ESTIMATION METHOD RESULTS %%%%%%%%%%%%%%%%
% figure(3); set(gcf,'position',[11 378 1567 420]);
% plot(time, accMag, 'k-');
% grid on;
% xlabel('Time (s)'); ylabel('acceleration magnitude (m/s^2)');
% hold on;
% plot(time, accMagFiltered, 'r-');
% plot(time(steps), zeros(1,length(steps)), 'ro', 'markerfacecolor','r');
% hold off;
% h = legend('$a_i$','$\tilde{a}_i$','detected step');
% set(h,'Interpreter','latex');
% set(gca,'gridlinestyle','--');
% title('Weinberg stride estimation process');

% Gyro data - used to determine stance phase along with accelerometer data
figure(4); clf; set(gcf,'position',[285 378 1096 420]); l = 2;
plot(time, gyroMag, 'k-', 'linewidth', l); hold on;
plot(time, ones(size(time))*(30/180)*pi, 'r--', 'linewidth', l); hold off;
grid on;
xlabel('Time (s)','interpreter', 'latex', 'fontsize', 22);
ylabel('rad/s','interpreter','latex','fontsize',22);
set(gca,'gridlinestyle','--','position',[0.039 0.110 0.950 0.815]);
set(gca,'xtick',0:1:24,'XTickLabel',{'','','2','','4','','6','','8','','',...
    '','12','','14','','16','','18','','20','','22','','24'},'FontSize',16);
% set(gca,'xtick', 'XTickLabel', [0:2:24], 'FontSize', 16);
h = legend('$\omega$','$T_{\omega}$');
set(h,'Interpreter','latex','location','northwest','fontsize',20);
set(gca,'gridlinestyle','--','position',[[0.0611 0.1357 0.9279 0.8429]]);
axis([0 21.2 0 14]);

fprintf('-------- Jimenez 2010 stance detection ---------\n');
C1 = compute_C1(accMag, 9, 11);
C2 = compute_C2(localVariance, 3);
C3 = compute_C3(gyroMag, (30/180)*pi);
stance = and_all_conditions(C1, C2, C3);
stanceMedian = median_filter(stance, 11);
stanceMedian(end-50:end) = ones(51,1);
stanceEnd = stance_phase_end_samples(stanceMedian);
% compute middle sample in swing phase: we will use it to determine heading measurement
middleSample = round((steps + stanceEnd) ./ 2) - 5;
jimenez2010steps = jimenez_2010_step_detection(stanceMedian);

figure(5); clf; set(gcf,'position',[11 378 1567 420]);
plot(time, 4.4+C1, 'r-','linewidth',l);
grid on; hold on;
plot(time, 3.3+C2, 'b-','linewidth',l); plot(time, 2.2+C3, 'g-','linewidth',l);
plot(time, 1.1+stance, 'c-','linewidth',l); plot(time, stanceMedian, 'k-','linewidth',l);
plot(time(jimenez2010steps), zeros(1,length(jimenez2010steps)), 'ro', 'markerfacecolor','r','linewidth',l);
hold off;
set(gca, 'XTickLabel', [0:2:24], 'FontSize', 16);
%title('Conditions for Stance & Still state detection (Jimenez 2010)');
set(gca,'gridlinestyle','--','position',[0.039 0.110 0.950 0.815]);
set(gca,'xtick',0:1:24,'XTickLabel',{'','','2','','4','','6','','8','','',...
    '','12','','14','','16','','18','','20','','22','','24'},'FontSize',16);
xlabel('Time (s)','interpreter','latex','fontsize',26);
h = legend('$C_1$ (a)','$C_2$ ($\sigma_{a})$','$C_3$ ($\omega$)',...
    '$C_1 \& C_2 \& C_3$','median filtered','detected steps');
set(h,'interpreter','latex', 'fontsize',22,'location','northwest');
axis([0 21.2 0 6]);

fprintf('------------------\n   Raul Feliz 2009 - ZUPT\n');
fprintf('Raul Feliz 2009 paper explicitly describes the implementation of ZUPT\n');
fprintf('Correct determination of stance and swing phases has direct impact on\n');
fprintf('accurate distance measurement.\n');
% ACCELERATIONS in x and y direction
figure(6); clf; set(gcf,'position',[285 378 1095 420]);
subplot(211);
plot(time,accX,'k-','linewidth',1.7);
grid on; set(gca,'gridlinestyle','--');
set(gca, 'YTickLabel', {'','-20','0','20','40'}, 'FontSize', 14);
set(gca, 'XTickLabel', {'','','','','','','','','','','',''}, 'FontSize', 14);
ylabel('$m/s^2$','interpreter','latex','fontsize',18);
h = legend('$a_x$');
set(h,'interpreter','latex','fontsize',20,'location','northwest');
% set(h,'position',[0.0639    0.5571    0.9269    0.4119]);
axis([0 21.2 -40 40]); set(gca,'position',[0.0639 0.5571+0.0071 0.9269 0.4119]);
subplot(212);
plot(time,accY,'k-','linewidth',1.7);
grid on; set(gca,'gridlinestyle','--');
set(gca, 'XTickLabel', [0:2:24], 'FontSize', 14);
set(gca, 'YTickLabel', {'-40','-20','0','20',''}, 'FontSize', 14);
xlabel('Time (s)','interpreter','latex','fontsize',22);
ylabel('$m/s^2$','interpreter','latex','fontsize',18);
h = legend('$a_y$');
set(h,'interpreter','latex', 'fontsize',20,'location','northwest');
% set(h,'position',[0.0639    0.1571    0.9269    0.4048]);
axis([0 21.2 -40 40]); set(gca,'position',[0.0639 0.1571 0.9269 0.4119]);

velXzupt = ZUPT(accX, stanceMedian, time);
velYzupt = ZUPT(accY, stanceMedian, time);
velX = integrate(accX, time);
velY = integrate(accY, time);
posX = -integrate(velXzupt, time);
posY = -integrate(velYzupt, time);
distance = total_distance_calculation(posX, posY);

% raw velocities and ZUPT corrected velocities in x and y directions
figure(7); clf; set(gcf,'position',[285 378 1095 420]);
subplot(2,1,1);
plot(time, velX,'b-','linewidth', 1.7); hold on;
grid on; set(gca,'gridlinestyle','--','position',[0.039 0.540 0.950 0.415]);
plot(time, velXzupt, 'r-','linewidth', 1.7); hold off;
ylabel('$m/s$','interpreter','latex','fontsize',20);
h = legend('$v_x$','ZUPT aided $v_x$');
set(h,'interpreter','latex','location','southwest','fontsize',18);
set(gca, 'YTickLabel', {'','-10','5','0','5'}, 'FontSize', 14);
set(gca, 'XTickLabel', {'','','','','','','','','','','',''}, 'FontSize', 14);
axis([0 21.2 -15 5]); set(gca,'position',[0.0639 0.5571+0.0071 0.9269 0.4119])
subplot(2,1,2);
plot(time,velY,'b-','linewidth', 1.7); hold on;
grid on; set(gca,'gridlinestyle','--','position',[0.039 0.130 0.950 0.415]);
plot(time, velYzupt, 'r-','linewidth', 1.7); hold off;
xlabel('Time (s)','interpreter','latex','fontsize',22);
ylabel('$m/s$','interpreter','latex','fontsize',20);
h = legend('$v_y$','ZUPT aided $v_y$');
set(h,'interpreter','latex','location','southwest','fontsize',18);
set(gca, 'XTickLabel', [0:2:24], 'FontSize', 14);
set(gca, 'YTickLabel', {'-15','-10','-5','0',''}, 'FontSize', 14);
axis([0 21.2 -15 5]); set(gca,'position',[0.0639 0.1571 0.9269 0.4119])


% raw velocities and ZUPT corrected velocities in x and y directions
figure(8); set(gcf,'position',[285 378 1095 420]);
subplot(2,1,1);
plot(time, velX,'b-','linewidth', 1.7); hold on;
grid on; set(gca,'gridlinestyle','--','position',[0.039 0.540 0.950 0.415]);
ylabel('m/s','interpreter','latex','fontsize',18);
h = legend('velocity x');
set(h,'interpreter','latex','location','southwest','fontsize',15);
set(gca, 'YTickLabel', {'','-10','5','0','5'}, 'FontSize', 14);
set(gca, 'XTickLabel', {'','','','','','','','','','','',''}, 'FontSize', 14);
axis([0 21.2 -15 5]); set(gca,'position',[0.0639    0.5357    0.9269    0.4121])
subplot(2,1,2);
plot(time,velY,'b-','linewidth', 1.7); hold on;
grid on; set(gca,'gridlinestyle','--','position',[0.039 0.130 0.950 0.415]);
xlabel('Time (s)','interpreter','latex','fontsize',18);
ylabel('m/s','interpreter','latex','fontsize',18);
h = legend('velocity y');
set(h,'interpreter','latex','location','southwest','fontsize',15);
set(gca, 'XTickLabel', [0:2:24], 'FontSize', 14);
set(gca, 'YTickLabel', {'-15','-10','-5','0',''}, 'FontSize', 14);
axis([0 21.2 -15 5]); set(gca,'position',[0.0639    0.1286    0.9269    0.4121])