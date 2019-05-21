clear;
close all;
clc;

nOfExperiments = 4;
fprintf('1 for Bolz Hall with magnetometer experiment 1\n');
fprintf('2 for Bolz Hall with magnetometer experiment 2\n');
fprintf('3 for Caldwell with magnetometer experiment 1\n');
fprintf('4 for Bolz Hall without magnetometer experiment 2\n');
choice = input('What is your choice? ');

%accRange = 255; % [0-255] is the range of accelerometer ADXL345 placed on Razor IMU
worldGravity = 9.81; % 1g = 9.81m/s^2
GRAVITY = 256; % defined at Razor_AHRS.ino

txtFileNames = cell(1,nOfExperiments);
txtFileNames{1} = '../sensor_data_bolz_hall_with_magnetometer_001.txt';
txtFileNames{2} = '../sensor_data_bolz_hall_with_magnetometer_002.txt';
txtFileNames{3} = '../sensor_data_caldwell_with_magnetometer_001.txt';
txtFileNames{4} = '../sensor_data_bolz_hall_no_magnetometer_002.txt';
data = load(txtFileNames{choice});
%data = load('../sensor_data_home_parking_lot_rectangle2.txt');
%data = load('../sensor_data_home_parking_lot_zigzags.txt');

%data = load('sensor_data_home_front_8g_range.txt');
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

figure(2); set(gcf,'position',[11 378 1567 420]);
plot(time, accMag, 'k-');
hold on;
plot(time, localMean, 'r-');
plot(time, sqrt(localVariance), 'g-');
plot(time, B1, 'b-');
plot(time(steps), zeros(1,length(steps)), 'ro', 'markerfacecolor','r');
plot(time, B2, 'm-');
hold off;
grid on;
xlabel('Time (s)'); ylabel('acceleration magnitude (m/s^2)');
h = legend('$a_i$','$\bar{a}_i$','$\sigma_{a_i}$','$B_1$ (swing phase)',...
    'detected step','$B_2$ (stance phase)');
set(h,'Interpreter','latex');
set(gca,'gridlinestyle','--','position',[0.039 0.110 0.950 0.815]);
title('Step detection by using only accelerometer data');

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
figure(4); set(gcf,'position',[11 378 1567 420]);
plot(time, gyroMag, 'k-');
grid on;
xlabel('Time (s)'); ylabel('Angular velocity magnitude (rad/s)');
set(gca,'gridlinestyle','--','position',[0.039 0.110 0.950 0.815]);

fprintf('-------- Jimenez 2010 stance detection ---------\n');
C1 = compute_C1(accMag, 9, 11);
C2 = compute_C2(localVariance, 3);
C3 = compute_C3(gyroMag, (50/180)*pi);
stance = and_all_conditions(C1, C2, C3);
stanceMedian = median_filter(stance, 11);
stanceMedian(end-50:end) = ones(51,1);
stanceEnd = stance_phase_end_samples(stanceMedian);
% compute middle sample in swing phase: we will use it to determine heading measurement
middleSample = round((steps + stanceEnd) ./ 2) - 5;
jimenez2010steps = jimenez_2010_step_detection(stanceMedian);

figure(5); set(gcf,'position',[11 378 1567 420]);
plot(time, 4.4+C1, 'r-');
grid on; hold on;
plot(time, 3.3+C2, 'b-'); plot(time, 2.2+C3, 'g-');
plot(time, 1.1+stance, 'c-'); plot(time, stanceMedian, 'k-');
plot(time(jimenez2010steps), zeros(1,length(jimenez2010steps)), 'ro', 'markerfacecolor','r');
hold off;
xlabel('Time (s)');
title('Conditions for Stance & Still state detection (Jimenez 2010)');
set(gca,'gridlinestyle','--','position',[0.039 0.110 0.950 0.815]);
h = legend('$C_1$ $(a_i)$','$C_2$ ($\sigma_{a_i})$','$C_3$ ($g_i$)',...
    'stance $C_1 \& C_2 \& C_3$','stance (median)','steps');
set(h,'interpreter','latex');

fprintf('------------------\n   Raul Feliz 2009 - ZUPT\n');
fprintf('Raul Feliz 2009 paper explicitly describes the implementation of ZUPT\n');
fprintf('Correct determination of stance and swing phases has direct impact on\n');
fprintf('accurate distance measurement.\n');
figure(6); set(gcf,'position',[11 378 1567 420]);
subplot(211);
plot(time,accX,'r-');
grid on; set(gca,'gridlinestyle','--');
xlabel('Time (s)'); ylabel('m/s^2'); title('acceleration x');
subplot(212);
plot(time,accY,'b-');
grid on; set(gca,'gridlinestyle','--');
xlabel('Time (s)'); ylabel('m/s^2'); title('acceleration y');

velXzupt = ZUPT(accX, stanceMedian, time);
velYzupt = ZUPT(accY, stanceMedian, time);
velX = integrate(accX, time);
velY = integrate(accY, time);
posX = -integrate(velXzupt, time);
posY = -integrate(velYzupt, time);
distance = total_distance_calculation(posX, posY);
distanceGT = cell(1,3);
distanceGT{1} = 115.2387; distanceGT{2} = 115.2387;
distanceGT{3} = 123.0782; distanceGT{4} = 115.2387;
scaleConstant = distanceGT{choice} / sum(distance);
posX = scaleConstant * posX; posY = scaleConstant * posY;
distance = total_distance_calculation(posX, posY);
fprintf('Total traveled distance is %.3f meters.\n', sum(distance));

% raw velocities and ZUPT corrected velocities in x and y directions
figure(7); set(gcf,'position',[11 378 1567 420]);
subplot(2,1,1);
plot(time, velX,'b-'); hold on;
grid on; set(gca,'gridlinestyle','--','position',[0.039 0.540 0.950 0.415]);
plot(time, velXzupt, 'r-'); hold off;
ylabel('m/s');
legend('v_x', 'v_x with ZUPT correction');
subplot(2,1,2);
plot(time,velY,'b-'); hold on;
grid on; set(gca,'gridlinestyle','--','position',[0.039 0.130 0.950 0.415]);
plot(time, velYzupt, 'r-'); hold off;
xlabel('Time (s)'); ylabel('m/s');
legend('v_y', 'v_y with ZUPT correction');

cornerSteps = cell(1,nOfExperiments);
cornerSteps{1} = [45 55 59 74 82 97 101];
cornerSteps{2} = [43 52 56 70 79 93 98];
cornerSteps{3} = [19 33 39 52 76 104];
cornerSteps{4} = [45 57 59 70 79 99 103];

figure(8); set(figure(8),'position',[54   194   497   467]);
plot(posY, posX,'r-','linewidth',1.5);
axis equal;
hold on;
plot(posY(steps(cornerSteps{choice})), posX(steps(cornerSteps{choice})),'gx','linewidth',2);
hold off;
grid on; set(gca,'gridlinestyle','--');
xlabel('East (meters)'); ylabel('North (meters)');
title(['Total distance traveled is ' num2str(sum(distance)) ' meters.']);

figure(9); set(figure(9),'position',[54 194 497 467]);
plot(posY,posX,'r-x','linewidth',1.5);
axis equal; hold on;
for i=1:length(stanceMedian)
    if (stanceMedian(i) == 1)
        plot(posY(i), posX(i),'go');
    end
end
plot(posY(steps(cornerSteps{choice})), posX(steps(cornerSteps{choice})),'gx','linewidth',2);
plot(posY(middleSample), posX(middleSample),'yx','linewidth',2);
hold off; grid on; set(gca,'gridlinestyle','--');
xlabel('East (meters)'); ylabel('North (meters)');
title(['Total distance traveled is ' num2str(sum(distance)) ' meters.']);

% compute stride lengths
strideLength = zeros(size(steps));
strideLength(1) = sum(distance(1:steps(1)));
for i=2:length(steps)
    strideLength(i) = sum(distance(steps(i-1)+1:steps(i)));
end
strideLength(length(steps)) = sum(distance(steps(end-1)+1:end));

% plot stride lengths
figure(10); set(gcf, 'position', [520 378 782 420]);
plot(strideLength,'r');
grid on; set(gca,'gridlinestyle','--');
xlabel('strides'); ylabel('stride length (meters)');
title(['In ' num2str(length(steps)) ' steps, total distance traveled is ' ...
    num2str(sum(strideLength)) ' meters']);
set(gca,'position',[0.0691 0.1100 0.9092 0.8150]);

% compute headings by using middle sample of swing regions
strideHeading = zeros(size(steps)); % unit is radians
for i=1:length(steps)
    dy = (posX(middleSample(i)) - posX(middleSample(i)-1));
    dx = (posY(middleSample(i)) - posY(middleSample(i)-1));
%     b = (posX(middleSample(i)+1) - posX(middleSample(i))) / ...
%         (posY(middleSample(i)+1) - posY(middleSample(i)));
    if (dy > 0 && dx > 0) % region 1
        strideHeading(i) = atan(dy/dx);
    else if (dy > 0 && dx < 0) % region 2
            strideHeading(i) = atan(dy/dx) + pi;
        else if (dy < 0 && dx < 0) % region 3
                strideHeading(i) = atan(dy/dx) + pi;
            else if (dy < 0 && dx > 0)
                strideHeading(i) = atan(dy/dx) + 2*pi;
                end
            end
        end
    end
end
% compute heading change measurements for Bayesian Location Estimation Framework
strideHeadingChange = strideHeading - [strideHeading(1) strideHeading(1:end-1)];
strideHeadingChangeDegrees = 180*(strideHeadingChange/pi);

% plot the traversal of the pedestrian by using stride lengths and headings
% obtained as measurements for the Bayesian Location Estimation Framework
figure(11); % measurements
X = zeros(1,length(steps)+1);
Y = zeros(1,length(steps)+1);
X(1) = 0; Y(1) = 0;
for i=2:length(steps)+1
    X(i) = X(i-1) + strideLength(i-1)*cos(strideHeading(i-1));
    Y(i) = Y(i-1) + strideLength(i-1)*sin(strideHeading(i-1));
end
plot(X,Y,'k.-');
grid on; set(gca,'gridlinestyle','--');
xlabel('East (meter)'); ylabel('North (meter)');


buildingMaps = cell(1,nOfExperiments);
buildingMaps{1} = 'bolz_hall_2nd_floor_rotated.jpg';
buildingMaps{2} = 'bolz_hall_2nd_floor_rotated.jpg';
buildingMaps{3} = 'caldwell_2nd_floor_rotated.png';
buildingMaps{4} = 'bolz_hall_2nd_floor_rotated.jpg';
img = rgb2gray(imread(buildingMaps{choice}));
%newmap = rgb2gray(map);
height = size(img,1); width = size(img,2);
startPixelCoordinates = cell(1,3); % start pixel coordinates of the pedestrian
startPixelCoordinates{1} = [1364 + 1, 738 + 1];
startPixelCoordinates{2} = [1364 + 1, 738 + 1];
startPixelCoordinates{3} = [387 + 1, 1081 + 1];
startPixelCoordinates{4} = [1364 + 1, 738 + 1];
meter2pixelConstant = cell(1,nOfExperiments);
meter2pixelConstant{1} = 200 / 9.7536; % unit conversion constant for bolz hall traverse
meter2pixelConstant{2} = 200 / 9.7536; % unit conversion constant for bolz hall traverse
meter2pixelConstant{3} = 400 / 18.288;
meter2pixelConstant{4} = 200 / 9.7536;
%%
figure(12); clf;% pedestrian trajectory on indoor environment map
set(gcf,'position',[564 61 927 670]);
imshow(img);
hold on;
if (choice == 1 || choice == 2 || choice == 4)
% plot raw trajectory on the indoor map
plot(startPixelCoordinates{choice}(1) + meter2pixelConstant{choice}*posY, ...
    startPixelCoordinates{choice}(2) - meter2pixelConstant{choice}*posX, 'r-','linewidth',2);
plot(startPixelCoordinates{choice}(1), startPixelCoordinates{choice}(2), ...
    'ko','linewidth',2,'markerfacecolor','r','markersize',10);
plot(276, 522, 'ks','linewidth',2,'markerfacecolor','g','markersize',10);
mapMatching = input('enter 1 to show map-matching, 0 for no MM: ');
if (mapMatching)
% now plot measured distance with constraints
newX = startPixelCoordinates{choice}(1) - meter2pixelConstant{choice}*sum(distance(1:steps(cornerSteps{choice}(1))));
plot([startPixelCoordinates{choice}(1) newX], ...
    [startPixelCoordinates{choice}(2) startPixelCoordinates{choice}(2)], 'g-','linewidth',2);
newY = startPixelCoordinates{choice}(2) - ...
    meter2pixelConstant{choice}*sum(distance(steps(cornerSteps{choice}(1))+1:steps(cornerSteps{choice}(2))));
plot([newX newX], [startPixelCoordinates{choice}(2) newY], 'g-','linewidth',2);
oldX = newX;
newX = oldX - ...
    meter2pixelConstant{choice}*sum(distance(steps(cornerSteps{choice}(2))+1:steps(cornerSteps{choice}(3))));
plot([oldX newX], [newY newY], 'g-','linewidth',2);
oldY = newY;
newY = oldY - ...
    meter2pixelConstant{choice}*sum(distance(steps(cornerSteps{choice}(3))+1:steps(cornerSteps{choice}(4))));
plot([newX newX], [oldY newY], 'g-','linewidth',2);
oldX = newX;
newX = oldX + ...
    meter2pixelConstant{choice}*sum(distance(steps(cornerSteps{choice}(4))+1:steps(cornerSteps{choice}(5))));
plot([oldX newX], [newY newY], 'g-','linewidth',2);
oldY = newY;
newY = oldY + ...
    meter2pixelConstant{choice}*sum(distance(steps(cornerSteps{choice}(5))+1:steps(cornerSteps{choice}(6))));
plot([newX newX], [oldY newY], 'g-','linewidth',2);
oldX = newX;
newX = oldX - ...
    meter2pixelConstant{choice}*sum(distance(steps(cornerSteps{choice}(6))+1:steps(cornerSteps{choice}(7))));
plot([oldX newX], [newY newY], 'g-','linewidth',2);
plot(startPixelCoordinates{choice}(1), startPixelCoordinates{choice}(2), ...
    'ko','linewidth',2,'markerfacecolor','r','markersize',10);
% plot end point
plot(276, 522, 'ks','linewidth',2,'markerfacecolor','g','markersize',10);
h = legend('only ZUPT corrected','ZUPT corrected and floor plan aided', ...
    'start point', 'end point');
else
h = legend('pedestrian INS with na\"ive ZUPT','start', 'end');
end
set(h, 'interpreter','latex','fontsize',20);
end

if (choice == 3)
% plot raw trajectory on the indoor map
plot(startPixelCoordinates{choice}(1) + meter2pixelConstant{choice}*posY, ...
    startPixelCoordinates{choice}(2) - meter2pixelConstant{choice}*posX, 'r-','linewidth',2);
% plot start point
plot(startPixelCoordinates{choice}(1), startPixelCoordinates{choice}(2), ...
        'ko','linewidth',2,'markerfacecolor','r','markersize',10);
mapMatching = input('enter 1 to show map-matching, 0 for no MM: ');
if (mapMatching)
% now plot measured distance with constraints
newX = startPixelCoordinates{choice}(1) + meter2pixelConstant{choice}*sum(distance(1:steps(cornerSteps{choice}(1))));
plot([startPixelCoordinates{choice}(1) newX], ...
    [startPixelCoordinates{choice}(2) startPixelCoordinates{choice}(2)], 'g-','linewidth',2);
newY = startPixelCoordinates{choice}(2) - ...
    meter2pixelConstant{choice}*sum(distance(steps(cornerSteps{choice}(1))+1:steps(cornerSteps{choice}(2))));
plot([newX newX], [startPixelCoordinates{choice}(2) newY], 'g-','linewidth',2);
oldX = newX;
newX = oldX + ...
    meter2pixelConstant{choice}*sum(distance(steps(cornerSteps{choice}(2))+1:steps(cornerSteps{choice}(3))));
plot([oldX newX], [newY newY], 'g-','linewidth',2);
oldY = newY;
newY = oldY - ...
    meter2pixelConstant{choice}*sum(distance(steps(cornerSteps{choice}(3))+1:steps(cornerSteps{choice}(4))));
plot([newX newX], [oldY newY], 'g-','linewidth',2);
oldX = newX;
newX = oldX - ...
    meter2pixelConstant{choice}*sum(distance(steps(cornerSteps{choice}(4))+1:steps(cornerSteps{choice}(5))));
plot([oldX newX], [newY newY], 'g-','linewidth',2);
oldY = newY;
newY = oldY + ...
    meter2pixelConstant{choice}*sum(distance(steps(cornerSteps{choice}(5))+1:steps(cornerSteps{choice}(6))));
plot([newX newX], [oldY newY], 'g-','linewidth',2);
h = legend('only ZUPT corrected','ZUPT corrected and floor plan aided', 'start and end point');
else
    h = legend('pedestrian INS with na\"ive ZUPT','start and end');
end
    set(h, 'interpreter','latex','fontsize',20);
end
hold off;

%===========================================================================================
fprintf('Till now, we manually tagged corner points in ZUPT corrected trajectories,\n');
fprintf('and distances between corners are mapped to drawings manually.\n');
fprintf('Now, automate this process by using the fact that ZUPT corrected measurements\n');
fprintf('provide distances very accurately, so by exploiting the idea that we can go\n');
fprintf('only in corridors, we map the raw trajectory to walking path.\n\n');

bolzDirections = {'left', 'up', 'left', 'up', 'right', 'down', 'left'};
caldwellDirections = {'right', 'up', 'right', 'up', 'left', 'down'};
bolzSkeletonDistances = [53.15, 10.38, 4.63, 16.72, 9.12, 16.72, 4.48];
caldwellSkeletonDistances = [22.17, 16.96, 6.81, 15.59, 28.98, 32.55];

