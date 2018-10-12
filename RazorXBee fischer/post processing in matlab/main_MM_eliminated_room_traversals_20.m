% this is the code I modified from Fischer's tutorial
% Inertial pedestrian tracking.
% Accompanying article in IEEE Pervasive Magazine.

% For best results use a foot-mounted inertial measurement unit with an
% accelerometer range greater than 10g and 2a gyroscope range greater than
% 900 degrees per second and at least 50 samples per second. The IMU is NOT
% required to estimate or1ientations.

clear all;
close all;
clc;
% Read data from file for gyro bias estimation.
% Data should include timestamps (seconds), 3 axis accelerations (m/s^2), 
% 3 axis gyroscopic rates of turn (rad/s).

choice = input('Enter 1 for Bolz Hall, 2 for Caldwell, 3 for around PCV: ');
if (choice == 1)
    choice2 = input('Enter which traverse it is: ');
end
if (choice == 1)
    fileName = '../bolz_hall_gyro_data_for_bias_estimation_wired_57600_%i.txt';
    %gyroDataForBiasEstimationName = '../bolz_hall_gyro_data_for_bias_estimation_wired_57600_100Hz.txt';
    gyroDataForBiasEstimationName = sprintf(fileName, choice2);
else if (choice == 2)
    gyroDataForBiasEstimationName = '../caldwell_lab_gyro_data_for_bias_estimation_wired_57600_multiple.txt';
    %gyroDataForBiasEstimationName = '../caldwell_lab_gyro_data_for_bias_estimation_wired_57600.txt';
    else if (choice == 3)
            gyroDataForBiasEstimationName = '../around pcv lab multiple times/around_pcv_gyro_data_for_bias_estimation_wired_57600_multiple.txt';
    else
        fprintf('Invalid input!\n');
        pause();
    end
    end
end
%gyroDataForBiasEstimationName = '../bolz_hall_gyro_data_for_bias_estimation_wired_1.txt';
%gyroDataForBiasEstimationName = '../bolz_hall_gyro_data_for_bias_estimation_3.txt';
%gyroDataForBiasEstimationName = '../caldwell_lab_gyro_data_for_bias_estimation.txt';
dataTaha = load(gyroDataForBiasEstimationName);
accData = dataTaha(:,1:3);
gyroData = dataTaha(:,4:6);


nOfCapturedSamples = length(dataTaha);
G_Dt = dataTaha(:,7);
packetCount = dataTaha(:,8);
nMissedSamples = number_of_missed_samples(packetCount);
if (nMissedSamples == 0)
    fprintf('There is no missed sample in gyro bias estimation.\n');
else
    fprintf('Number of missed samples in gyro bias computation data is %i.\n', nMissedSamples);
end
% recover missed samples (due to misses in XBee) by using linear interpolation
% [gyroX, timestamp] = construct_missed_samples(gyroData(:,1), G_Dt, packetCount);
% [gyroY, timestamp] = construct_missed_samples(gyroData(:,2), G_Dt, packetCount);
% [gyroZ, timestamp] = construct_missed_samples(gyroData(:,3), G_Dt, packetCount);
% compute gyro bias from static shoe data
gyro_bias = compute_gyro_bias(gyroData(:,1), gyroData(:,2), gyroData(:,3));

if (choice == 1)
    fileName = '../bolz_hall_wired_57600_%i.txt';
    %pedestrianTraverseName = '../bolz_hall_wired_57600_100Hz.txt';
    pedestrianTraverseName = sprintf(fileName, choice2);
    %pedestrianTraverseName = '../bolz_hall_wired_57600_5.txt';
    %pedestrianTraverseName = '../bolz_hall_wired_57600.txt';
    else if (choice == 2)
        pedestrianTraverseName = '../caldwell_lab_wired_57600_multiple.txt';
        % pedestrianTraverseName = '../caldwell_lab_wired_57600.txt';
        else if (choice == 3)
                pedestrianTraverseName = '../around pcv lab multiple times/around_pcv_wired_57600_multiple.txt';
    else
        fprintf('Invalid input!\n');
        pause();
    end
        end
end
%pedestrianTraverseName = '../bolz_hall_wired_57600.txt';
%pedestrianTraverseName = '../caldwell_lab_wired_57600.txt';
% pedestrianTraverseName = '../bolz_hall_wired_1.txt';
%pedestrianTraverseName = '../bolz_hall_3.txt';
%pedestrianTraverseName = '../caldwell_lab.txt';

dataTaha = load(pedestrianTraverseName);
accData = dataTaha(:,1:3);
gyroData = dataTaha(:,4:6);

% accRange = 255; % [0-255] is the range of accelerometer ADXL345 placed on Razor IMU
worldGravity = 9.8; % 1g = 9.81m/s^2
GRAVITY = 256; % defined at Razor_AHRS.ino
accData = (accData ./ GRAVITY) * worldGravity; % first turn into g, then into m/s2

nOfCapturedSamples = length(dataTaha);
G_Dt = dataTaha(:,7);
packetCount = dataTaha(:,8);

nMissedSamples = number_of_missed_samples(packetCount);
if ( nMissedSamples > 0 )
% recover missed samples (due to misses in XBee) by using linear interpolation
    [gyroX, timestamp] = construct_missed_samples(gyroData(:,1), G_Dt, packetCount);
    [gyroY, timestamp] = construct_missed_samples(gyroData(:,2), G_Dt, packetCount);
    [gyroZ, timestamp] = construct_missed_samples(gyroData(:,3), G_Dt, packetCount);
    [accX, timestamp] = construct_missed_samples(accData(:,1), G_Dt, packetCount);
    [accY, timestamp] = construct_missed_samples(accData(:,2), G_Dt, packetCount);
    [accZ, timestamp] = construct_missed_samples(accData(:,3), G_Dt, packetCount);
else
    fprintf('There is no missed sample in pedestrian traverse data.\n');
    gyroX = gyroData(:,1);
    gyroY = gyroData(:,2);
    gyroZ = gyroData(:,3);
    accX = accData(:,1);
    accY = accData(:,2);
    accZ = accData(:,3);
    timestamp = construct_time_axis(G_Dt);
end
clear gyroData accData G_Dt packetCount

timestamp = timestamp'; % making it compatible w/ Fischer's code
data_size = length(gyroX);
acc_s = [accX accY accZ]'; % Accelerations in sensor frame.
gyro_s = [gyroX gyroY gyroZ]'; % Rates of turn in sensor frame.
%%
clear accX accY accZ gyroX gyroY gyroZ dataTaha
g = 9.8; % Gravity.

%% Initialise parameters.
% Orientation from accelerometers. Sensor is assumed to be stationary.
pitch = -asin(acc_s(1,1)/g);
roll = atan(acc_s(2,1)/acc_s(3,1));
yaw = 0;

C = [cos(pitch)*cos(yaw) (sin(roll)*sin(pitch)*cos(yaw))-(cos(roll)*sin(yaw)) (cos(roll)*sin(pitch)*cos(yaw))+(sin(roll)*sin(yaw));
    cos(pitch)*sin(yaw)  (sin(roll)*sin(pitch)*sin(yaw))+(cos(roll)*cos(yaw))  (cos(roll)*sin(pitch)*sin(yaw))-(sin(roll)*cos(yaw));
    -sin(pitch) sin(roll)*cos(pitch) cos(roll)*cos(pitch)];
C_prev = C;

% Preallocate storage for heading estimate. Different from direction of
% travel, the heading indicates the direction that the sensor, and therefore
% the pedestrian, is facing.
heading = nan(1, data_size);
heading(1) = yaw;

% Gyroscope bias, to be determined for each sensor.
%  -- Defined above so we don't forget to change for each dataset. --

% Preallocate storage for accelerations in navigation frame.
acc_n = nan(3, data_size);
acc_n(:,1) = C*acc_s(:,1);

% Preallocate storage for velocity (in navigation frame).
% Initial velocity assumed to be zero.
vel_n = nan(3, data_size);
vel_n(:,1) = [0 0 0]';

% Preallocate storage for position (in navigation frame).
% Initial position arbitrarily set to the origin.
pos_n = nan(3, data_size);
pos_n(:,1) = [0 0 0]';

% Preallocate storage for distance travelled used for altitude plots.
distance = nan(1,data_size-1);
distance(1) = 0;


% Error covariance matrix.
P = zeros(9);

% Process noise parameter, gyroscope and accelerometer noise.
sigma_omega = 1e-2; sigma_a = 1e-2;

% ZUPT measurement matrix.
H = [zeros(3) zeros(3) eye(3)];

% ZUPT measurement noise covariance matrix.
sigma_v = 1e-2;
R = diag([sigma_v sigma_v sigma_v]).^2;

% Gyroscope stance phase detection threshold.
gyro_threshold = 0.6; % 0.6 in the paper

z = 0; % index for ZUPT samples
consequtive = 0; % used to determine ZUPT chains and stride sample
consequtiveZUPTsamples = 0; % a parameter to determine stride sample, if you set it to high values you might miss the stride
if (choice == 1)
    consequtiveZUPTsamples = 8;
else if (choice == 2)
        consequtiveZUPTsamples = 9;
    end
end
strideNumber = 0;

%% Main Loop
for t = 2:data_size
    %%% Start INS (transformation, double integration) %%%
    dt = timestamp(t) - timestamp(t-1);
    
    % Remove bias from gyro measurements.
    gyro_s1 = gyro_s(:,t) - gyro_bias;
    
    % Skew-symmetric matrix for angular rates, aka angular velocity tensor
    ang_rate_matrix = [0   -gyro_s1(3)   gyro_s1(2);
                    gyro_s1(3)  0   -gyro_s1(1);
                    -gyro_s1(2)  gyro_s1(1)  0];
    
    % orientation estimation
    C = C_prev*(2*eye(3)+(ang_rate_matrix*dt))/(2*eye(3)-(ang_rate_matrix*dt));
    
    % Transforming the acceleration from sensor frame to navigation frame.
    acc_n(:,t) = 0.5*(C + C_prev)*acc_s(:,t);
    
    % Velocity and position estimation using trapeze integration.
    vel_n(:,t) = vel_n(:,t-1) + ((acc_n(:,t) - [0; 0; g] )+(acc_n(:,t-1) - [0; 0; g]))*dt/2;
    pos_n(:,t) = pos_n(:,t-1) + (vel_n(:,t) + vel_n(:,t-1))*dt/2;
    
    % Skew-symmetric cross-product operator matrix formed from the n-frame accelerations.
    S = [0  -acc_n(3,t)  acc_n(2,t);
        acc_n(3,t)  0  -acc_n(1,t);
        -acc_n(2,t) acc_n(1,t) 0];
    
    % State transition matrix.
    F = [eye(3)  zeros(3,3)    zeros(3,3);
        zeros(3,3)   eye(3)  dt*eye(3);
        -dt*S  zeros(3,3)    eye(3) ];
    
    % Compute the process noise covariance Q.
    Q = diag([sigma_omega sigma_omega sigma_omega 0 0 0 sigma_a sigma_a sigma_a]*dt).^2;
    
    % Propagate the error covariance matrix.
    P = F*P*F' + Q;
    %%% End INS %%%
    
    % Stance phase detection and zero-velocity updates.
    if ( norm(gyro_s(:,t)) < gyro_threshold )
        z = z+1;
        ZUPTindexes(z) = t;
        if (z > 1)
            if ( (ZUPTindexes(z) - ZUPTindexes(z-1)) < 5 ) % (ZUPTindexes(z) - ZUPTindexes(z-1)) == 1
                consequtive = consequtive + 1;
            else
                consequtive = 0;
            end
            if (consequtive == consequtiveZUPTsamples)
                strideNumber = strideNumber + 1;
                strideIndexes(strideNumber) = t - consequtiveZUPTsamples;
            end
        end
        %%% Start Kalman filter zero-velocity update %%%
        % Kalman gain.
        K = (P*(H)')/((H)*P*(H)' + R);
        
        % Update the filter state (compute state errors)
        delta_x = K*vel_n(:,t);
        
        % Update the error covariance matrix.
        %P = (eye(9) - K*(H)) * P * (eye(9) - K*(H))' + K*R*K'; % Joseph form to guarantee symmetry and positive-definiteness.
        P = (eye(9) - K*H)*P; % Simplified covariance update found in most books.
        
        % Extract errors from the KF state.
        attitude_error = delta_x(1:3);
        pos_error = delta_x(4:6);
        vel_error = delta_x(7:9);
        %%% End Kalman filter zero-velocity update %%%
        
        %%% Apply corrections to INS estimates. %%%
        % Skew-symmetric matrix for small angles to correct orientation.
        ang_matrix = -[0   -attitude_error(3,1)   attitude_error(2,1);
            attitude_error(3,1)  0   -attitude_error(1,1);
            -attitude_error(2,1)  attitude_error(1,1)  0];
        
        % Correct orientation.
        C = (2*eye(3)+(ang_matrix))/(2*eye(3)-(ang_matrix))*C;
        
        % Correct position and velocity based on Kalman error estimates.
        vel_n(:,t)=vel_n(:,t)-vel_error;
        pos_n(:,t)=pos_n(:,t)-pos_error;
    end
    heading(t) = atan2(C(2,1), C(1,1)); % Estimate and save the yaw of the sensor (different from the direction of travel). Unused here but potentially useful for orienting a GUI correctly.
    C_prev = C; % Save orientation estimate, required at start of main loop.
    
    % Compute horizontal distance.
    distance(1,t) = distance(1,t-1) + sqrt((pos_n(1,t)-pos_n(1,t-1))^2 + (pos_n(2,t)-pos_n(2,t-1))^2);
end

%choice = input('Enter 1 for Bolz Hall, 2 for Caldwell: ');
%% Rotate position estimates and plot.
figure(1); set(figure(1),'position',[6 212 560 420]);
box on;
hold on;
angle = 0;
sf = 1; % scale factor
if (choice == 1)
    sf = 1.06; % 1.09 for bolz 1.11 it was 1.15 for _2 1.16 for _4 1.06 for _5 1.07 for _8
    angle = 189; % 187 or 182. 189 for 8.
    if (choice2 == 6) % room traversal
        sf = 1.06; % or 1.06
        angle = 184;
    else if (choice2 == 7)
        sf = 1.06; angle = 187;
        else if (choice2 == 8)
                sf = 1.06; angle = 190;
            else if (choice2 == 9) % room traversal
                    sf = 1.05; angle = 189;
                else if (choice2 == 10 )
                        sf = 1.07; angle = 189;
                        else if (choice2 == 11)
                                sf = 1.05; angle = 184;
                            else if (choice2 == 12)
                                    sf = 1.06; angle = 182;
                                else if (choice2 == 13)
                                        sf = 1.06; angle = 187;
                                    else if (choice2 == 14)
                                            sf = 1.04; angle = 182;
                                        else if (choice2 == 15)
                                                sf = 1.05; angle = 190;
                                            else if (choice2 == 16)
                                                    sf = 1.06; angle = 182;
                                                else if (choice2 == 17)
                                                        sf = 1.05; angle = 194;
                                                    else if (choice2 == 19)
                                                            sf = 1.07; angle = 183;
                                                        else if (choice2 == 20)
                                                                sf = 1.08; angle = 185;
                                                            end
                                                        end
                                                    end
                                                end
                                            end
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
    end
else if (choice == 2)
    sf = 1.1; % scale factor
    angle = 190; % 181   1892     190 for multiple traverse
    else if (choice == 3)
        sf = 1.11; % scale factor1 1.09 for bolz 1.11 it was 1.18 for _2
        angle = 187; % Rotation angle required to achieve an aesthetic alignment of the figure.
        end
    end
end
% 183 degree for bolz hall
rotation_matrix = [cosd(angle) -sind(angle);
    sind(angle) cosd(angle)];
pos_r = zeros(2,data_size);
for idx = 1:data_size
    pos_r(:,idx) = rotation_matrix*[pos_n(1,idx) pos_n(2,idx)]';
end

pos_r = sf*pos_r;
if (choice == 1)
    pos_r(1,:) = -pos_r(1,:);
else if (choice == 2)
        pos_r(2,:) = -pos_r(2,:);
    else if (choice == 3)
            pos_r(2,:) = -pos_r(2,:);
        end
    end
end

distance = total_distance_calculation(pos_r(1,:), pos_r(2,:));
fprintf('Total traveled distance is %.3f meters.\n', sum(distance));

figure(1);
plot(pos_r(1,:),pos_r(2,:),'r','LineWidth',2);
hold on;
plot(pos_r(1,strideIndexes), pos_r(2,strideIndexes), 'bx','linewidth',2);
start = plot(pos_r(1,1),pos_r(2,1),'Marker','^','LineWidth',2,'LineStyle','none');
stop = plot(pos_r(1,end),pos_r(2,end),'Marker','o','LineWidth',2,'LineStyle','none');
xlabel('x (m)'); ylabel('y (m)');
title('Estimated 2D path'); legend([start;stop],'Start','End');
axis equal; grid; hold off;

%run plot_sensor_data.m

%% illustrating floor plan plots after this point
nOfBuildings = 3; % bolz and caldwell
buildingMaps = cell(1,nOfBuildings);
buildingMaps{1} = 'bolz_hall_2nd_floor_rotated.jpg';
buildingMaps{2} = 'caldwell_2nd_floor_rotated.png';
buildingMaps{3} = 'bolz_hall_2nd_floor_rotated.jpg';
if (choice == 1 || choice == 2)
    img = rgb2gray(imread(buildingMaps{choice}));
else if (choice == 3)
        img = rgb2gray(imread(buildingMaps{choice}));
        img = img(1:780,1:465);
    end
end
height = size(img,1); width = size(img,2);
startPixelCoordinates = cell(1,nOfBuildings); % start pixel coordinates of the pedestrian
startPixelCoordinates{1} = [1364 + 1; 735 + 1];
startPixelCoordinates{2} = [387 + 1; 1081 + 1];
startPixelCoordinates{3} = [277 + 1; 525 + 1];
meter2pixelConstant = cell(1,nOfBuildings);
meter2pixelConstant{1} = 200 / 9.7536; % unit conversion constant for bolz hall traverse
meter2pixelConstant{2} = 400 / 18.288;
meter2pixelConstant{3} = 200 / 9.7536; % unit conversion constant for bolz hall traverse

temp1 = repmat(startPixelCoordinates{choice},1,data_size);
temp2 = meter2pixelConstant{choice}.*pos_r;
temp3 = meter2pixelConstant{choice}.*pos_r;
trajectoryPixels = [temp1(1,:) + temp2(1,:); temp1(2,:) - temp3(2,:)];
trajectoryStridesPixels = trajectoryPixels(:,strideIndexes);
figure(2); set(figure(2),'position',[481 14 1114 799]);
set(gcf,'position',[564 61 927 670]);
imshow(img);
hold on;
% plot raw trajectory on the indoor map
% all samples
plot(trajectoryPixels(1,:),trajectoryPixels(2,:), 'r.','linewidth',2);
% ZUPT samples with green x
plot(trajectoryPixels(1,ZUPTindexes),trajectoryPixels(2,ZUPTindexes), 'gx','linewidth',2);
% strides with blue circles
plot(trajectoryStridesPixels(1,:),trajectoryStridesPixels(2,:), 'bo','linewidth',2);
hold off;

figure(2); set(figure(2),'position',[481 14 1114 799]);
set(gcf,'position',[564 61 927 670]);
imshow(img);
hold on;
% plot raw trajectory on the indoor map
% all samples
plot(trajectoryPixels(1,:),trajectoryPixels(2,:), 'r.','linewidth',2);
% ZUPT samples with green x
plot(trajectoryPixels(1,ZUPTindexes),trajectoryPixels(2,ZUPTindexes), 'gx','linewidth',2);
% strides with blue circles
plot(trajectoryStridesPixels(1,:),trajectoryStridesPixels(2,:), 'bo','linewidth',2);
hold off;

% figure(13); set(figure(13),'position',[481 14 1114 799]);
% set(gcf,'position',[564 61 927 670]);
% imshow(img);

%% import static floor plan pdfs from previous studies
load('static_floor_pdfs.mat');
bolzPDF = bolzPDF + 1; caldwellPDF = caldwellPDF + 1;

global staticFloorPDF;
staticFloorPDF = cell(1,nOfBuildings);
staticFloorPDF{1} = double(bolzPDF)./255; staticFloorPDF{2} = double(caldwellPDF)./255;
staticFloorPDF{3} = staticFloorPDF{1}(1:780,1:465);
delete bolzPDF caldwellPDF
load('bolz_hall_pdf_updated.mat');
staticFloorPDF{1} = double(bolzPDF)./255;
delete bolzPDF

figure(3); set(gcf,'position',[371 151 535 422]);
imshow(staticFloorPDF{choice},'colormap',jet(255));
colormap jet;
set(gca, 'YDir', 'reverse');
hold on;
plot(trajectoryPixels(1,strideIndexes),trajectoryPixels(2,strideIndexes),'wo-','linewidth',2);
hold off;

figure(4); set(gcf, 'position', [911   152   560   420]);
surf(staticFloorPDF{choice}); shading flat; colormap jet;
set(gca, 'YDir', 'reverse'); axis tight;

%% IMPLEMENTATION of the NON-RECURSIVE BAYESIAN ESTIMATION ALGORITHM
muPrior = zeros(2,strideNumber);
muLH = zeros(2,strideNumber);
muPostTemp = zeros(2,strideNumber);
sigmaPostTemp = zeros(2,strideNumber);
finalEstimate = zeros(2,strideNumber);
% std deviation of prior and measurement distributions are adjustable parameters
measSigmaX = 25; measSigmaY = measSigmaX; % for sensor likelihood
priorSigmaX = 40; priorSigmaY = priorSigmaX; % for prior distribution
nx = 25; px = 25; ny = 25; py = 25;
posterior = zeros(ny+py+1,nx+px+1,strideNumber);
f_staticFloor = zeros(ny+py+1,nx+px+1,strideNumber);
f_postTemp = zeros(ny+py+1,nx+px+1,strideNumber);
f_prior = zeros(ny+py+1,nx+px+1,strideNumber);
f_LH = zeros(ny+py+1,nx+px+1,strideNumber);
mu = trajectoryStridesPixels;
till = 3; % 33 for bolz, 55 for caldwell
for i = 1 : strideNumber % strideNumber
    if (i < strideNumber+1) % 3 for constant velocity MM, 4 for constant acceleration MM
        % nx = 30; px = 30; ny = 30; py = 30;
        x = (mu(1,i) - nx) : 1 : (mu(1,i) + px);
        y = (mu(2,i) - ny) : 1 : (mu(2,i) + py);
        [X,Y] = meshgrid(x,y);
        f_likelihood = exp(-0.5*(((X-mu(1,i))./measSigmaX).^2 ...
            + ((Y-mu(2,i))./measSigmaY).^2));
        % I need to write my own subpixel crop function
        if (i <= 2)
            f_staticFloor(:,:,i) = imcrop_taha_subpixel_accuracy...
                (choice,mu(:,i),nx,px,ny,py);
        else
            fprintf('%i\n',i);
            deltaImuPosition = position_wrt_previous_two_strides...
                (mu(:,i-2),mu(:,i-1),mu(:,i)); % delta sensor likelihood
%             figure(3); hold on;
%             plot(trajectoryPixels(1,strideIndexes(i)),trajectoryPixels(2,strideIndexes(i)),'bx-','linewidth',2);
%             hold off;
            % now we need to slice the f_staticFloor correctly
            imuPosition = sensor_likelihood_wrt_previous_two_estimates...
                (finalEstimate(:,i-2),finalEstimate(:,i-1),deltaImuPosition);
%             figure(3); hold on; plot(imuPosition(1),imuPosition(2),'wx'); hold off;
            f_staticFloor(:,:,i) = imcrop_taha_subpixel_accuracy...
                (choice,imuPosition,nx,px,ny,py);
        end
        % manually switching off static floor map when room visit happens
        manual_switch = false;
        if (choice2 == 6 || choice2 >= 9) % traverse indexes with room traversals
            manual_switch = true;
        else
            manual_switch = false;
        end
        if (manual_switch == true)
            if (choice2 == 6)
                if ((i >= 8 && i <=20) || (i >= 33 && i <=45) || ...
                        (i >= 63 && i <=75) || (i >= 104 && i <=117))
                    f_staticFloor(:,:,i) = 255;
                end
            else if (choice2 == 9)
                if ((i >= 19 && i <=34) || (i >= 54 && i <=67) || ...
                     (i >= 164 && i <=172) || (i >= 194 && i <=210))
                    f_staticFloor(:,:,i) = 255;
                end
            else if (choice2 == 10)
                if ((i >= 17 && i <= 35))
                    f_staticFloor(:,:,i) = 255;
                end
            else if (choice2 == 11 || choice2 == 17)
                if ((i >= 16 && i <= 33))
                    f_staticFloor(:,:,i) = 255;
                end
            else if (choice2 == 12)
                if (i >= 18)
                    f_staticFloor(:,:,i) = 255;
                end
            else if (choice2 == 13)
                if (i >= 18) % added 50 to avoid FP
                    f_staticFloor(:,:,i) = 255;
                end
            else if (choice2 == 14)
                if ((i >= 16 && i <= strideNumber-16+50)) % added 50 to avoid FP
                    f_staticFloor(:,:,i) = 255;
                end
            else if (choice2 == 15)
            if (i >= 17) % added 50 to avoid
                f_staticFloor(:,:,i) = 255;
            end
            else if (choice2 == 16)
            if ((i >= 54 && i <= strideNumber-54))
                f_staticFloor(:,:,i) = 255;
            end
            else if (choice2 == 20)
            if ((i >= 8 && i <= 55) || (i >= 68)) % && i <= 129
                f_staticFloor(:,:,i) = 255;
            end
                end
                end
                end
                end
                end
                end
                end
                end
                end
            end
        end
        % figure(10); clf; subplot(121); imagesc(f_likelihood);subplot(122); imagesc(f_staticFloor(:,:,i));
        posterior = f_likelihood.*f_staticFloor(:,:,i);
        posterior(:,:,i) = posterior/max(max(posterior));
        maxValue = max(max(posterior(:,:,i)));
        [rowsOfMaxes,colsOfMaxes] = find(posterior(:,:,i) == maxValue);
        if (i > 2)
            finalEstimate(:,i) = [mean(colsOfMaxes); mean(rowsOfMaxes)] - ...
                             [nx; ny] + imuPosition - [1;1];
        else
            finalEstimate(:,i) = [mean(colsOfMaxes); mean(rowsOfMaxes)] - ...
                             [nx; ny] + mu(:,i);
        end
%         figure(3); hold on;
%         plot(finalEstimate(1,i),finalEstimate(2,i),'ks-','linewidth',2);
%         hold off;
%         zeta = 1;
%         pause();
    else
        % apply MM first, and then compute tempPost dist with theoretical_multiplication_of_two_gaussians() 
        % fcn. Finally apply static floor plan pdf to find resulting posterior distribution
        v = finalEstimate(:,i-1) - finalEstimate(:,i-2);
%         vPrev = finalEstimate(:,i-2)-finalEstimate(:,i-3);
%         a = v - vPrev;
%         muPrior(:,i) = finalEstimate(:,i-1)+v+0.5*a; % mean of prior dist after applying MM
        muPrior(:,i) = finalEstimate(:,i-1)+sf*v; % mean of prior dist after applying MM
        x = (muPrior(1,i) - nx) : 1 : (muPrior(1,i) + px);
        y = (muPrior(2,i) - ny) : 1 : (muPrior(2,i) + py);
        [X,Y] = meshgrid(x,y);
        f_prior(:,:,i) = exp(-0.5*(((X-muPrior(1,i))./priorSigmaX).^2 + ...
            ((Y-muPrior(2,i))./priorSigmaY).^2));
        % likelihood mean is the change in IMU solution added to previous estimate
        muLH(:,i) = finalEstimate(:,i-1)+(mu(:,i)-mu(:,i-1)); % mean of likelihood
        x = (muLH(1,i) - nx) : 1 : (muLH(1,i) + px);
        y = (muLH(2,i) - ny) : 1 : (muLH(2,i) + py);
        [X,Y] = meshgrid(x,y);
        f_LH(:,:,i) = exp(-0.5*(((X-muLH(1,i))./measSigmaX).^2 + ...
            ((Y-muLH(2,i))./measSigmaY).^2));
        cd('gaussians multiplication exercise');
        [muPostTemp(:,i), sigmaPostTemp(:,i)] = ...
            theoretical_multiplication_of_two_2d_gaussians(muPrior(:,i),...
            [priorSigmaX;priorSigmaY],muLH(:,i),[measSigmaX;measSigmaY]);
        cd('../'); % come back to the correct directory
        x = (muPostTemp(1,i) - nx) : 1 : (muPostTemp(1,i) + px);
        y = (muPostTemp(2,i) - ny) : 1 : (muPostTemp(2,i) + py);
        [X,Y] = meshgrid(x,y);
        f_postTemp(:,:,i) = exp(-0.5*(((X-muPostTemp(1,i))./sigmaPostTemp(1,i)).^2 + ...
            ((Y-muPostTemp(2,i))./sigmaPostTemp(2,i)).^2));
        % i need to write my own subpixel crop function
        f_staticFloor(:,:,i) = imcrop_taha_subpixel_accuracy(choice,muPostTemp(:,i),...
            nx,px,ny,py);
        posterior = f_postTemp(:,:,i).*f_staticFloor(:,:,i);
        posterior(:,:,i) = posterior/max(max(posterior));
        maxValue = max(max(posterior(:,:,i)));
        [rowsOfMaxes,colsOfMaxes] = find(posterior(:,:,i) == maxValue);
        finalEstimate(:,i) = [mean(colsOfMaxes); mean(rowsOfMaxes)] - [nx; ny] + ...
                                                                    muPostTemp(:,i);
%         figure(5);
%         subplot(231); imshow(f_prior(:,:,i)); title('prior');
%         subplot(232); imshow(f_LH(:,:,i)); title('likelihood');
%         subplot(234); imshow(f_postTemp(:,:,i)); title('postTemp');
%         subplot(235); imshow(f_staticFloor(:,:,i)); title('static floor plan pdf');
%         subplot(236); imshow(posterior(:,:,i)); title('posterior');
        
    end
%     figure(10); set(gcf,'position',[371 151 535 422]);
%     imshow(staticFloorPDF{choice},'colormap',jet(255));
%     colormap jet; set(gca, 'YDir', 'reverse'); hold on;
%     plot(mu(1,1:i),mu(2,1:i), 'wo-','linewidth',2); % this is IMU (EKF+DR+ZUPT) solution
%     plot(finalEstimate(1,1:i),finalEstimate(2,1:i), 'kx-','linewidth',2);
%     hold off;
%     close(figure(10));
end

%%
figure(5); clf; set(figure(5),'position',[1 59 1114 671]);
set(gcf,'position',[564 61 927 670]);
imshow(img); hold on;
% 1upsamplingRate = 1; muUS = zeros(size(mu,1),upsamplingRate*size(mu,2));
% muUS(1,:) = interp(mu(1,:),upsamplingRate);
% muUS(2,:) = interp(mu(2,:),upsamplingRate);
if (choice == 1 || choice == 2)
plot(mu(1,1:end),mu(2,1:end),'b-x','linewidth',1.5); % this is IMU (EKF+DR+ZUPT) solution
%plot(muUS(1,1:end-18*upsamplingRate),muUS(2,1:end-18*upsamplingRate),'b-.','linewidth',3); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,1:end),finalEstimate(2,1:end),'r-x','linewidth',1.5);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',2,'markerfacecolor','r','markeredgecolor','k','markersize',10);
%plot(finalEstimate(1,68),finalEstimate(2,68),'gs','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',10);
%plot(finalEstimate(1,strideNumber-230),finalEstimate(2,strideNumber-230),'gs','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',10);
% plot(mu(1,56),mu(2,56),'bs','linewidth',2,'markerfacecolor','b','markeredgecolor','b','markersize',10);
else (choice == 3)
plot(mu(1,1:end-30),mu(2,1:end-30), 'b-.','linewidth',2); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,:),finalEstimate(2,:), 'r-','linewidth',2);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',2,'markerfacecolor','r','markeredgecolor','k','markersize',10);
plot(finalEstimate(1,end-30),finalEstimate(2,end-30), 'gs','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',10);
end
% plot(finalEstimate(1,19),finalEstimate(2,19),'ko','linewidth',2,'markeredgecolor','k','markersize',16);
% plot(finalEstimate(1,strideNumber-15),finalEstimate(2,strideNumber-15),'ko','linewidth',2,'markeredgecolor','k','markersize',16);
hold off;
legend('complementary KF','non-recursive Bayesian map-matching based MHT','start','stop');
set(legend,'fontsize',16,'interpreter','latex','location','northeast');
% figure(5);xlim([660 1410]);ylim([548 1018]);
%%
figure(6); set(gcf,'position',[371 151 535 422]);
imshow(staticFloorPDF{choice},'colormap',jet(255));
colormap jet; set(gca, 'YDir', 'reverse'); hold on;
plot(mu(1,:),mu(2,:), 'wx-','linewidth',2); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,:),finalEstimate(2,:), 'kx-','linewidth',2);
%plot(finalEstimate(1,till),finalEstimate(2,till), 'ko-','linewidth',2);
hold off;

% figure(7);
% plot(finalEstimate(1,:),finalEstimate(2,:),'bo-'); grid on;

%%
% activation of floor plan
y = ones(1,strideNumber);
if (choice2 == 6 || choice2 == 9)
    figure(6);
    hold on;
    if (choice2 == 6)
        for i=1:strideNumber
            if ((i >= 8 && i <=20) || (i >= 33 && i <=45) || ...
                            (i >= 63 && i <=75) || (i >= 104 && i <=117))
                y(i)=0;
            end
        end
    end
    if (choice2 == 9)
        for i=1:strideNumber
            if ((i >= 19 && i <=34) || (i >= 54 && i <=67) || ...
                     (i >= 164 && i <=172) || (i >= 194 && i <=210))
                y(i)=0;
            end
        end
    end
    plot(1:strideNumber,y,'k-','linewidth',1.5);
    grid on;
    hold off;
    xlabel('stride number');
    %ylabel('Map-matching activation');
    yticks([0 1]);
    yticklabels({'map OFF','map ON'})
    axis([0 strideNumber -0.1 1.1])
end
%% illustration of hypotheses
figure(7); set(figure(7),'position',[1 59 1114 671]);
set(gcf,'position',[564 61 927 670]);
imshow(img); hold on;
% 1upsamplingRate = 1; muUS = zeros(size(mu,1),upsamplingRate*size(mu,2));
% muUS(1,:) = interp(mu(1,:),upsamplingRate);
% muUS(2,:) = interp(mu(2,:),upsamplingRate);
if (choice == 1 || choice == 2)
plot(mu(1,1:end),mu(2,1:end),'b-x','linewidth',1.5); % this is IMU (EKF+DR+ZUPT) solution
%plot(muUS(1,1:end-18*upsamplingRate),muUS(2,1:end-18*upsamplingRate),'b-.','linewidth',3); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,1:end),finalEstimate(2,1:end),'r-x','linewidth',1.5);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',2,'markerfacecolor','r','markeredgecolor','k','markersize',16);
plot(finalEstimate(1,end),finalEstimate(2,end),'gs','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',16);
plot(finalEstimate(1,strideNumber-235),finalEstimate(2,strideNumber-235),'kp','linewidth',2,'markerfacecolor','g','markersize',16);
plot(finalEstimate(1,strideNumber-234),finalEstimate(2,strideNumber-234),'kv','linewidth',2,'markerfacecolor','y','markersize',16);
plot(finalEstimate(1,strideNumber-233),finalEstimate(2,strideNumber-233),'ko','linewidth',2,'markerfacecolor',[1 0.5 0],'markersize',16);
plot(finalEstimate(1,strideNumber-232),finalEstimate(2,strideNumber-232),'ks','linewidth',2,'markerfacecolor',[1 0 0.5],'markersize',16);
plot(finalEstimate(1,strideNumber-231),finalEstimate(2,strideNumber-231),'kh','linewidth',2,'markerfacecolor',[0 0.5 1],'markersize',16);
plot(finalEstimate(1,strideNumber-230),finalEstimate(2,strideNumber-230),'kd','linewidth',2,'markerfacecolor',[0.5 0 0.5],'markersize',16);
plot(finalEstimate(1,strideNumber-229),finalEstimate(2,strideNumber-229),'k^','linewidth',2,'markerfacecolor',[0 0.5 0],'markersize',16);
% plot(finalEstimate(1,18),finalEstimate(2,18),'k<','linewidth',2,'markerfacecolor','k','markersize',16);
else (choice == 3)
plot(mu(1,1:end-30),mu(2,1:end-30), 'b-.','linewidth',2); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,:),finalEstimate(2,:), 'r-','linewidth',2);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',2,'markerfacecolor','r','markeredgecolor','k','markersize',16);
plot(finalEstimate(1,end-30),finalEstimate(2,end-30), 'gs','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',16);
end
hold off;
legend('complementary KF','non-recursive Bayesian map-matching','pedestrian start',...
    'pedestrian stop','hypothesis k-3','hypothesis k-2','hypothesis k-1','hypothesis k',...
    'hypothesis k+1','hypothesis k+2','hypothesis k+3');
set(legend,'fontsize',16,'interpreter','latex','location','northwest');
% figure(7);xlim([660 1410]);ylim([548 1018]);
%% hypothesis k-3, k-2, k-1, k, k+1, k+2, k+3
% the angles are calculated according to IMU location & actual door location
hkmin3angle = 65; % rotation for hypothesis k-3, computed wrt k-3 and k-4
hkmin2angle = 18; % rotation for hypothesis k-2.
hkmin1angle = -17; % rotation for hypothesis k-1.
hkangle = -40; % rotation for hypothesis k.
hkplus1angle = -46; % rotation for hypothesis k+1.
% burda kaldik kplus2
hkplus2angle = -30; % rotation for hypothesis k+2.
hkplus3angle = 34; % rotation for hypothesis k+3.
% for each hypothesis, the rotation matrix is wrt to actual door location
rotation_matrixkmin3angle = [cosd(hkmin3angle) -sind(hkmin3angle);
    sind(hkmin3angle) cosd(hkmin3angle)];
rotation_matrixkmin2angle = [cosd(hkmin2angle) -sind(hkmin2angle);
    sind(hkmin2angle) cosd(hkmin2angle)];
rotation_matrixkmin1angle = [cosd(hkmin1angle) -sind(hkmin1angle);
    sind(hkmin1angle) cosd(hkmin1angle)];
rotation_matrixkangle = [cosd(hkangle) -sind(hkangle);
    sind(hkangle) cosd(hkangle)];
rotation_matrixkplus1angle = [cosd(hkplus1angle) -sind(hkplus1angle);
    sind(hkplus1angle) cosd(hkplus1angle)];
rotation_matrixkplus2angle = [cosd(hkplus2angle) -sind(hkplus2angle);
    sind(hkplus2angle) cosd(hkplus2angle)];
rotation_matrixkplus3angle = [cosd(hkplus3angle) -sind(hkplus3angle);
    sind(hkplus3angle) cosd(hkplus3angle)];

finalEstimateRotatedhkmin3 = zeros(2,strideNumber);
finalEstimateRotatedhkmin2 = zeros(2,strideNumber);
finalEstimateRotatedhkmin1 = zeros(2,strideNumber);
finalEstimateRotatedhk = zeros(2,strideNumber);
finalEstimateRotatedhkplus1 = zeros(2,strideNumber);
finalEstimateRotatedhkplus2 = zeros(2,strideNumber);
finalEstimateRotatedhkplus3 = zeros(2,strideNumber);

finalEstimateTemp = zeros(2,strideNumber);

for i = 1:strideNumber
    finalEstimateTemp(:,i) = finalEstimate(:,i) - finalEstimate(:,1);
end
for idx = 1:strideNumber
    finalEstimateRotatedhkmin3(:,idx) = rotation_matrixkmin3angle*...
        [finalEstimateTemp(1,idx) finalEstimateTemp(2,idx)]';
    finalEstimateRotatedhkmin2(:,idx) = rotation_matrixkmin2angle*...
        [finalEstimateTemp(1,idx) finalEstimateTemp(2,idx)]';
    finalEstimateRotatedhkmin1(:,idx) = rotation_matrixkmin1angle*...
        [finalEstimateTemp(1,idx) finalEstimateTemp(2,idx)]';
    finalEstimateRotatedhk(:,idx) = rotation_matrixkangle*...
        [finalEstimateTemp(1,idx) finalEstimateTemp(2,idx)]';
    finalEstimateRotatedhkplus1(:,idx) = rotation_matrixkplus1angle*...
        [finalEstimateTemp(1,idx) finalEstimateTemp(2,idx)]';
    finalEstimateRotatedhkplus2(:,idx) = rotation_matrixkplus2angle*...
        [finalEstimateTemp(1,idx) finalEstimateTemp(2,idx)]';
    finalEstimateRotatedhkplus3(:,idx) = rotation_matrixkplus3angle*...
        [finalEstimateTemp(1,idx) finalEstimateTemp(2,idx)]';
end
for i=1:strideNumber
    finalEstimateRotatedhkmin3(:,i) = finalEstimateRotatedhkmin3(:,i) + finalEstimate(:,1);
    finalEstimateRotatedhkmin2(:,i) = finalEstimateRotatedhkmin2(:,i) + finalEstimate(:,1);
    finalEstimateRotatedhkmin1(:,i) = finalEstimateRotatedhkmin1(:,i) + finalEstimate(:,1);
    finalEstimateRotatedhk(:,i) = finalEstimateRotatedhk(:,i) + finalEstimate(:,1);
    finalEstimateRotatedhkplus1(:,i) = finalEstimateRotatedhkplus1(:,i) + finalEstimate(:,1);
    finalEstimateRotatedhkplus2(:,i) = finalEstimateRotatedhkplus2(:,i) + finalEstimate(:,1);
    finalEstimateRotatedhkplus3(:,i) = finalEstimateRotatedhkplus3(:,i) + finalEstimate(:,1);
end

%door216Location = [760.4018;999.5193];
hkmin3Translation = [-140;258]; % [-137;273]
finalEstimateRotatedTranslatedhkmin3 = finalEstimateRotatedhkmin3 + hkmin3Translation; % translation for hypothesis k-3
hkmin2Translation = finalEstimateRotatedhkmin2(:,strideNumber-234)-finalEstimateRotatedTranslatedhkmin3(:,strideNumber-235);%[-30; 140];
finalEstimateRotatedTranslatedhkmin2 = finalEstimateRotatedhkmin2 - hkmin2Translation; % translation for hypothesis k-2
hkmin1Translation = finalEstimateRotatedhkmin1(:,strideNumber-233)-finalEstimateRotatedTranslatedhkmin3(:,strideNumber-235);%[-30; 140];
finalEstimateRotatedTranslatedhkmin1 = finalEstimateRotatedhkmin1 - hkmin1Translation; % translation for hypothesis k-2
hkTranslation = finalEstimateRotatedhk(:,strideNumber-232)-finalEstimateRotatedTranslatedhkmin3(:,strideNumber-235);%[-30; 140];
finalEstimateRotatedTranslatedhk = finalEstimateRotatedhk - hkTranslation; % translation for hypothesis k-2
hkplus1Translation = finalEstimateRotatedhkplus1(:,strideNumber-231)-finalEstimateRotatedTranslatedhkmin3(:,strideNumber-235);%[-30; 140];
finalEstimateRotatedTranslatedhkplus1 = finalEstimateRotatedhkplus1 - hkplus1Translation; % translation for hypothesis k-2
hkplus2Translation = finalEstimateRotatedhkplus2(:,strideNumber-230)-finalEstimateRotatedTranslatedhkmin3(:,strideNumber-235);%[-30; 140];
finalEstimateRotatedTranslatedhkplus2 = finalEstimateRotatedhkplus2 - hkplus2Translation; % translation for hypothesis k-2
hkplus3Translation = finalEstimateRotatedhkplus3(:,strideNumber-229)-finalEstimateRotatedTranslatedhkmin3(:,strideNumber-235);%[-30; 140];
finalEstimateRotatedTranslatedhkplus3 = finalEstimateRotatedhkplus3 - hkplus3Translation; % translation for hypothesis k-2
probStrides = 10;
figure(8); set(figure(8),'position',[1 59 1114 671]);
set(gcf,'position',[564 61 927 670]);
imshow(img); hold on;
% 1upsamplingRate = 1; muUS = zeros(size(mu,1),upsamplingRate*size(mu,2));
% muUS(1,:) = interp(mu(1,:),upsamplingRate);
% muUS(2,:) = interp(mu(2,:),upsamplingRate);
showRotated = 0;
if (choice == 1 || choice == 2)
    if (showRotated)
        plot(finalEstimateRotatedhkplus3(1,1:end),finalEstimateRotatedhkplus3(2,1:end),'r-x','linewidth',1.5);
        plot(finalEstimateRotatedhkplus3(1,strideNumber-229),finalEstimateRotatedhkplus3(2,strideNumber-229),'ks','linewidth',2,'markerfacecolor','y','markersize',16);
        plot(finalEstimateRotatedhkplus3(1,strideNumber-230),finalEstimateRotatedhkplus3(2,strideNumber-230),'ko','linewidth',2,'markerfacecolor','y','markersize',16);
        % we can take stride k-2 to calculate the heading of the current hypothesis
        %plot(finalEstimateRotatedhkplus3(1,strideNumber-16),finalEstimateRotatedhkplus3(2,strideNumber-16),'ko','linewidth',2,'markerfacecolor','y','markersize',16);
    end
else (choice == 3)
%plot(finalEstimateRotated(1,1:end),finalEstimateRotated(2,1:end),'r-x','linewidth',1.5);
end
plot(finalEstimateRotatedTranslatedhkmin3(1,strideNumber-235:strideNumber-235+probStrides),...
    finalEstimateRotatedTranslatedhkmin3(2,strideNumber-235:strideNumber-235+probStrides),'g-x','linewidth',1.5);
plot(finalEstimateRotatedTranslatedhkmin2(1,strideNumber-234:strideNumber-234+probStrides),...
    finalEstimateRotatedTranslatedhkmin2(2,strideNumber-234:strideNumber-234+probStrides),'y-x','linewidth',1.5);
plot(finalEstimateRotatedTranslatedhkmin1(1,strideNumber-233:strideNumber-233+probStrides),...
    finalEstimateRotatedTranslatedhkmin1(2,strideNumber-233:strideNumber-233+probStrides),'y-x','linewidth',1.5,'color',[1 0.5 0]);
plot(finalEstimateRotatedTranslatedhk(1,strideNumber-232:strideNumber-232+probStrides),...
    finalEstimateRotatedTranslatedhk(2,strideNumber-232:strideNumber-232+probStrides),'y-x','linewidth',1.5,'color',[1 0 0.5]);
plot(finalEstimateRotatedTranslatedhkplus1(1,strideNumber-231:strideNumber-231+probStrides),...
    finalEstimateRotatedTranslatedhkplus1(2,strideNumber-231:strideNumber-231+probStrides),'y-x','linewidth',1.5,'color',[0 0.5 1]);
plot(finalEstimateRotatedTranslatedhkplus2(1,strideNumber-230:strideNumber-230+probStrides),...
    finalEstimateRotatedTranslatedhkplus2(2,strideNumber-230:strideNumber-230+probStrides),'y-x','linewidth',1.5,'color',[0.5 0 0.5]);
plot(finalEstimateRotatedTranslatedhkplus3(1,strideNumber-229:strideNumber-229+probStrides),...
    finalEstimateRotatedTranslatedhkplus3(2,strideNumber-229:strideNumber-229+probStrides),'y-x','linewidth',1.5,'color',[0 0.5 0]);
hold off; legend('hypothesis k-3 trajectory','hypothesis k-2 trajectory',...
    'hypothesis k-1 trajectory','hypothesis k trajectory',...
    'hypothesis k+1 trajectory','hypothesis k+2 trajectory',...
    'hypothesis k+3 trajectory');
set(legend,'fontsize',16,'interpreter','latex','location','northwest');
% figure(8);xlim([660 1410]);ylim([548 1018]);
probStridesVectorhkmin3 = zeros(1,probStrides);
probStridesVectorhkmin2 = zeros(1,probStrides);
probStridesVectorhkmin1 = zeros(1,probStrides);
probStridesVectorhk = zeros(1,probStrides);
probStridesVectorhkplus1 = zeros(1,probStrides);
probStridesVectorhkplus2 = zeros(1,probStrides);
probStridesVectorhkplus3 = zeros(1,probStrides);
for i=1:probStrides
    probStridesVectorhkmin3(i) = imcrop_taha_subpixel_accuracy(choice,finalEstimateRotatedTranslatedhkmin3(:,strideNumber-235+i),0,0,0,0);
    probStridesVectorhkmin2(i) = imcrop_taha_subpixel_accuracy(choice,finalEstimateRotatedTranslatedhkmin2(:,strideNumber-234+i),0,0,0,0);
    probStridesVectorhkmin1(i) = imcrop_taha_subpixel_accuracy(choice,finalEstimateRotatedTranslatedhkmin1(:,strideNumber-233+i),0,0,0,0);
    probStridesVectorhk(i) = imcrop_taha_subpixel_accuracy(choice,finalEstimateRotatedTranslatedhk(:,strideNumber-232+i),0,0,0,0);
    probStridesVectorhkplus1(i) = imcrop_taha_subpixel_accuracy(choice,finalEstimateRotatedTranslatedhkplus1(:,strideNumber-231+i),0,0,0,0);
    probStridesVectorhkplus2(i) = imcrop_taha_subpixel_accuracy(choice,finalEstimateRotatedTranslatedhkplus2(:,strideNumber-230+i),0,0,0,0);
    probStridesVectorhkplus3(i) = imcrop_taha_subpixel_accuracy(choice,finalEstimateRotatedTranslatedhkplus3(:,strideNumber-229+i),0,0,0,0);
end

fprintf('Probability of sequential strides for hypothesis k-3\n');
for i=1:probStrides
    if (i==probStrides)
        fprintf('%.3f\n', probStridesVectorhkmin3(i));
    else
        fprintf('%.3f - ', probStridesVectorhkmin3(i));
    end
end
fprintf('Sum of probabilities for hypothesis k-3 is %.3f\n', sum(probStridesVectorhkmin3));

fprintf('Probability of sequential strides for hypothesis k-2\n');
for i=1:probStrides
    if (i==probStrides)
        fprintf('%.3f\n', probStridesVectorhkmin2(i));
    else
        fprintf('%.3f - ', probStridesVectorhkmin2(i));
    end
end
fprintf('Sum of probabilities for hypothesis k-2 is %.3f\n', sum(probStridesVectorhkmin2));

fprintf('Probability of sequential strides for hypothesis k-1\n');
for i=1:probStrides
    if (i==probStrides)
        fprintf('%.3f\n', probStridesVectorhkmin1(i));
    else
        fprintf('%.3f - ', probStridesVectorhkmin1(i));
    end
end
fprintf('Sum of probabilities for hypothesis k-1 is %.3f\n', sum(probStridesVectorhkmin1));

fprintf('Probability of sequential strides for hypothesis k\n');
for i=1:probStrides
    if (i==probStrides)
        fprintf('%.3f\n', probStridesVectorhk(i));
    else
        fprintf('%.3f - ', probStridesVectorhk(i));
    end
end
fprintf('Sum of probabilities for hypothesis k is %.3f\n', sum(probStridesVectorhk));

fprintf('Probability of sequential strides for hypothesis k+1\n');
for i=1:probStrides
    if (i==probStrides)
        fprintf('%.3f\n', probStridesVectorhkplus1(i));
    else
        fprintf('%.3f - ', probStridesVectorhkplus1(i));
    end
end
fprintf('Sum of probabilities for hypothesis k+1 is %.3f\n', sum(probStridesVectorhkplus1));

fprintf('Probability of sequential strides for hypothesis k+2\n');
for i=1:probStrides
    if (i==probStrides)
        fprintf('%.3f\n', probStridesVectorhkplus2(i));
    else
        fprintf('%.3f - ', probStridesVectorhkplus2(i));
    end
end
fprintf('Sum of probabilities for hypothesis k+2 is %.3f\n', sum(probStridesVectorhkplus2));

fprintf('Probability of sequential strides for hypothesis k+3\n');
for i=1:probStrides
    if (i==probStrides)
        fprintf('%.3f\n', probStridesVectorhkplus3(i));
    else
        fprintf('%.3f - ', probStridesVectorhkplus3(i));
    end
end
fprintf('Sum of probabilities for hypothesis k+3 is %.3f\n', sum(probStridesVectorhkplus3));

%%
figure(9); set(gcf,'position',[213 187 1120 420]);
subplot(241);
stem(1:probStrides, probStridesVectorhkmin3,'g.','linewidth',1.7);
grid on; xlabel('stride');ylabel('probability');
title(['$\sum p_i \;=\;$', num2str(sum(probStridesVectorhkmin3)), ' for hypothesis k-3'],'interpreter','latex');
xticks([1:10]); yticks([0:0.2:1]);
xticklabels({'1','2','3','4','5','6','7','8','9','10'});
yticklabels({'','.2','.4','.6','.8','1'});
set(gca,'GridLineStyle','--','position',[0.0342 0.6001 0.2345 0.3216]);
subplot(242);
stem(1:probStrides, probStridesVectorhkmin2,'y.','linewidth',1.7);
grid on; xlabel('stride');
title(['$\sum p_i \;=\;$', num2str(sum(probStridesVectorhkmin2)), ' for hypothesis k-2'],'interpreter','latex');
xticks([1:10]); yticks([0:0.2:1]);
xticklabels({'1','2','3','4','5','6','7','8','9','10'});
yticklabels({'','','','','',''});
set(gca,'GridLineStyle','--','position',[0.0342+0.2345+0.005 0.6001 0.2345 0.3216]);
subplot(243);
stem(1:probStrides, probStridesVectorhkmin1,'y.','linewidth',1.7,'color',[1 0.5 0]);
grid on; xlabel('stride');
title(['$\sum p_i \;=\;$', num2str(sum(probStridesVectorhkmin1)), ' for hypothesis k-1'],'interpreter','latex');
xticks([1:10]); yticks([0:0.2:1]);
xticklabels({'1','2','3','4','5','6','7','8','9','10'});
yticklabels({'','','','','',''});
set(gca,'GridLineStyle','--','position',[0.0342+2*0.2345+2*0.005 0.6001 0.2345 0.3216]);
subplot(244);
stem(1:probStrides, probStridesVectorhk,'y.','linewidth',1.7,'color',[1 0 0.5]);
grid on; xlabel('stride');
title(['$\sum p_i \;=\;$', num2str(sum(probStridesVectorhk)), ' for hypothesis k'],'interpreter','latex');
xticks([1:10]); yticks([0:0.2:1]);
xticklabels({'1','2','3','4','5','6','7','8','9','10'});
yticklabels({'','','','','',''});
set(gca,'GridLineStyle','--','position',[0.0342+3*0.2345+3*0.005 0.6001 0.2345 0.3216]);
subplot(247);
stem(1:probStrides, probStridesVectorhkplus3,'g.','linewidth',1.7,'color',[0 0.5 0]);
grid on; xlabel('stride');%ylabel('probability');
title(['$\sum p_i \;=\;$', num2str(sum(probStridesVectorhkplus3)), ' for hypothesis k+3'],'interpreter','latex');
xticks([1:10]); yticks([0:0.2:1]);
xticklabels({'1','2','3','4','5','6','7','8','9','10'});
yticklabels({'','','','','',''});
set(gca,'GridLineStyle','--','position',[0.1442+2*(0.2345+0.005) 0.1221 0.2345 0.3216]);
subplot(246);
stem(1:probStrides, probStridesVectorhkplus2,'g.','linewidth',1.7,'color',[0.5 0 0.5]);
grid on; xlabel('stride');%ylabel('probability');
title(['$\sum p_i \;=\;$', num2str(sum(probStridesVectorhkplus2)), ' for hypothesis k+2'],'interpreter','latex');
xticks([1:10]); yticks([0:0.2:1]);
xticklabels({'1','2','3','4','5','6','7','8','9','10'});
yticklabels({'','','','','',''});
set(gca,'GridLineStyle','--','position',[0.1442+0.2345+0.005 0.1221 0.2345 0.3216]);
subplot(245);
stem(1:probStrides, probStridesVectorhkplus1,'g.','linewidth',1.7,'color',[0 0.5 1]);
grid on; xlabel('stride');ylabel('probability');
title(['$\sum p_i \;=\;$', num2str(sum(probStridesVectorhkplus1)), ' for hypothesis k+1'],'interpreter','latex');
xticks([1:10]); yticks([0:0.2:1]);
xticklabels({'1','2','3','4','5','6','7','8','9','10'});
yticklabels({'','.2','.4','.6','.8','1'});
set(gca,'GridLineStyle','--','position',[0.1442 0.1221 0.2345 0.3216]);
%% gradual correction for the trajectory in the room
% the hypothesis (stride) with the highest probability has the max rotation and 
% translation. the angle and the shift is gradually distributed to the strides in 
% the room. the very first stride in the room is applied least rotation and shift.
figure(10); set(figure(10),'position',[1 59 1114 671]);
set(gcf,'position',[564 61 927 670]);
imshow(img); hold on;
if (choice == 1 || choice == 2)
plot(mu(1,1:end),mu(2,1:end),'b-x','linewidth',1.5); % this is IMU (EKF+DR+ZUPT) solution
%plot(muUS(1,1:end-18*upsamplingRate),muUS(2,1:end-18*upsamplingRate),'b-.','linewidth',3); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,1:end),finalEstimate(2,1:end),'r-x','linewidth',1.5);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',2,'markerfacecolor','r','markeredgecolor','k','markersize',16);
plot(finalEstimate(1,end),finalEstimate(2,end),'gs','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',16);
else (choice == 3)
plot(mu(1,1:end-30),mu(2,1:end-30), 'b-.','linewidth',2); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,:),finalEstimate(2,:), 'r-','linewidth',2);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',2,'markerfacecolor','r','markeredgecolor','k','markersize',10);
plot(finalEstimate(1,end-30),finalEstimate(2,end-30), 'gs','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',10);
end
% plot(finalEstimateRotatedTranslatedhkplus2(1,strideNumber-15),finalEstimateRotatedTranslatedhkplus2(2,strideNumber-15),'ko','markersize',16);
% plot(finalEstimateRotatedTranslatedhkplus2(1,19),finalEstimateRotatedTranslatedhkplus2(2,19),'ko','markersize',16);
% figure(10);xlim([660 1410]);ylim([548 1018]);

% hkplus2angle = -36; % this is the most voted hypothesis
firstStride = 68; % 18
lastStride = strideNumber-230; % 16
roomStrides = lastStride-firstStride;
translationReference = finalEstimateRotatedTranslatedhkplus2(:,lastStride);
for i = 0:roomStrides
    angleTemp = hkplus2angle*(roomStrides-i)/roomStrides;
    rotation_matrixTemp = [cosd(angleTemp) -sind(angleTemp);
                                sind(angleTemp) cosd(angleTemp)];
    finalEstimateRotatedhkplus2(:,lastStride-i) = rotation_matrixTemp*...
        [finalEstimateTemp(1,lastStride-i) finalEstimateTemp(2,lastStride-i)]';
    translationTemp = hkplus2Translation*(roomStrides-i)/roomStrides;
    finalEstimateRotatedTranslatedhkplus2(:,lastStride-i) = ...
        finalEstimateRotatedhkplus2(:,lastStride-i) - translationTemp; % translation for hypothesis k-2
    if (i == 0)
        translationFix = translationReference - finalEstimateRotatedTranslatedhkplus2(:,lastStride-i);
    end
    plot(finalEstimateRotatedTranslatedhkplus2(1,lastStride-i)+translationFix(1),...
        finalEstimateRotatedTranslatedhkplus2(2,lastStride-i)+translationFix(2),...
        'gx','markersize',12,'linewidth',1.8);
    finalEstimate(:,lastStride-i) = finalEstimateRotatedTranslatedhkplus2(:,lastStride-i)+translationFix;
    fprintf('%i ', lastStride-i);
end
legend('complementary KF','non-recursive estimation','start','stop');
set(legend,'fontsize',16,'interpreter','latex','location','northwest');
hold off;
fprintf('\n');
%% apply non-recursive map-matching to remaining trajectory after leaving the door
for i = lastStride:strideNumber % strideNumber
        x = (mu(1,i) - nx) : 1 : (mu(1,i) + px);
        y = (mu(2,i) - ny) : 1 : (mu(2,i) + py);
        [X,Y] = meshgrid(x,y);
        f_likelihood = exp(-0.5*(((X-mu(1,i))./measSigmaX).^2 ...
            + ((Y-mu(2,i))./measSigmaY).^2));
        fprintf('%i\n',i);
        deltaImuPosition = position_wrt_previous_two_strides...
            (mu(:,i-2),mu(:,i-1),mu(:,i)); % delta sensor likelihood
        % now we need to slice the f_staticFloor correctly
        imuPosition = sensor_likelihood_wrt_previous_two_estimates...
            (finalEstimate(:,i-2),finalEstimate(:,i-1),deltaImuPosition);
        f_staticFloor(:,:,i) = imcrop_taha_subpixel_accuracy...
            (choice,imuPosition,nx,px,ny,py);
        if (choice2 == 20)
                if (i >= 143)
                    f_staticFloor(:,:,i) = 255;
                end

        end
        posterior = f_likelihood.*f_staticFloor(:,:,i);
        posterior(:,:,i) = posterior/max(max(posterior));
        maxValue = max(max(posterior(:,:,i)));
        [rowsOfMaxes,colsOfMaxes] = find(posterior(:,:,i) == maxValue);
        finalEstimate(:,i) = [mean(colsOfMaxes); mean(rowsOfMaxes)] - ...
                             [nx; ny] + imuPosition - [1;1];
end

% the very first stride in the room gets least rotation and shift
figure(11); set(figure(11),'position',[1 59 1114 671]);
set(gcf,'position',[564 61 927 670]);
imshow(img); hold on;
if (choice == 1 || choice == 2)
plot(mu(1,1:end),mu(2,1:end),'b-x','linewidth',1.5); % this is IMU (EKF+DR+ZUPT) solution
%plot(muUS(1,1:end-18*upsamplingRate),muUS(2,1:end-18*upsamplingRate),'b-.','linewidth',3); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,1:end),finalEstimate(2,1:end),'r-x','linewidth',1.5);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',2,'markerfacecolor','r','markeredgecolor','k','markersize',16);
plot(finalEstimate(1,end),finalEstimate(2,end),'gs','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',16);
% plot(finalEstimate(1,143),finalEstimate(2,143),'go','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',16);
else (choice == 3)
plot(mu(1,1:end-30),mu(2,1:end-30), 'b-.','linewidth',2); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,:),finalEstimate(2,:), 'r-','linewidth',2);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',2,'markerfacecolor','r','markeredgecolor','k','markersize',10);
plot(finalEstimate(1,end-30),finalEstimate(2,end-30), 'gs','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',10);
end
% plot(finalEstimateRotatedTranslatedhkplus2(1,strideNumber-15),finalEstimateRotatedTranslatedhkplus2(2,strideNumber-15),'ko','markersize',16);
% plot(finalEstimateRotatedTranslatedhkplus2(1,19),finalEstimateRotatedTranslatedhkplus2(2,19),'ko','markersize',16);
hold off;
legend('complementary KF','MHT with non-recursive map-matching','start','stop');
set(legend,'fontsize',16,'interpreter','latex','location','northwest');
% figure(11);xlim([660 1410]);ylim([548 1018]);
% Room 216 traversal is over. The pedestrian is walking to Room 221 now.
%% illustration of hypotheses
figure(12); set(figure(12),'position',[1 59 1114 671]);
set(gcf,'position',[564 61 927 670]);
imshow(img); hold on;
% 1upsamplingRate = 1; muUS = zeros(size(mu,1),upsamplingRate*size(mu,2));
% muUS(1,:) = interp(mu(1,:),upsamplingRate);
% muUS(2,:) = interp(mu(2,:),upsamplingRate);
if (choice == 1 || choice == 2)
plot(mu(1,1:end),mu(2,1:end),'b-x','linewidth',1.5); % this is IMU (EKF+DR+ZUPT) solution
%plot(muUS(1,1:end-18*upsamplingRate),muUS(2,1:end-18*upsamplingRate),'b-.','linewidth',3); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,1:end),finalEstimate(2,1:end),'r-x','linewidth',1.5);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',2,'markerfacecolor','r','markeredgecolor','k','markersize',16);
plot(finalEstimate(1,end),finalEstimate(2,end),'gs','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',16);
plot(finalEstimate(1,strideNumber-175),finalEstimate(2,strideNumber-175),'kp','linewidth',2,'markerfacecolor','g','markersize',16);
plot(finalEstimate(1,strideNumber-174),finalEstimate(2,strideNumber-174),'kv','linewidth',2,'markerfacecolor','y','markersize',16);
plot(finalEstimate(1,strideNumber-173),finalEstimate(2,strideNumber-173),'ko','linewidth',2,'markerfacecolor',[1 0.5 0],'markersize',16);
plot(finalEstimate(1,strideNumber-172),finalEstimate(2,strideNumber-172),'ks','linewidth',2,'markerfacecolor',[1 0 0.5],'markersize',16);
plot(finalEstimate(1,strideNumber-171),finalEstimate(2,strideNumber-171),'kh','linewidth',2,'markerfacecolor',[0 0.5 1],'markersize',16);
plot(finalEstimate(1,strideNumber-170),finalEstimate(2,strideNumber-170),'kd','linewidth',2,'markerfacecolor',[0.5 0 0.5],'markersize',16);
plot(finalEstimate(1,strideNumber-169),finalEstimate(2,strideNumber-169),'k^','linewidth',2,'markerfacecolor',[0 0.5 0],'markersize',16);
% plot(finalEstimate(1,18),finalEstimate(2,18),'k<','linewidth',2,'markerfacecolor','k','markersize',16);
else (choice == 3)
plot(mu(1,1:end-30),mu(2,1:end-30), 'b-.','linewidth',2); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,:),finalEstimate(2,:), 'r-','linewidth',2);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',2,'markerfacecolor','r','markeredgecolor','k','markersize',16);
plot(finalEstimate(1,end-30),finalEstimate(2,end-30), 'gs','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',16);
end
hold off;
legend('complementary KF','non-recursive Bayesian map-matching','pedestrian start',...
    'pedestrian stop','hypothesis k-3','hypothesis k-2','hypothesis k-1','hypothesis k',...
    'hypothesis k+1','hypothesis k+2','hypothesis k+3');
set(legend,'fontsize',16,'interpreter','latex','location','northwest');
% figure(7);xlim([660 1410]);ylim([548 1018]);
%% hypothesis k-3, k-2, k-1, k, k+1, k+2, k+3
% the angles are calculated according to IMU location & actual door location
hkmin3angle = -130; % rotation for hypothesis k-3, computed wrt k-3 and k-4
hkmin2angle = -125; % rotation for hypothesis k-2.
hkmin1angle = -66; % rotation for hypothesis k-1.
hkangle = -35; % rotation for hypothesis k.
hkplus1angle = -60; % rotation for hypothesis k+1.
% burda kaldik kplus2
hkplus2angle = -110; % rotation for hypothesis k+2.
hkplus3angle = -115; % rotation for hypothesis k+3.
% for each hypothesis, the rotation matrix is wrt to actual door location
rotation_matrixkmin3angle = [cosd(hkmin3angle) -sind(hkmin3angle);
    sind(hkmin3angle) cosd(hkmin3angle)];
rotation_matrixkmin2angle = [cosd(hkmin2angle) -sind(hkmin2angle);
    sind(hkmin2angle) cosd(hkmin2angle)];
rotation_matrixkmin1angle = [cosd(hkmin1angle) -sind(hkmin1angle);
    sind(hkmin1angle) cosd(hkmin1angle)];
rotation_matrixkangle = [cosd(hkangle) -sind(hkangle);
    sind(hkangle) cosd(hkangle)];
rotation_matrixkplus1angle = [cosd(hkplus1angle) -sind(hkplus1angle);
    sind(hkplus1angle) cosd(hkplus1angle)];
rotation_matrixkplus2angle = [cosd(hkplus2angle) -sind(hkplus2angle);
    sind(hkplus2angle) cosd(hkplus2angle)];
rotation_matrixkplus3angle = [cosd(hkplus3angle) -sind(hkplus3angle);
    sind(hkplus3angle) cosd(hkplus3angle)];

finalEstimateRotatedhkmin3 = zeros(2,strideNumber);
finalEstimateRotatedhkmin2 = zeros(2,strideNumber);
finalEstimateRotatedhkmin1 = zeros(2,strideNumber);
finalEstimateRotatedhk = zeros(2,strideNumber);
finalEstimateRotatedhkplus1 = zeros(2,strideNumber);
finalEstimateRotatedhkplus2 = zeros(2,strideNumber);
finalEstimateRotatedhkplus3 = zeros(2,strideNumber);

finalEstimateTemp = zeros(2,strideNumber);

for i = 1:strideNumber
    finalEstimateTemp(:,i) = finalEstimate(:,i) - finalEstimate(:,1);
end
for idx = 1:strideNumber
    finalEstimateRotatedhkmin3(:,idx) = rotation_matrixkmin3angle*...
        [finalEstimateTemp(1,idx) finalEstimateTemp(2,idx)]';
    finalEstimateRotatedhkmin2(:,idx) = rotation_matrixkmin2angle*...
        [finalEstimateTemp(1,idx) finalEstimateTemp(2,idx)]';
    finalEstimateRotatedhkmin1(:,idx) = rotation_matrixkmin1angle*...
        [finalEstimateTemp(1,idx) finalEstimateTemp(2,idx)]';
    finalEstimateRotatedhk(:,idx) = rotation_matrixkangle*...
        [finalEstimateTemp(1,idx) finalEstimateTemp(2,idx)]';
    finalEstimateRotatedhkplus1(:,idx) = rotation_matrixkplus1angle*...
        [finalEstimateTemp(1,idx) finalEstimateTemp(2,idx)]';
    finalEstimateRotatedhkplus2(:,idx) = rotation_matrixkplus2angle*...
        [finalEstimateTemp(1,idx) finalEstimateTemp(2,idx)]';
    finalEstimateRotatedhkplus3(:,idx) = rotation_matrixkplus3angle*...
        [finalEstimateTemp(1,idx) finalEstimateTemp(2,idx)]';
end
for i=1:strideNumber
    finalEstimateRotatedhkmin3(:,i) = finalEstimateRotatedhkmin3(:,i) + finalEstimate(:,1);
    finalEstimateRotatedhkmin2(:,i) = finalEstimateRotatedhkmin2(:,i) + finalEstimate(:,1);
    finalEstimateRotatedhkmin1(:,i) = finalEstimateRotatedhkmin1(:,i) + finalEstimate(:,1);
    finalEstimateRotatedhk(:,i) = finalEstimateRotatedhk(:,i) + finalEstimate(:,1);
    finalEstimateRotatedhkplus1(:,i) = finalEstimateRotatedhkplus1(:,i) + finalEstimate(:,1);
    finalEstimateRotatedhkplus2(:,i) = finalEstimateRotatedhkplus2(:,i) + finalEstimate(:,1);
    finalEstimateRotatedhkplus3(:,i) = finalEstimateRotatedhkplus3(:,i) + finalEstimate(:,1);
end

%door216Location = [760.4018;999.5193];
hkmin3Translation = [-1040;-579]; % [-137;273]
finalEstimateRotatedTranslatedhkmin3 = finalEstimateRotatedhkmin3 + hkmin3Translation; % translation for hypothesis k-3
hkmin2Translation = finalEstimateRotatedhkmin2(:,strideNumber-174)-finalEstimateRotatedTranslatedhkmin3(:,strideNumber-175);%[-30; 140];
finalEstimateRotatedTranslatedhkmin2 = finalEstimateRotatedhkmin2 - hkmin2Translation; % translation for hypothesis k-2
hkmin1Translation = finalEstimateRotatedhkmin1(:,strideNumber-173)-finalEstimateRotatedTranslatedhkmin3(:,strideNumber-175);%[-30; 140];
finalEstimateRotatedTranslatedhkmin1 = finalEstimateRotatedhkmin1 - hkmin1Translation; % translation for hypothesis k-2
hkTranslation = finalEstimateRotatedhk(:,strideNumber-172)-finalEstimateRotatedTranslatedhkmin3(:,strideNumber-175);%[-30; 140];
finalEstimateRotatedTranslatedhk = finalEstimateRotatedhk - hkTranslation; % translation for hypothesis k-2
hkplus1Translation = finalEstimateRotatedhkplus1(:,strideNumber-171)-finalEstimateRotatedTranslatedhkmin3(:,strideNumber-175);%[-30; 140];
finalEstimateRotatedTranslatedhkplus1 = finalEstimateRotatedhkplus1 - hkplus1Translation; % translation for hypothesis k-2
hkplus2Translation = finalEstimateRotatedhkplus2(:,strideNumber-170)-finalEstimateRotatedTranslatedhkmin3(:,strideNumber-175);%[-30; 140];
finalEstimateRotatedTranslatedhkplus2 = finalEstimateRotatedhkplus2 - hkplus2Translation; % translation for hypothesis k-2
hkplus3Translation = finalEstimateRotatedhkplus3(:,strideNumber-169)-finalEstimateRotatedTranslatedhkmin3(:,strideNumber-175);%[-30; 140];
finalEstimateRotatedTranslatedhkplus3 = finalEstimateRotatedhkplus3 - hkplus3Translation; % translation for hypothesis k-2
probStrides = 10;
figure(13); set(figure(13),'position',[1 59 1114 671]);
set(gcf,'position',[564 61 927 670]);
imshow(img); hold on;
% 1upsamplingRate = 1; muUS = zeros(size(mu,1),upsamplingRate*size(mu,2));
% muUS(1,:) = interp(mu(1,:),upsamplingRate);
% muUS(2,:) = interp(mu(2,:),upsamplingRate);
showRotated = 0;
if (choice == 1 || choice == 2)
    if (showRotated)
        plot(finalEstimateRotatedhkplus3(1,1:end),finalEstimateRotatedhkplus3(2,1:end),'r-x','linewidth',1.5);
        plot(finalEstimateRotatedhkplus3(1,strideNumber-169),finalEstimateRotatedhkplus3(2,strideNumber-169),'ks','linewidth',2,'markerfacecolor','y','markersize',16);
        plot(finalEstimateRotatedhkplus3(1,strideNumber-170),finalEstimateRotatedhkplus3(2,strideNumber-170),'ko','linewidth',2,'markerfacecolor','y','markersize',16);
        % we can take stride k-2 to calculate the heading of the current hypothesis
        %plot(finalEstimateRotatedhkplus3(1,strideNumber-16),finalEstimateRotatedhkplus3(2,strideNumber-16),'ko','linewidth',2,'markerfacecolor','y','markersize',16);
    end
else (choice == 3)
%plot(finalEstimateRotated(1,1:end),finalEstimateRotated(2,1:end),'r-x','linewidth',1.5);
end
plot(finalEstimateRotatedTranslatedhkmin3(1,strideNumber-175:strideNumber-175+probStrides),...
    finalEstimateRotatedTranslatedhkmin3(2,strideNumber-175:strideNumber-175+probStrides),'g-x','linewidth',1.5);
plot(finalEstimateRotatedTranslatedhkmin2(1,strideNumber-174:strideNumber-174+probStrides),...
    finalEstimateRotatedTranslatedhkmin2(2,strideNumber-174:strideNumber-174+probStrides),'y-x','linewidth',1.5);
plot(finalEstimateRotatedTranslatedhkmin1(1,strideNumber-173:strideNumber-173+probStrides),...
    finalEstimateRotatedTranslatedhkmin1(2,strideNumber-173:strideNumber-173+probStrides),'y-x','linewidth',1.5,'color',[1 0.5 0]);
plot(finalEstimateRotatedTranslatedhk(1,strideNumber-172:strideNumber-172+probStrides),...
    finalEstimateRotatedTranslatedhk(2,strideNumber-172:strideNumber-172+probStrides),'y-x','linewidth',1.5,'color',[1 0 0.5]);
plot(finalEstimateRotatedTranslatedhkplus1(1,strideNumber-171:strideNumber-171+probStrides),...
    finalEstimateRotatedTranslatedhkplus1(2,strideNumber-171:strideNumber-171+probStrides),'y-x','linewidth',1.5,'color',[0 0.5 1]);
plot(finalEstimateRotatedTranslatedhkplus2(1,strideNumber-170:strideNumber-170+probStrides),...
    finalEstimateRotatedTranslatedhkplus2(2,strideNumber-170:strideNumber-170+probStrides),'y-x','linewidth',1.5,'color',[0.5 0 0.5]);
plot(finalEstimateRotatedTranslatedhkplus3(1,strideNumber-169:strideNumber-169+probStrides),...
    finalEstimateRotatedTranslatedhkplus3(2,strideNumber-169:strideNumber-169+probStrides),'y-x','linewidth',1.5,'color',[0 0.5 0]);
hold off; legend('hypothesis k-3 trajectory','hypothesis k-2 trajectory',...
    'hypothesis k-1 trajectory','hypothesis k trajectory',...
    'hypothesis k+1 trajectory','hypothesis k+2 trajectory',...
    'hypothesis k+3 trajectory');
set(legend,'fontsize',16,'interpreter','latex','location','northwest');
% figure(8);xlim([660 1410]);ylim([548 1018]);
probStridesVectorhkmin3 = zeros(1,probStrides);
probStridesVectorhkmin2 = zeros(1,probStrides);
probStridesVectorhkmin1 = zeros(1,probStrides);
probStridesVectorhk = zeros(1,probStrides);
probStridesVectorhkplus1 = zeros(1,probStrides);
probStridesVectorhkplus2 = zeros(1,probStrides);
probStridesVectorhkplus3 = zeros(1,probStrides);
for i=1:probStrides
    probStridesVectorhkmin3(i) = imcrop_taha_subpixel_accuracy(choice,finalEstimateRotatedTranslatedhkmin3(:,strideNumber-175+i),0,0,0,0);
    probStridesVectorhkmin2(i) = imcrop_taha_subpixel_accuracy(choice,finalEstimateRotatedTranslatedhkmin2(:,strideNumber-174+i),0,0,0,0);
    probStridesVectorhkmin1(i) = imcrop_taha_subpixel_accuracy(choice,finalEstimateRotatedTranslatedhkmin1(:,strideNumber-173+i),0,0,0,0);
    probStridesVectorhk(i) = imcrop_taha_subpixel_accuracy(choice,finalEstimateRotatedTranslatedhk(:,strideNumber-172+i),0,0,0,0);
    probStridesVectorhkplus1(i) = imcrop_taha_subpixel_accuracy(choice,finalEstimateRotatedTranslatedhkplus1(:,strideNumber-171+i),0,0,0,0);
    probStridesVectorhkplus2(i) = imcrop_taha_subpixel_accuracy(choice,finalEstimateRotatedTranslatedhkplus2(:,strideNumber-170+i),0,0,0,0);
    probStridesVectorhkplus3(i) = imcrop_taha_subpixel_accuracy(choice,finalEstimateRotatedTranslatedhkplus3(:,strideNumber-169+i),0,0,0,0);
end

fprintf('Probability of sequential strides for hypothesis k-3\n');
for i=1:probStrides
    if (i==probStrides)
        fprintf('%.3f\n', probStridesVectorhkmin3(i));
    else
        fprintf('%.3f - ', probStridesVectorhkmin3(i));
    end
end
fprintf('Sum of probabilities for hypothesis k-3 is %.3f\n', sum(probStridesVectorhkmin3));

fprintf('Probability of sequential strides for hypothesis k-2\n');
for i=1:probStrides
    if (i==probStrides)
        fprintf('%.3f\n', probStridesVectorhkmin2(i));
    else
        fprintf('%.3f - ', probStridesVectorhkmin2(i));
    end
end
fprintf('Sum of probabilities for hypothesis k-2 is %.3f\n', sum(probStridesVectorhkmin2));

fprintf('Probability of sequential strides for hypothesis k-1\n');
for i=1:probStrides
    if (i==probStrides)
        fprintf('%.3f\n', probStridesVectorhkmin1(i));
    else
        fprintf('%.3f - ', probStridesVectorhkmin1(i));
    end
end
fprintf('Sum of probabilities for hypothesis k-1 is %.3f\n', sum(probStridesVectorhkmin1));

fprintf('Probability of sequential strides for hypothesis k\n');
for i=1:probStrides
    if (i==probStrides)
        fprintf('%.3f\n', probStridesVectorhk(i));
    else
        fprintf('%.3f - ', probStridesVectorhk(i));
    end
end
fprintf('Sum of probabilities for hypothesis k is %.3f\n', sum(probStridesVectorhk));

fprintf('Probability of sequential strides for hypothesis k+1\n');
for i=1:probStrides
    if (i==probStrides)
        fprintf('%.3f\n', probStridesVectorhkplus1(i));
    else
        fprintf('%.3f - ', probStridesVectorhkplus1(i));
    end
end
fprintf('Sum of probabilities for hypothesis k+1 is %.3f\n', sum(probStridesVectorhkplus1));

fprintf('Probability of sequential strides for hypothesis k+2\n');
for i=1:probStrides
    if (i==probStrides)
        fprintf('%.3f\n', probStridesVectorhkplus2(i));
    else
        fprintf('%.3f - ', probStridesVectorhkplus2(i));
    end
end
fprintf('Sum of probabilities for hypothesis k+2 is %.3f\n', sum(probStridesVectorhkplus2));

fprintf('Probability of sequential strides for hypothesis k+3\n');
for i=1:probStrides
    if (i==probStrides)
        fprintf('%.3f\n', probStridesVectorhkplus3(i));
    else
        fprintf('%.3f - ', probStridesVectorhkplus3(i));
    end
end
fprintf('Sum of probabilities for hypothesis k+3 is %.3f\n', sum(probStridesVectorhkplus3));

%%
figure(14); set(gcf,'position',[213 187 1120 420]);
subplot(241);
stem(1:probStrides, probStridesVectorhkmin3,'g.','linewidth',1.7);
grid on; xlabel('stride');ylabel('probability');
title(['$\sum p_i \;=\;$', num2str(sum(probStridesVectorhkmin3)), ' for hypothesis k-3'],'interpreter','latex');
xticks([1:10]); yticks([0:0.2:1]);
xticklabels({'1','2','3','4','5','6','7','8','9','10'});
yticklabels({'','.2','.4','.6','.8','1'});
set(gca,'GridLineStyle','--','position',[0.0342 0.6001 0.2345 0.3216]);
subplot(242);
stem(1:probStrides, probStridesVectorhkmin2,'y.','linewidth',1.7);
grid on; xlabel('stride');
title(['$\sum p_i \;=\;$', num2str(sum(probStridesVectorhkmin2)), ' for hypothesis k-2'],'interpreter','latex');
xticks([1:10]); yticks([0:0.2:1]);
xticklabels({'1','2','3','4','5','6','7','8','9','10'});
yticklabels({'','','','','',''});
set(gca,'GridLineStyle','--','position',[0.0342+0.2345+0.005 0.6001 0.2345 0.3216]);
subplot(243);
stem(1:probStrides, probStridesVectorhkmin1,'y.','linewidth',1.7,'color',[1 0.5 0]);
grid on; xlabel('stride');
title(['$\sum p_i \;=\;$', num2str(sum(probStridesVectorhkmin1)), ' for hypothesis k-1'],'interpreter','latex');
xticks([1:10]); yticks([0:0.2:1]);
xticklabels({'1','2','3','4','5','6','7','8','9','10'});
yticklabels({'','','','','',''});
set(gca,'GridLineStyle','--','position',[0.0342+2*0.2345+2*0.005 0.6001 0.2345 0.3216]);
subplot(244);
stem(1:probStrides, probStridesVectorhk,'y.','linewidth',1.7,'color',[1 0 0.5]);
grid on; xlabel('stride');
title(['$\sum p_i \;=\;$', num2str(sum(probStridesVectorhk)), ' for hypothesis k'],'interpreter','latex');
xticks([1:10]); yticks([0:0.2:1]);
xticklabels({'1','2','3','4','5','6','7','8','9','10'});
yticklabels({'','','','','',''});
set(gca,'GridLineStyle','--','position',[0.0342+3*0.2345+3*0.005 0.6001 0.2345 0.3216]);
subplot(247);
stem(1:probStrides, probStridesVectorhkplus3,'g.','linewidth',1.7,'color',[0 0.5 0]);
grid on; xlabel('stride');%ylabel('probability');
title(['$\sum p_i \;=\;$', num2str(sum(probStridesVectorhkplus3)), ' for hypothesis k+3'],'interpreter','latex');
xticks([1:10]); yticks([0:0.2:1]);
xticklabels({'1','2','3','4','5','6','7','8','9','10'});
yticklabels({'','','','','',''});
set(gca,'GridLineStyle','--','position',[0.1442+2*(0.2345+0.005) 0.1221 0.2345 0.3216]);
subplot(246);
stem(1:probStrides, probStridesVectorhkplus2,'g.','linewidth',1.7,'color',[0.5 0 0.5]);
grid on; xlabel('stride');%ylabel('probability');
title(['$\sum p_i \;=\;$', num2str(sum(probStridesVectorhkplus2)), ' for hypothesis k+2'],'interpreter','latex');
xticks([1:10]); yticks([0:0.2:1]);
xticklabels({'1','2','3','4','5','6','7','8','9','10'});
yticklabels({'','','','','',''});
set(gca,'GridLineStyle','--','position',[0.1442+0.2345+0.005 0.1221 0.2345 0.3216]);
subplot(245);
stem(1:probStrides, probStridesVectorhkplus1,'g.','linewidth',1.7,'color',[0 0.5 1]);
grid on; xlabel('stride');ylabel('probability');
title(['$\sum p_i \;=\;$', num2str(sum(probStridesVectorhkplus1)), ' for hypothesis k+1'],'interpreter','latex');
xticks([1:10]); yticks([0:0.2:1]);
xticklabels({'1','2','3','4','5','6','7','8','9','10'});
yticklabels({'','.2','.4','.6','.8','1'});
set(gca,'GridLineStyle','--','position',[0.1442 0.1221 0.2345 0.3216]);
%% gradual correction for the trajectory in the room 221
% the hypothesis (stride) with the highest probability has the max rotation and 
% translation. the angle and the shift is gradually distributed to the strides in 
% the room. the very first stride in the room is applied least rotation and shift.
figure(15); set(figure(15),'position',[1 59 1114 671]);
set(gcf,'position',[564 61 927 670]);
imshow(img); hold on;
if (choice == 1 || choice == 2)
plot(mu(1,1:end),mu(2,1:end),'b-x','linewidth',1.5); % this is IMU (EKF+DR+ZUPT) solution
%plot(muUS(1,1:end-18*upsamplingRate),muUS(2,1:end-18*upsamplingRate),'b-.','linewidth',3); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,1:end),finalEstimate(2,1:end),'r-x','linewidth',1.5);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',2,'markerfacecolor','r','markeredgecolor','k','markersize',16);
plot(finalEstimate(1,end),finalEstimate(2,end),'gs','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',16);
else (choice == 3)
plot(mu(1,1:end-30),mu(2,1:end-30), 'b-.','linewidth',2); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,:),finalEstimate(2,:), 'r-','linewidth',2);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',2,'markerfacecolor','r','markeredgecolor','k','markersize',10);
plot(finalEstimate(1,end-30),finalEstimate(2,end-30), 'gs','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',10);
end

% hkplus2angle = -36; % this is the most voted hypothesis
firstStride = 143; % 18
lastStride = strideNumber-172; % 16
roomStrides = lastStride-firstStride;
translationReference = finalEstimateRotatedTranslatedhk(:,lastStride);
for i = 0:roomStrides
    angleTemp = hkangle*(roomStrides-i)/roomStrides;
    rotation_matrixTemp = [cosd(angleTemp) -sind(angleTemp);
                                sind(angleTemp) cosd(angleTemp)];
    finalEstimateRotatedhk(:,lastStride-i) = rotation_matrixTemp*...
        [finalEstimateTemp(1,lastStride-i) finalEstimateTemp(2,lastStride-i)]';
    translationTemp = hkTranslation*(roomStrides-i)/roomStrides;
    finalEstimateRotatedTranslatedhk(:,lastStride-i) = ...
        finalEstimateRotatedhk(:,lastStride-i) - translationTemp; % translation for hypothesis k-2
    if (i == 0)
        translationFix = translationReference - finalEstimateRotatedTranslatedhk(:,lastStride-i);
    end
    plot(finalEstimateRotatedTranslatedhk(1,lastStride-i)+translationFix(1),...
        finalEstimateRotatedTranslatedhk(2,lastStride-i)+translationFix(2),...
        'gx','markersize',12,'linewidth',1.8);
    finalEstimate(:,lastStride-i) = finalEstimateRotatedTranslatedhk(:,lastStride-i)+translationFix;
    fprintf('%i ', lastStride-i);
end
legend('complementary KF','non-recursive estimation','start','stop');
set(legend,'fontsize',16,'interpreter','latex','location','northwest');
hold off;
fprintf('\n');
%% apply non-recursive map-matching to remaining trajectory after leaving the door
for i = lastStride:strideNumber % strideNumber
        x = (mu(1,i) - nx) : 1 : (mu(1,i) + px);
        y = (mu(2,i) - ny) : 1 : (mu(2,i) + py);
        [X,Y] = meshgrid(x,y);
        f_likelihood = exp(-0.5*(((X-mu(1,i))./measSigmaX).^2 ...
            + ((Y-mu(2,i))./measSigmaY).^2));
        fprintf('%i\n',i);
        deltaImuPosition = position_wrt_previous_two_strides...
            (mu(:,i-2),mu(:,i-1),mu(:,i)); % delta sensor likelihood
        % now we need to slice the f_staticFloor correctly
        imuPosition = sensor_likelihood_wrt_previous_two_estimates...
            (finalEstimate(:,i-2),finalEstimate(:,i-1),deltaImuPosition);
        f_staticFloor(:,:,i) = imcrop_taha_subpixel_accuracy...
            (choice,imuPosition,nx,px,ny,py);
        if (choice2 == 20)
                if (i >= 187+25)
                    f_staticFloor(:,:,i) = 255;
                end

        end
        posterior = f_likelihood.*f_staticFloor(:,:,i);
        posterior(:,:,i) = posterior/max(max(posterior));
        maxValue = max(max(posterior(:,:,i)));
        [rowsOfMaxes,colsOfMaxes] = find(posterior(:,:,i) == maxValue);
        finalEstimate(:,i) = [mean(colsOfMaxes); mean(rowsOfMaxes)] - ...
                             [nx; ny] + imuPosition - [1;1];
end

% the very first stride in the room gets least rotation and shift
figure(16); set(figure(16),'position',[1 59 1114 671]);
set(gcf,'position',[564 61 927 670]);
imshow(img); hold on;
if (choice == 1 || choice == 2)
plot(mu(1,1:end),mu(2,1:end),'b-x','linewidth',1.5); % this is IMU (EKF+DR+ZUPT) solution
%plot(muUS(1,1:end-18*upsamplingRate),muUS(2,1:end-18*upsamplingRate),'b-.','linewidth',3); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,1:end),finalEstimate(2,1:end),'r-x','linewidth',1.5);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',2,'markerfacecolor','r','markeredgecolor','k','markersize',16);
plot(finalEstimate(1,end),finalEstimate(2,end),'gs','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',16);
% plot(finalEstimate(1,143),finalEstimate(2,143),'go','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',16);
% plot(finalEstimate(1,187),finalEstimate(2,187),'go','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',16);
% plot(finalEstimate(1,187+25),finalEstimate(2,187+25),'go','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',16);
else (choice == 3)
plot(mu(1,1:end-30),mu(2,1:end-30), 'b-.','linewidth',2); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,:),finalEstimate(2,:), 'r-','linewidth',2);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',2,'markerfacecolor','r','markeredgecolor','k','markersize',10);
plot(finalEstimate(1,end-30),finalEstimate(2,end-30), 'gs','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',10);
end
% plot(finalEstimateRotatedTranslatedhkplus2(1,strideNumber-15),finalEstimateRotatedTranslatedhkplus2(2,strideNumber-15),'ko','markersize',16);
% plot(finalEstimateRotatedTranslatedhkplus2(1,19),finalEstimateRotatedTranslatedhkplus2(2,19),'ko','markersize',16);
hold off;
legend('complementary KF','MHT with non-recursive map-matching','start','stop');
set(legend,'fontsize',16,'interpreter','latex','location','northwest');
% figure(11);xlim([660 1410]);ylim([548 1018]);
% Room 221 traversal is over. The pedestrian is walking to PCV Lab now.
%% illustration of hypotheses
figure(17); set(figure(17),'position',[1 59 1114 671]);
set(gcf,'position',[564 61 927 670]);
imshow(img); hold on;
% 1upsamplingRate = 1; muUS = zeros(size(mu,1),upsamplingRate*size(mu,2));
% muUS(1,:) = interp(mu(1,:),upsamplingRate);
% muUS(2,:) = interp(mu(2,:),upsamplingRate);
if (choice == 1 || choice == 2)
plot(mu(1,1:end),mu(2,1:end),'b-x','linewidth',1.5); % this is IMU (EKF+DR+ZUPT) solution
%plot(muUS(1,1:end-18*upsamplingRate),muUS(2,1:end-18*upsamplingRate),'b-.','linewidth',3); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,1:end),finalEstimate(2,1:end),'r-x','linewidth',1.5);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',2,'markerfacecolor','r','markeredgecolor','k','markersize',16);
plot(finalEstimate(1,end),finalEstimate(2,end),'gs','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',16);
plot(finalEstimate(1,strideNumber-95),finalEstimate(2,strideNumber-95),'kp','linewidth',2,'markerfacecolor','g','markersize',16);
plot(finalEstimate(1,strideNumber-94),finalEstimate(2,strideNumber-94),'kv','linewidth',2,'markerfacecolor','y','markersize',16);
plot(finalEstimate(1,strideNumber-93),finalEstimate(2,strideNumber-93),'ko','linewidth',2,'markerfacecolor',[1 0.5 0],'markersize',16);
plot(finalEstimate(1,strideNumber-92),finalEstimate(2,strideNumber-92),'ks','linewidth',2,'markerfacecolor',[1 0 0.5],'markersize',16);
plot(finalEstimate(1,strideNumber-91),finalEstimate(2,strideNumber-91),'kh','linewidth',2,'markerfacecolor',[0 0.5 1],'markersize',16);
plot(finalEstimate(1,strideNumber-90),finalEstimate(2,strideNumber-90),'kd','linewidth',2,'markerfacecolor',[0.5 0 0.5],'markersize',16);
plot(finalEstimate(1,strideNumber-89),finalEstimate(2,strideNumber-89),'k^','linewidth',2,'markerfacecolor',[0 0.5 0],'markersize',16);
% plot(finalEstimate(1,18),finalEstimate(2,18),'k<','linewidth',2,'markerfacecolor','k','markersize',16);
else (choice == 3)
plot(mu(1,1:end-30),mu(2,1:end-30), 'b-.','linewidth',2); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,:),finalEstimate(2,:), 'r-','linewidth',2);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',2,'markerfacecolor','r','markeredgecolor','k','markersize',16);
plot(finalEstimate(1,end-30),finalEstimate(2,end-30), 'gs','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',16);
end
hold off;
legend('complementary KF','non-recursive Bayesian map-matching','pedestrian start',...
    'pedestrian stop','hypothesis k-3','hypothesis k-2','hypothesis k-1','hypothesis k',...
    'hypothesis k+1','hypothesis k+2','hypothesis k+3');
set(legend,'fontsize',16,'interpreter','latex','location','northwest');
% figure(7);xlim([660 1410]);ylim([548 1018]);
%% hypothesis k-3, k-2, k-1, k, k+1, k+2, k+3
% the angles are calculated according to IMU location & actual door location
hkmin3angle = -47; % rotation for hypothesis k-3, computed wrt k-3 and k-4
hkmin2angle = -90; % rotation for hypothesis k-2.
hkmin1angle = -48; % rotation for hypothesis k-1.
hkangle = -3; % rotation for hypothesis k.
hkplus1angle = 64; % rotation for hypothesis k+1.
% burda kaldik kplus2
hkplus2angle = 65; % rotation for hypothesis k+2.
hkplus3angle = 95; % rotation for hypothesis k+3.
% for each hypothesis, the rotation matrix is wrt to actual door location
rotation_matrixkmin3angle = [cosd(hkmin3angle) -sind(hkmin3angle);
    sind(hkmin3angle) cosd(hkmin3angle)];
rotation_matrixkmin2angle = [cosd(hkmin2angle) -sind(hkmin2angle);
    sind(hkmin2angle) cosd(hkmin2angle)];
rotation_matrixkmin1angle = [cosd(hkmin1angle) -sind(hkmin1angle);
    sind(hkmin1angle) cosd(hkmin1angle)];
rotation_matrixkangle = [cosd(hkangle) -sind(hkangle);
    sind(hkangle) cosd(hkangle)];
rotation_matrixkplus1angle = [cosd(hkplus1angle) -sind(hkplus1angle);
    sind(hkplus1angle) cosd(hkplus1angle)];
rotation_matrixkplus2angle = [cosd(hkplus2angle) -sind(hkplus2angle);
    sind(hkplus2angle) cosd(hkplus2angle)];
rotation_matrixkplus3angle = [cosd(hkplus3angle) -sind(hkplus3angle);
    sind(hkplus3angle) cosd(hkplus3angle)];

finalEstimateRotatedhkmin3 = zeros(2,strideNumber);
finalEstimateRotatedhkmin2 = zeros(2,strideNumber);
finalEstimateRotatedhkmin1 = zeros(2,strideNumber);
finalEstimateRotatedhk = zeros(2,strideNumber);
finalEstimateRotatedhkplus1 = zeros(2,strideNumber);
finalEstimateRotatedhkplus2 = zeros(2,strideNumber);
finalEstimateRotatedhkplus3 = zeros(2,strideNumber);

finalEstimateTemp = zeros(2,strideNumber);

for i = 1:strideNumber
    finalEstimateTemp(:,i) = finalEstimate(:,i) - finalEstimate(:,1);
end
for idx = 1:strideNumber
    finalEstimateRotatedhkmin3(:,idx) = rotation_matrixkmin3angle*...
        [finalEstimateTemp(1,idx) finalEstimateTemp(2,idx)]';
    finalEstimateRotatedhkmin2(:,idx) = rotation_matrixkmin2angle*...
        [finalEstimateTemp(1,idx) finalEstimateTemp(2,idx)]';
    finalEstimateRotatedhkmin1(:,idx) = rotation_matrixkmin1angle*...
        [finalEstimateTemp(1,idx) finalEstimateTemp(2,idx)]';
    finalEstimateRotatedhk(:,idx) = rotation_matrixkangle*...
        [finalEstimateTemp(1,idx) finalEstimateTemp(2,idx)]';
    finalEstimateRotatedhkplus1(:,idx) = rotation_matrixkplus1angle*...
        [finalEstimateTemp(1,idx) finalEstimateTemp(2,idx)]';
    finalEstimateRotatedhkplus2(:,idx) = rotation_matrixkplus2angle*...
        [finalEstimateTemp(1,idx) finalEstimateTemp(2,idx)]';
    finalEstimateRotatedhkplus3(:,idx) = rotation_matrixkplus3angle*...
        [finalEstimateTemp(1,idx) finalEstimateTemp(2,idx)]';
end
for i=1:strideNumber
    finalEstimateRotatedhkmin3(:,i) = finalEstimateRotatedhkmin3(:,i) + finalEstimate(:,1);
    finalEstimateRotatedhkmin2(:,i) = finalEstimateRotatedhkmin2(:,i) + finalEstimate(:,1);
    finalEstimateRotatedhkmin1(:,i) = finalEstimateRotatedhkmin1(:,i) + finalEstimate(:,1);
    finalEstimateRotatedhk(:,i) = finalEstimateRotatedhk(:,i) + finalEstimate(:,1);
    finalEstimateRotatedhkplus1(:,i) = finalEstimateRotatedhkplus1(:,i) + finalEstimate(:,1);
    finalEstimateRotatedhkplus2(:,i) = finalEstimateRotatedhkplus2(:,i) + finalEstimate(:,1);
    finalEstimateRotatedhkplus3(:,i) = finalEstimateRotatedhkplus3(:,i) + finalEstimate(:,1);
end

%door216Location = [760.4018;999.5193];
hkmin3Translation = [-123;-810]; % [-137;273]
finalEstimateRotatedTranslatedhkmin3 = finalEstimateRotatedhkmin3 + hkmin3Translation; % translation for hypothesis k-3
hkmin2Translation = finalEstimateRotatedhkmin2(:,strideNumber-94)-finalEstimateRotatedTranslatedhkmin3(:,strideNumber-95);%[-30; 140];
finalEstimateRotatedTranslatedhkmin2 = finalEstimateRotatedhkmin2 - hkmin2Translation; % translation for hypothesis k-2
hkmin1Translation = finalEstimateRotatedhkmin1(:,strideNumber-93)-finalEstimateRotatedTranslatedhkmin3(:,strideNumber-95);%[-30; 140];
finalEstimateRotatedTranslatedhkmin1 = finalEstimateRotatedhkmin1 - hkmin1Translation; % translation for hypothesis k-2
hkTranslation = finalEstimateRotatedhk(:,strideNumber-92)-finalEstimateRotatedTranslatedhkmin3(:,strideNumber-95);%[-30; 140];
finalEstimateRotatedTranslatedhk = finalEstimateRotatedhk - hkTranslation; % translation for hypothesis k-2
hkplus1Translation = finalEstimateRotatedhkplus1(:,strideNumber-91)-finalEstimateRotatedTranslatedhkmin3(:,strideNumber-95);%[-30; 140];
finalEstimateRotatedTranslatedhkplus1 = finalEstimateRotatedhkplus1 - hkplus1Translation; % translation for hypothesis k-2
hkplus2Translation = finalEstimateRotatedhkplus2(:,strideNumber-90)-finalEstimateRotatedTranslatedhkmin3(:,strideNumber-95);%[-30; 140];
finalEstimateRotatedTranslatedhkplus2 = finalEstimateRotatedhkplus2 - hkplus2Translation; % translation for hypothesis k-2
hkplus3Translation = finalEstimateRotatedhkplus3(:,strideNumber-89)-finalEstimateRotatedTranslatedhkmin3(:,strideNumber-95);%[-30; 140];
finalEstimateRotatedTranslatedhkplus3 = finalEstimateRotatedhkplus3 - hkplus3Translation; % translation for hypothesis k-2
probStrides = 10;
figure(18); set(figure(18),'position',[1 59 1114 671]);
set(gcf,'position',[564 61 927 670]);
imshow(img); hold on;
% 1upsamplingRate = 1; muUS = zeros(size(mu,1),upsamplingRate*size(mu,2));
% muUS(1,:) = interp(mu(1,:),upsamplingRate);
% muUS(2,:) = interp(mu(2,:),upsamplingRate);
showRotated = 0;
if (choice == 1 || choice == 2)
    if (showRotated)
        plot(finalEstimateRotatedhkmin3(1,1:end),finalEstimateRotatedhkmin3(2,1:end),'r-x','linewidth',1.5);
%         plot(finalEstimateRotatedhkmin3(1,strideNumber-94),finalEstimateRotatedhkmin3(2,strideNumber-94),'ko','linewidth',2,'markerfacecolor','y','markersize',16);
        plot(finalEstimateRotatedhkmin3(1,strideNumber-95),finalEstimateRotatedhkmin3(2,strideNumber-95),'ks','linewidth',2,'markerfacecolor','y','markersize',16);
        plot(finalEstimateRotatedhkmin3(1,strideNumber-96),finalEstimateRotatedhkmin3(2,strideNumber-96),'ko','linewidth',2,'markerfacecolor','y','markersize',16);
        % we can take stride k-2 to calculate the heading of the current hypothesis
        %plot(finalEstimateRotatedhkplus3(1,strideNumber-16),finalEstimateRotatedhkplus3(2,strideNumber-16),'ko','linewidth',2,'markerfacecolor','y','markersize',16);
    end
else (choice == 3)
%plot(finalEstimateRotated(1,1:end),finalEstimateRotated(2,1:end),'r-x','linewidth',1.5);
end
plot(finalEstimateRotatedTranslatedhkmin3(1,strideNumber-95:strideNumber-95+probStrides),...
    finalEstimateRotatedTranslatedhkmin3(2,strideNumber-95:strideNumber-95+probStrides),'g-x','linewidth',1.5);
plot(finalEstimateRotatedTranslatedhkmin2(1,strideNumber-94:strideNumber-94+probStrides),...
    finalEstimateRotatedTranslatedhkmin2(2,strideNumber-94:strideNumber-94+probStrides),'y-x','linewidth',1.5);
plot(finalEstimateRotatedTranslatedhkmin1(1,strideNumber-93:strideNumber-93+probStrides),...
    finalEstimateRotatedTranslatedhkmin1(2,strideNumber-93:strideNumber-93+probStrides),'y-x','linewidth',1.5,'color',[1 0.5 0]);
plot(finalEstimateRotatedTranslatedhk(1,strideNumber-92:strideNumber-92+probStrides),...
    finalEstimateRotatedTranslatedhk(2,strideNumber-92:strideNumber-92+probStrides),'y-x','linewidth',1.5,'color',[1 0 0.5]);
plot(finalEstimateRotatedTranslatedhkplus1(1,strideNumber-91:strideNumber-91+probStrides),...
    finalEstimateRotatedTranslatedhkplus1(2,strideNumber-91:strideNumber-91+probStrides),'y-x','linewidth',1.5,'color',[0 0.5 1]);
plot(finalEstimateRotatedTranslatedhkplus2(1,strideNumber-90:strideNumber-90+probStrides),...
    finalEstimateRotatedTranslatedhkplus2(2,strideNumber-90:strideNumber-90+probStrides),'y-x','linewidth',1.5,'color',[0.5 0 0.5]);
plot(finalEstimateRotatedTranslatedhkplus3(1,strideNumber-89:strideNumber-89+probStrides),...
    finalEstimateRotatedTranslatedhkplus3(2,strideNumber-89:strideNumber-89+probStrides),'y-x','linewidth',1.5,'color',[0 0.5 0]);
hold off; legend('hypothesis k-3 trajectory','hypothesis k-2 trajectory',...
    'hypothesis k-1 trajectory','hypothesis k trajectory',...
    'hypothesis k+1 trajectory','hypothesis k+2 trajectory',...
    'hypothesis k+3 trajectory');
set(legend,'fontsize',16,'interpreter','latex','location','northwest');
% figure(8);xlim([660 1410]);ylim([548 1018]);
probStridesVectorhkmin3 = zeros(1,probStrides);
probStridesVectorhkmin2 = zeros(1,probStrides);
probStridesVectorhkmin1 = zeros(1,probStrides);
probStridesVectorhk = zeros(1,probStrides);
probStridesVectorhkplus1 = zeros(1,probStrides);
probStridesVectorhkplus2 = zeros(1,probStrides);
probStridesVectorhkplus3 = zeros(1,probStrides);
for i=1:probStrides
    probStridesVectorhkmin3(i) = imcrop_taha_subpixel_accuracy(choice,finalEstimateRotatedTranslatedhkmin3(:,strideNumber-95+i),0,0,0,0);
    probStridesVectorhkmin2(i) = imcrop_taha_subpixel_accuracy(choice,finalEstimateRotatedTranslatedhkmin2(:,strideNumber-94+i),0,0,0,0);
    probStridesVectorhkmin1(i) = imcrop_taha_subpixel_accuracy(choice,finalEstimateRotatedTranslatedhkmin1(:,strideNumber-93+i),0,0,0,0);
    probStridesVectorhk(i) = imcrop_taha_subpixel_accuracy(choice,finalEstimateRotatedTranslatedhk(:,strideNumber-92+i),0,0,0,0);
    probStridesVectorhkplus1(i) = imcrop_taha_subpixel_accuracy(choice,finalEstimateRotatedTranslatedhkplus1(:,strideNumber-91+i),0,0,0,0);
    probStridesVectorhkplus2(i) = imcrop_taha_subpixel_accuracy(choice,finalEstimateRotatedTranslatedhkplus2(:,strideNumber-90+i),0,0,0,0);
    probStridesVectorhkplus3(i) = imcrop_taha_subpixel_accuracy(choice,finalEstimateRotatedTranslatedhkplus3(:,strideNumber-89+i),0,0,0,0);
end

fprintf('Probability of sequential strides for hypothesis k-3\n');
for i=1:probStrides
    if (i==probStrides)
        fprintf('%.3f\n', probStridesVectorhkmin3(i));
    else
        fprintf('%.3f - ', probStridesVectorhkmin3(i));
    end
end
fprintf('Sum of probabilities for hypothesis k-3 is %.3f\n', sum(probStridesVectorhkmin3));

fprintf('Probability of sequential strides for hypothesis k-2\n');
for i=1:probStrides
    if (i==probStrides)
        fprintf('%.3f\n', probStridesVectorhkmin2(i));
    else
        fprintf('%.3f - ', probStridesVectorhkmin2(i));
    end
end
fprintf('Sum of probabilities for hypothesis k-2 is %.3f\n', sum(probStridesVectorhkmin2));

fprintf('Probability of sequential strides for hypothesis k-1\n');
for i=1:probStrides
    if (i==probStrides)
        fprintf('%.3f\n', probStridesVectorhkmin1(i));
    else
        fprintf('%.3f - ', probStridesVectorhkmin1(i));
    end
end
fprintf('Sum of probabilities for hypothesis k-1 is %.3f\n', sum(probStridesVectorhkmin1));

fprintf('Probability of sequential strides for hypothesis k\n');
for i=1:probStrides
    if (i==probStrides)
        fprintf('%.3f\n', probStridesVectorhk(i));
    else
        fprintf('%.3f - ', probStridesVectorhk(i));
    end
end
fprintf('Sum of probabilities for hypothesis k is %.3f\n', sum(probStridesVectorhk));

fprintf('Probability of sequential strides for hypothesis k+1\n');
for i=1:probStrides
    if (i==probStrides)
        fprintf('%.3f\n', probStridesVectorhkplus1(i));
    else
        fprintf('%.3f - ', probStridesVectorhkplus1(i));
    end
end
fprintf('Sum of probabilities for hypothesis k+1 is %.3f\n', sum(probStridesVectorhkplus1));

fprintf('Probability of sequential strides for hypothesis k+2\n');
for i=1:probStrides
    if (i==probStrides)
        fprintf('%.3f\n', probStridesVectorhkplus2(i));
    else
        fprintf('%.3f - ', probStridesVectorhkplus2(i));
    end
end
fprintf('Sum of probabilities for hypothesis k+2 is %.3f\n', sum(probStridesVectorhkplus2));

fprintf('Probability of sequential strides for hypothesis k+3\n');
for i=1:probStrides
    if (i==probStrides)
        fprintf('%.3f\n', probStridesVectorhkplus3(i));
    else
        fprintf('%.3f - ', probStridesVectorhkplus3(i));
    end
end
fprintf('Sum of probabilities for hypothesis k+3 is %.3f\n', sum(probStridesVectorhkplus3));

%%
figure(19); set(gcf,'position',[213 187 1120 420]);
subplot(241);
stem(1:probStrides, probStridesVectorhkmin3,'g.','linewidth',1.7);
grid on; xlabel('stride');ylabel('probability');
title(['$\sum p_i \;=\;$', num2str(sum(probStridesVectorhkmin3)), ' for hypothesis k-3'],'interpreter','latex');
xticks([1:10]); yticks([0:0.2:1]);
xticklabels({'1','2','3','4','5','6','7','8','9','10'});
yticklabels({'','.2','.4','.6','.8','1'});
set(gca,'GridLineStyle','--','position',[0.0342 0.6001 0.2345 0.3216]);
subplot(242);
stem(1:probStrides, probStridesVectorhkmin2,'y.','linewidth',1.7);
grid on; xlabel('stride');
title(['$\sum p_i \;=\;$', num2str(sum(probStridesVectorhkmin2)), ' for hypothesis k-2'],'interpreter','latex');
xticks([1:10]); yticks([0:0.2:1]);
xticklabels({'1','2','3','4','5','6','7','8','9','10'});
yticklabels({'','','','','',''});
set(gca,'GridLineStyle','--','position',[0.0342+0.2345+0.005 0.6001 0.2345 0.3216]);
subplot(243);
stem(1:probStrides, probStridesVectorhkmin1,'y.','linewidth',1.7,'color',[1 0.5 0]);
grid on; xlabel('stride');
title(['$\sum p_i \;=\;$', num2str(sum(probStridesVectorhkmin1)), ' for hypothesis k-1'],'interpreter','latex');
xticks([1:10]); yticks([0:0.2:1]);
xticklabels({'1','2','3','4','5','6','7','8','9','10'});
yticklabels({'','','','','',''});
set(gca,'GridLineStyle','--','position',[0.0342+2*0.2345+2*0.005 0.6001 0.2345 0.3216]);
subplot(244);
stem(1:probStrides, probStridesVectorhk,'y.','linewidth',1.7,'color',[1 0 0.5]);
grid on; xlabel('stride');
title(['$\sum p_i \;=\;$', num2str(sum(probStridesVectorhk)), ' for hypothesis k'],'interpreter','latex');
xticks([1:10]); yticks([0:0.2:1]);
xticklabels({'1','2','3','4','5','6','7','8','9','10'});
yticklabels({'','','','','',''});
set(gca,'GridLineStyle','--','position',[0.0342+3*0.2345+3*0.005 0.6001 0.2345 0.3216]);
subplot(247);
stem(1:probStrides, probStridesVectorhkplus3,'g.','linewidth',1.7,'color',[0 0.5 0]);
grid on; xlabel('stride');%ylabel('probability');
title(['$\sum p_i \;=\;$', num2str(sum(probStridesVectorhkplus3)), ' for hypothesis k+3'],'interpreter','latex');
xticks([1:10]); yticks([0:0.2:1]);
xticklabels({'1','2','3','4','5','6','7','8','9','10'});
yticklabels({'','','','','',''});
set(gca,'GridLineStyle','--','position',[0.1442+2*(0.2345+0.005) 0.1221 0.2345 0.3216]);
subplot(246);
stem(1:probStrides, probStridesVectorhkplus2,'g.','linewidth',1.7,'color',[0.5 0 0.5]);
grid on; xlabel('stride');%ylabel('probability');
title(['$\sum p_i \;=\;$', num2str(sum(probStridesVectorhkplus2)), ' for hypothesis k+2'],'interpreter','latex');
xticks([1:10]); yticks([0:0.2:1]);
xticklabels({'1','2','3','4','5','6','7','8','9','10'});
yticklabels({'','','','','',''});
set(gca,'GridLineStyle','--','position',[0.1442+0.2345+0.005 0.1221 0.2345 0.3216]);
subplot(245);
stem(1:probStrides, probStridesVectorhkplus1,'g.','linewidth',1.7,'color',[0 0.5 1]);
grid on; xlabel('stride');ylabel('probability');
title(['$\sum p_i \;=\;$', num2str(sum(probStridesVectorhkplus1)), ' for hypothesis k+1'],'interpreter','latex');
xticks([1:10]); yticks([0:0.2:1]);
xticklabels({'1','2','3','4','5','6','7','8','9','10'});
yticklabels({'','.2','.4','.6','.8','1'});
set(gca,'GridLineStyle','--','position',[0.1442 0.1221 0.2345 0.3216]);
%% gradual correction for the trajectory in the room 221
% the hypothesis (stride) with the highest probability has the max rotation and 
% translation. the angle and the shift is gradually distributed to the strides in 
% the room. the very first stride in the room is applied least rotation and shift.
figure(20); set(figure(20),'position',[1 59 1114 671]);
set(gcf,'position',[564 61 927 670]);
imshow(img); hold on;
if (choice == 1 || choice == 2)
plot(mu(1,1:end),mu(2,1:end),'b-x','linewidth',1.5); % this is IMU (EKF+DR+ZUPT) solution
%plot(muUS(1,1:end-18*upsamplingRate),muUS(2,1:end-18*upsamplingRate),'b-.','linewidth',3); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,1:end),finalEstimate(2,1:end),'r-x','linewidth',1.5);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',2,'markerfacecolor','r','markeredgecolor','k','markersize',16);
plot(finalEstimate(1,end),finalEstimate(2,end),'gs','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',16);
plot(finalEstimate(1,212),finalEstimate(2,212),'go','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',16);
plot(finalEstimate(1,strideNumber-93),finalEstimate(2,strideNumber-93),'go','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',16);
else (choice == 3)
plot(mu(1,1:end-30),mu(2,1:end-30), 'b-.','linewidth',2); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,:),finalEstimate(2,:), 'r-','linewidth',2);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',2,'markerfacecolor','r','markeredgecolor','k','markersize',10);
plot(finalEstimate(1,end-30),finalEstimate(2,end-30), 'gs','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',10);
end

% hkplus2angle = -36; % this is the most voted hypothesis
firstStride = 212; % 18
lastStride = strideNumber-93; % 16
roomStrides = lastStride-firstStride;
translationReference = finalEstimateRotatedTranslatedhkmin1(:,lastStride);
for i = 0:roomStrides
    angleTemp = hkmin1angle*(roomStrides-i)/roomStrides;
    rotation_matrixTemp = [cosd(angleTemp) -sind(angleTemp);
                                sind(angleTemp) cosd(angleTemp)];
    finalEstimateRotatedhkmin1(:,lastStride-i) = rotation_matrixTemp*...
        [finalEstimateTemp(1,lastStride-i) finalEstimateTemp(2,lastStride-i)]';
    translationTemp = hkmin1Translation*(roomStrides-i)/roomStrides;
    finalEstimateRotatedTranslatedhkmin1(:,lastStride-i) = ...
        finalEstimateRotatedhkmin1(:,lastStride-i) - translationTemp; % translation for hypothesis k-2
    if (i == 0)
        translationFix = translationReference - finalEstimateRotatedTranslatedhkmin1(:,lastStride-i);
    end
    plot(finalEstimateRotatedTranslatedhkmin1(1,lastStride-i)+translationFix(1),...
        finalEstimateRotatedTranslatedhkmin1(2,lastStride-i)+translationFix(2),...
        'gx','markersize',12,'linewidth',1.8);
    finalEstimate(:,lastStride-i) = finalEstimateRotatedTranslatedhkmin1(:,lastStride-i)+translationFix;
    fprintf('%i ', lastStride-i);
end
legend('complementary KF','non-recursive estimation','start','stop');
set(legend,'fontsize',16,'interpreter','latex','location','northwest');
hold off;
fprintf('\n');
%% apply non-recursive map-matching to remaining trajectory after leaving the door
for i = lastStride:strideNumber % strideNumber
        x = (mu(1,i) - nx) : 1 : (mu(1,i) + px);
        y = (mu(2,i) - ny) : 1 : (mu(2,i) + py);
        [X,Y] = meshgrid(x,y);
        f_likelihood = exp(-0.5*(((X-mu(1,i))./measSigmaX).^2 ...
            + ((Y-mu(2,i))./measSigmaY).^2));
        fprintf('%i\n',i);
        deltaImuPosition = position_wrt_previous_two_strides...
            (mu(:,i-2),mu(:,i-1),mu(:,i)); % delta sensor likelihood
        % now we need to slice the f_staticFloor correctly
        imuPosition = sensor_likelihood_wrt_previous_two_estimates...
            (finalEstimate(:,i-2),finalEstimate(:,i-1),deltaImuPosition);
        f_staticFloor(:,:,i) = imcrop_taha_subpixel_accuracy...
            (choice,imuPosition,nx,px,ny,py);
%         if (choice2 == 20)
%                 if (i >= 187+25)
%                     f_staticFloor(:,:,i) = 255;
%                 end
% 
%         end
        posterior = f_likelihood.*f_staticFloor(:,:,i);
        posterior(:,:,i) = posterior/max(max(posterior));
        maxValue = max(max(posterior(:,:,i)));
        [rowsOfMaxes,colsOfMaxes] = find(posterior(:,:,i) == maxValue);
        finalEstimate(:,i) = [mean(colsOfMaxes); mean(rowsOfMaxes)] - ...
                             [nx; ny] + imuPosition - [1;1];
end

% the very first stride in the room gets least rotation and shift
figure(21); set(figure(21),'position',[1 59 1114 671]);
set(gcf,'position',[564 61 927 670]);
imshow(img); hold on;
if (choice == 1 || choice == 2)
plot(mu(1,1:end),mu(2,1:end),'b-x','linewidth',1.5); % this is IMU (EKF+DR+ZUPT) solution
%plot(muUS(1,1:end-18*upsamplingRate),muUS(2,1:end-18*upsamplingRate),'b-.','linewidth',3); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,1:end),finalEstimate(2,1:end),'r-x','linewidth',1.5);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',2,'markerfacecolor','r','markeredgecolor','k','markersize',16);
plot(finalEstimate(1,end),finalEstimate(2,end),'gs','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',16);
% plot(finalEstimate(1,143),finalEstimate(2,143),'go','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',16);
% plot(finalEstimate(1,firstStride),finalEstimate(2,firstStride),'go','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',16);
% plot(finalEstimate(1,lastStride),finalEstimate(2,lastStride),'go','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',16);
else (choice == 3)
plot(mu(1,1:end-30),mu(2,1:end-30), 'b-.','linewidth',2); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,:),finalEstimate(2,:), 'r-','linewidth',2);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',2,'markerfacecolor','r','markeredgecolor','k','markersize',10);
plot(finalEstimate(1,end-30),finalEstimate(2,end-30), 'gs','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',10);
end
% plot(finalEstimateRotatedTranslatedhkplus2(1,strideNumber-15),finalEstimateRotatedTranslatedhkplus2(2,strideNumber-15),'ko','markersize',16);
% plot(finalEstimateRotatedTranslatedhkplus2(1,19),finalEstimateRotatedTranslatedhkplus2(2,19),'ko','markersize',16);
hold off;
legend('complementary KF','MHT with non-recursive map-matching','start','stop');
set(legend,'fontsize',16,'interpreter','latex','location','northwest');
% figure(11);xlim([660 1410]);ylim([548 1018]);
% Room 221 traversal is over. The pedestrian is walking to PCV Lab now.