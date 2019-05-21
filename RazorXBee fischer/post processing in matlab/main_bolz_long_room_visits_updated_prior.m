% this is the code modified from Fischer's tutorial
% Inertial pedestrian tracking.
% Accompanying article in IEEE Pervasive Magazine.

% For best results use a foot-mounted inertial measurement unit with an
% accelerometer range greater than 10g and 2a gyroscope range greater than
% 900 degrees per second and at least 50 samples per second. The IMU is NOT
% required to estimate orientations.

clear all;
close all;
clc;
% Read data from file for gyro bias estimation.
% Data should include timestamps (seconds), 3 axis accelerations (m/s^2), 
% 3 axis gyroscopic rates of turn (rad/s).
disp('Enter 1 to select Bolz, and 20 to select long room visits.');
choice = input('Enter 1 for Bolz Hall, 2 for Caldwell, 3 for around PCV: ');
if (choice == 1)
    choice2 = input('Enter which traverse it is: ');
end
if (choice == 1)
    fileName = '../bolz_hall_gyro_data_for_bias_estimation_wired_57600_%i.txt';
    %gyroDataForBiasEstimationName = '../bolz_hall_gyro_data_for_bias_estimation_wired_57600_100Hz.txt';
    gyroDataForBiasEstimationName = sprintf(fileName, choice2);
else if (choice == 2)
    gyroDataForBiasEstimationName = '../caldwell_lab_gyro_data_for_bias_estimation_wired_57600_2.txt';
    %gyroDataForBiasEstimationName = '../caldwell_lab_gyro_data_for_bias_estimation_wired_57600.txt';
    else if (choice == 3)
            gyroDataForBiasEstimationName = '../around pcv lab multiple times/around_pcv_gyro_data_for_bias_estimation_wired_57600_multiple.txt';
    else
        fprintf('Invalid input!\n');
        pause();
    end
    end
end

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
        pedestrianTraverseName = '../caldwell_lab_wired_57600_2.txt';
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
                                                                sf = 1.06; angle = 185;
                                                            else if (choice2 == 21)
                                                                    sf = 1.07; angle = 183;
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
    end
else if (choice == 2)
    sf = 1.06; % scale factor
    angle = 182; % 181   1892     190 for multiple traverse
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
startPixelCoordinates{1} = [1363 + 1; 735 + 1];
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
load('bolz_hall_pdf_updated3.mat');
staticFloorPDF{1} = double(bolzPDF)./255;
delete bolzPDF
load('caldwell_lab_pdf_updated.mat');
staticFloorPDF{2} = double(caldwellPDF)./255;
delete caldwellPDF

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
% DOOR ENTRANCE REGION COORDINATES TO DETECT WHEN THE USER ENTERS THE ROOMS
% unlike PF map-matching, non-recursive Bayesian map-matching based MHT
% only needs room door coordinates (as a region) in the proposed algorithm 
rooms.room1.door.xmin = 1207;
rooms.room1.door.xmax = 1227;
rooms.room1.door.ymin = 703-5;
rooms.room1.door.ymax = 718+5;
% map-matching will be deactivated when pedestrian is headed to the rooms
rooms.room1.door.insideRegion = false;
for i = 1 : strideNumber % strideNumber
    if (i < strideNumber+1) % 3 for constant velocity MM, 4 for constant acceleration MM
        % nx = 30; px = 30; ny = 30; py = 30;
        x = (mu(1,i) - nx) : 1 : (mu(1,i) + px);
        y = (mu(2,i) - ny) : 1 : (mu(2,i) + py);
        [X,Y] = meshgrid(x,y);
        f_likelihood = exp(-0.5*(((X-mu(1,i))./measSigmaX).^2 ...
            + ((Y-mu(2,i))./measSigmaY).^2));
        % I needed to write my own subpixel crop function
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
        % automatically deactivate map-matching when the pedestrian walks
        % into the rooms
        if (rooms.room1.door.insideRegion == true)
                f_staticFloor(:,:,i) = 255;
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
        % automatically checking if the user is headed to room 1
        if (~rooms.room1.door.insideRegion && ...
                room_door_region_check(finalEstimate(:,i),...
                rooms.room1.door.xmin,rooms.room1.door.xmax,...
                rooms.room1.door.ymin,rooms.room1.door.ymax))
            rooms.room1.door.insideRegion = true;
            fprintf('Stride #%i is at room 1 entrance.\n',i);
            rooms.room1.doorRegionStride = i;
        end
        % if the user is at door 1 region, next stride will not affected by
        % map-matching, which means MHT has to start to detect the last
        % stride in the room
        
            
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
rooms.room1.firstStride = rooms.room1.doorRegionStride+1;
%%
figure(5); clf; set(figure(5),'position',[564 61 927 670]);
imshow(img); hold on;
plot(mu(1,1:end),mu(2,1:end),'b-x','linewidth',1.5); % this is IMU (EKF+DR+ZUPT) solution
%plot(muUS(1,1:end-18*upsamplingRate),muUS(2,1:end-18*upsamplingRate),'b-.','linewidth',3); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,1:end),finalEstimate(2,1:end),'r-x','linewidth',1.5);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',2,'markerfacecolor','r','markeredgecolor','k','markersize',10);
plot([rooms.room1.door.xmin rooms.room1.door.xmax ...
    rooms.room1.door.xmax rooms.room1.door.xmin rooms.room1.door.xmin], ...
    [rooms.room1.door.ymin rooms.room1.door.ymin rooms.room1.door.ymax ...
    rooms.room1.door.ymax rooms.room1.door.ymin],'g-','linewidth',2);
plot(finalEstimate(1,rooms.room1.doorRegionStride),...
    finalEstimate(2,rooms.room1.doorRegionStride),'yo','linewidth',2,...
    'markeredgecolor','y','markersize',10);
hold off;
legend('error state KF with ZUPT','non-recursive Bayesian map-matching based MHT','start','stop');
set(legend,'fontsize',16,'interpreter','latex','location','northeast');
%%
figure(6); set(gcf,'position',[371 151 535 422]);
imshow(staticFloorPDF{choice},'colormap',jet(255));
colormap jet; set(gca, 'YDir', 'reverse'); hold on;
plot(finalEstimate(1,rooms.room1.firstStride),...
    finalEstimate(2,rooms.room1.firstStride),'wo','linewidth',2,...
    'markeredgecolor','w','markersize',10);
% plot(finalEstimate(1,rooms.room2.firstStride+1),...
%     finalEstimate(2,rooms.room2.firstStride+1),'wo','linewidth',2,...
%     'markeredgecolor','w','markersize',10);
plot(mu(1,:),mu(2,:), 'wx-','linewidth',2); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,:),finalEstimate(2,:), 'kx-','linewidth',2);
%plot(finalEstimate(1,till),finalEstimate(2,till), 'ko-','linewidth',2);
legend('First stride in room 1','First stride in room 2',...
    'location','southwest','color','y');
set(legend,'interpreter','latex','fontsize',16);
hold off;
%% room 1 hypothesis illustrations
figure(7); clf; set(gcf,'position',[564 61 927 670]);
imshow(img); hold on;
plot(mu(1,1:end),mu(2,1:end),'b-x','linewidth',1.5); % this is IMU (EKF+DR+ZUPT) solution
%plot(muUS(1,1:end-18*upsamplingRate),muUS(2,1:end-18*upsamplingRate),'b-.','linewidth',3); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,1:end),finalEstimate(2,1:end),'r-x','linewidth',1.5);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',2,'markerfacecolor','r','markeredgecolor','k','markersize',16);
plot(finalEstimate(1,end),finalEstimate(2,end),'gs','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',16);
rooms.room1.numberOfHyp = 56; % max 56
for q=1:rooms.room1.numberOfHyp % starts with strideNumber-350 and decreases 1 by 1
    plot(finalEstimate(1,rooms.room1.doorRegionStride+q),...
    finalEstimate(2,rooms.room1.doorRegionStride+q),'ko','linewidth',2,'markersize',10);
    fprintf('strideNumber - %i = stride #%i = hypothesis #%i\n',351-q,...
        rooms.room1.doorRegionStride+q, q);
end
hold off;
%% Room 1 all hypothesis probability computations
% hypothesis k-3, k-2, k-1, k, k+1, k+2, k+3
% the angles are calculated according to IMU location & actual door location
% rotation angles of associated hypotheses. these angles can be
% automatically determined yet manually computed by using MATLAB's
% illustrative environment. In order to determine which stride leaves the
% room, we assume that the user leaves the room and return the hallway in
% right angle that is 90 degrees.
%save('continue_from_here_1.mat');
% load continue_from_here_1.mat;
close all; clc;
lastFigNumber = 7;
rooms.room1.numberOfHyp = 56; % 56
hypRotAngle = zeros(1,rooms.room1.numberOfHyp);
hypRotAngle(1)=-173;hypRotAngle(2)=157;hypRotAngle(3)=119;hypRotAngle(4)=101;
hypRotAngle(5)=94;hypRotAngle(6)=97;hypRotAngle(7)=27;hypRotAngle(8)=-69;
hypRotAngle(9)=-84;hypRotAngle(10)=-76;hypRotAngle(11)=-104;hypRotAngle(12)=164;
hypRotAngle(13)=104;hypRotAngle(14)=94;hypRotAngle(15)=96;hypRotAngle(16)=4;
hypRotAngle(17)=-68; hypRotAngle(18)=-81; hypRotAngle(19)=-80;
hypRotAngle(20)=-158; hypRotAngle(21)=120; hypRotAngle(22)=92;
hypRotAngle(23)=96; hypRotAngle(24)=58; hypRotAngle(25)=-50;
hypRotAngle(26)=-82; hypRotAngle(27)=-76; hypRotAngle(28)=-101;
hypRotAngle(29)=170; hypRotAngle(30)=106; hypRotAngle(31)=99;
hypRotAngle(32)=99; hypRotAngle(33)=74; hypRotAngle(34)=-36;
hypRotAngle(35)=-78; hypRotAngle(36)=-76; hypRotAngle(37)=-73;
hypRotAngle(38)=-138; hypRotAngle(39)=129; hypRotAngle(40)=101;
hypRotAngle(41)=105; hypRotAngle(42)=89; hypRotAngle(43)=-16;
hypRotAngle(44)=-71; hypRotAngle(45)=-81; hypRotAngle(46)=-83;
hypRotAngle(47)=-75; hypRotAngle(48)=4; hypRotAngle(49)=9; % hypRotAngle(48)=10
hypRotAngle(50)=-53; hypRotAngle(51)=-85; hypRotAngle(52)=-86;
hypRotAngle(53)=-78;hypRotAngle(54)=-89;hypRotAngle(55)=-83;
hypRotAngle(56)=-82;
% roomExitAngle can be -10 -5 hypRotAngle +5 +10
% -10 yields h1 with score of 7.61, -5 yields h1 with 7.13
% 0 yields hyp 48 with 7.85, +5 yields hyp 48 with 9.74
% and +10 hypt 48 w/ 9.11
roomExitAngle = 5;
hypRotAngle = hypRotAngle + roomExitAngle; % 10
subsequentStrides = 10; % design parameter - number of subsequent strides
% for each hypothesis, the rotation matrix is wrt to actual door location
hypRotMatrix = zeros(2,2,rooms.room1.numberOfHyp);
for i=1:rooms.room1.numberOfHyp
    hypRotMatrix(:,:,i) = [cosd(hypRotAngle(i)) -sind(hypRotAngle(i));
                           sind(hypRotAngle(i)) cosd(hypRotAngle(i))];
end

finalEstimateRotated = zeros(2,strideNumber,rooms.room1.numberOfHyp);
finalEstimateTemp = zeros(2,strideNumber);

for i = 1:strideNumber
    finalEstimateTemp(:,i) = finalEstimate(:,i) - finalEstimate(:,1);
end
for i=1:rooms.room1.numberOfHyp
    for idx = 1:strideNumber
        finalEstimateRotated(:,idx,i) = hypRotMatrix(:,:,i)*...
            [finalEstimateTemp(1,idx) finalEstimateTemp(2,idx)]';
    end
end
for j=1:rooms.room1.numberOfHyp
    for i=1:strideNumber
        finalEstimateRotated(:,i,j) = finalEstimateRotated(:,i,j) + ...
            finalEstimate(:,1);
    end
end

%door216Location = [760.4018;999.5193];
hypTranslation = zeros(2,rooms.room1.numberOfHyp);
finalEstimateRotatedTranslated=zeros(2,strideNumber,rooms.room1.numberOfHyp);
% following translation vectors are calculated according to room 1 door coordinates
% recall that we only need room door coordinates in non-recusive bayesian
% map-matching with MHT approach unlike PF that need room wall coordinates
% in addition to hallway wall coordinates
rooms.room1.door.x = 1217; rooms.room1.door.y = 704;
hypTranslation(:,1) = [rooms.room1.door.x; rooms.room1.door.y] - ...
    finalEstimateRotated(:,rooms.room1.firstStride,1);
% hypTranslation(:,1) = [-278;-93]; % room 1 door coordinates determine this
% hypTranslation(:,1) = [-265;-110]; % + 10 angle
% hypTranslation(:,1) = [-286;-67]; % - 10 angle
finalEstimateRotatedTranslated(:,:,1) = finalEstimateRotated(:,:,1) + hypTranslation(:,1);
% (strideNumber - rooms.room1.doorRegionStride - 1) = 350
% (strideNumber - rooms.room1.doorRegionStride - i) = 349, 348, ...
for i=2:rooms.room1.numberOfHyp
    hypTranslation(:,i) = finalEstimateRotated(:,strideNumber - ...
        (strideNumber - rooms.room1.doorRegionStride - i),i) - ...
        finalEstimateRotatedTranslated(:,strideNumber - ...
        (strideNumber - rooms.room1.doorRegionStride - 1),1);
    finalEstimateRotatedTranslated(:,:,i) = finalEstimateRotated(:,:,i) - hypTranslation(:,i);
end

% illustrations
maxColors = 8;
color = zeros(maxColors,3); % different colors for different hypothesis
color(1,:) = [0,1,0]; color(2,:) = [1,1,0]; color(3,:) = [1 0.5 0];
color(4,:) = [1,0,.5]; color(5,:) = [0,0.5,0]; color(6,:) = [0.5 0 0.5];
color(7,:) = [0 0.5 1]; color(8,:) = [0.5 0.5 0.5];

lastFigNumber = subsequent_strides_inside_room(img,maxColors,...
    finalEstimateRotatedTranslated,strideNumber,...
    rooms.room1.doorRegionStride,subsequentStrides,color,...
    rooms.room1.numberOfHyp,finalEstimateRotated,choice,lastFigNumber);
% SUBSEQUENT STRIDE PROBABILITIES
probStrides = zeros(rooms.room1.numberOfHyp,subsequentStrides);
for i=1:rooms.room1.numberOfHyp
    for j=1:subsequentStrides
        probStrides(i,j) = imcrop_taha_subpixel_accuracy(choice,...
            finalEstimateRotatedTranslated(:,strideNumber-(strideNumber - ...
            rooms.room1.doorRegionStride - i)+j,i),0,0,0,0);
        if (isnan(probStrides(i,j)))
            probStrides(i,j) = 0.004;
        end
    end
end

lastFigNumber = illustration_of_probabilities(subsequentStrides,...
    probStrides,lastFigNumber,color,rooms.room1.numberOfHyp);
% ========= DETERMINING the LAST STRIDE IN ROOM 211 ================
% total probabilities of all hypotheses
hypTotalProbs = sum(probStrides');
[maxValue,rooms.room1.lastHyp] = max(hypTotalProbs);
rooms.room1.lastStride = rooms.room1.doorRegionStride + rooms.room1.lastHyp;
rooms.room1.numberOfHypInsideRoom = rooms.room1.lastStride - ...
    rooms.room1.firstStride + 1;
rooms.room1.maxAngle = hypRotAngle(rooms.room1.numberOfHypInsideRoom);
rooms.room1.maxTranslation = hypTranslation(:,rooms.room1.numberOfHypInsideRoom);
lastFigNumber = lastFigNumber + 1;
figure(lastFigNumber); clf; ...
    set(gcf,'Name','Determining the last stride in room 1'); hold on;
plot(rooms.room1.lastHyp,hypTotalProbs(rooms.room1.lastHyp),...
    'ro','markersize',12,'linewidth',1.2);
plot(1:size(probStrides,1),hypTotalProbs,'linewidth',1.3);
xlabel('hypothesis','interpreter','latex','fontsize',15);
ylabel('total probability','interpreter','latex','fontsize',15);
grid on; axis([1 size(probStrides,1) 0 10]); hold off;
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
set(gca,'GridLineStyle','--');
legend(['$\displaystyle \sum_{i=1}^{10} h_{', ...
    num2str(rooms.room1.lastHyp), '}^{',num2str(roomExitAngle),...
    '}(i) = ', num2str(maxValue),'$']);
set(legend,'fontsize',13,'interpreter','latex','location','north');
% SUMMARY of ROOM 1 visit
fprintf('The pedestrian stepped inside door 1 region at stride %i.\n',...
    rooms.room1.doorRegionStride);
fprintf('Map-matching will be deactivated for strides in the room.\n');
fprintf('First stride in room 1 is stride #%i that is hypothesis 1.\n',...
    rooms.room1.firstStride);
fprintf('Last stride in room 1 is stride #%i that is hypothesis #%i.\n',...
    rooms.room1.lastStride,rooms.room1.lastHyp);
fprintf('# of strides = # of hypotheses in room 1 is %i.\n',...
    rooms.room1.numberOfHypInsideRoom);
title(['Hypothesis ',num2str(rooms.room1.lastHyp),...
    ' is the last stride in room 1'],'interpreter','latex',...
    'fontsize',14);
%% GRADUAL ROOM TRAJECTORY CORRECTION & MAP-MATCHING REACTIVATION
% now last stride in room 1 is determined so firstly we can 
% correct room trajectory left behind in an intuitive manner, and secondly
% subsequent strides of last stride should be exposed to map-matching as
% they are located in the hallway
%============= STEP 1 - ROOM TRAJECTORY CORRECTION ========================
% gradual correction for the trajectory in the room
% the hypothesis (stride) with the highest probability has the max rotation 
% and translation. the angle and the shift is gradually distributed to the 
% strides in the room. the very first stride in the room is exposed to least 
% rotation and shift.
lastFigNumber = lastFigNumber+1;
figure(lastFigNumber); set(gcf,'position',[564 61 927 670],...
    'name','Gradual correction of room trajectory');
imshow(img); hold on;
if (choice == 1 || choice == 2)
plot(mu(1,1:end),mu(2,1:end),'b-x','linewidth',1.5); % this is IMU (EKF+DR+ZUPT) solution
%plot(muUS(1,1:end-18*upsamplingRate),muUS(2,1:end-18*upsamplingRate),'b-.','linewidth',3); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,1:end),finalEstimate(2,1:end),'r-x','linewidth',1.5);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',2,'markerfacecolor','r','markeredgecolor','k','markersize',16);
plot(finalEstimate(1,end),finalEstimate(2,end),'gs','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',16);
end
firstStride = rooms.room1.firstStride;
lastStride = rooms.room1.lastStride; % 16
roomStrides = lastStride-firstStride;
translationReference = finalEstimateRotatedTranslated(:,lastStride,...
    rooms.room1.lastHyp);
for i = 0:roomStrides
    angleTemp = rooms.room1.maxAngle*(roomStrides-i)/roomStrides;
    rotation_matrixTemp = [cosd(angleTemp) -sind(angleTemp);
                           sind(angleTemp) cosd(angleTemp)];
    finalEstimateRotated(:,lastStride-i,rooms.room1.lastHyp) = ...
        rotation_matrixTemp*[finalEstimateTemp(1,lastStride-i) ...
                             finalEstimateTemp(2,lastStride-i)]';
    translationTemp = rooms.room1.maxTranslation*(roomStrides-i)/roomStrides;
    finalEstimateRotatedTranslated(:,lastStride-i,rooms.room1.lastHyp) = ...
        finalEstimateRotated(:,lastStride-i,rooms.room1.lastHyp) - ...
        translationTemp; % translation for hypothesis k-2
    if (i == 0)
        translationFix = translationReference - ...
            finalEstimateRotatedTranslated(:,lastStride-i,rooms.room1.lastHyp);
    end
    plot(finalEstimateRotatedTranslated(1,lastStride-i,...
        rooms.room1.lastHyp)+translationFix(1),...
        finalEstimateRotatedTranslated(2,lastStride-i,...
        rooms.room1.lastHyp)+translationFix(2),...
        'gx','markersize',12,'linewidth',1.8);
    finalEstimate(:,lastStride-i) = finalEstimateRotatedTranslated(:,...
        lastStride-i,rooms.room1.lastHyp)+translationFix;
    fprintf('%i ', lastStride-i);
end
legend('error state KF with ZUPT','non-recursive Bayesian filter','start','stop');
set(legend,'fontsize',16,'interpreter','latex','location','northwest');
hold off;
fprintf('\n');
%% Room 211 traversal is over. The pedestrian is walking to Room 216 now.
% ============= STEP 2 - REACTIVATE MAP-MATCHING ========================
% as we know the last stride in room 1, map-matching is activated for
% subsequent strides automatically until the pedestrian is headed to room 1
xShift = -219; yShift = 50;
rooms.room2.door.xmin = rooms.room1.door.xmin + xShift;
rooms.room2.door.xmax = rooms.room1.door.xmax + xShift;
rooms.room2.door.ymin = rooms.room1.door.ymin + yShift;
rooms.room2.door.ymax = rooms.room1.door.ymax + yShift;
% map-matching will be deactivated automatically when pedestrian is headed 
% to the room 2.
rooms.room2.door.insideRegion = false;
for i = lastStride:strideNumber % strideNumber
        x = (mu(1,i) - nx) : 1 : (mu(1,i) + px);
        y = (mu(2,i) - ny) : 1 : (mu(2,i) + py);
        [X,Y] = meshgrid(x,y);
        f_likelihood = exp(-0.5*(((X-mu(1,i))./measSigmaX).^2 ...
            + ((Y-mu(2,i))./measSigmaY).^2));
%         fprintf('%i\n',i);
        deltaImuPosition = position_wrt_previous_two_strides...
            (mu(:,i-2),mu(:,i-1),mu(:,i)); % delta sensor likelihood
        % now we need to slice the f_staticFloor correctly
        imuPosition = sensor_likelihood_wrt_previous_two_estimates...
            (finalEstimate(:,i-2),finalEstimate(:,i-1),deltaImuPosition);
        f_staticFloor(:,:,i) = imcrop_taha_subpixel_accuracy...
            (choice,imuPosition,nx,px,ny,py);
        if (rooms.room2.door.insideRegion == true)
                f_staticFloor(:,:,i) = 255;
        end
        
        posterior = f_likelihood.*f_staticFloor(:,:,i);
        posterior(:,:,i) = posterior/max(max(posterior));
        maxValue = max(max(posterior(:,:,i)));
        [rowsOfMaxes,colsOfMaxes] = find(posterior(:,:,i) == maxValue);
        finalEstimate(:,i) = [mean(colsOfMaxes); mean(rowsOfMaxes)] - ...
                             [nx; ny] + imuPosition - [1;1];
                         
        % automatically checking if the user is headed to room 1
        if (~rooms.room2.door.insideRegion && ...
                room_door_region_check(finalEstimate(:,i),...
                rooms.room2.door.xmin,rooms.room2.door.xmax,...
                rooms.room2.door.ymin,rooms.room2.door.ymax))
            rooms.room2.door.insideRegion = true;
            fprintf('Stride #%i is at room 2 entrance.\n',i);
            rooms.room2.doorRegionStride = i;
        end
        % if the user is at door 2 region, next stride will not affected by
        % map-matching, which means MHT has to start to detect the last
        % stride in the room
end
rooms.room2.firstStride = rooms.room2.doorRegionStride+1;
lastFigNumber = lastFigNumber + 1;
figure(lastFigNumber); clf; set(gcf,'position',[564 61 927 670],'name',...
    'Room trajectory corrected, map-matching reactivated, next room entering stride detected');
imshow(img); hold on;
plot(mu(1,1:end),mu(2,1:end),'b-x','linewidth',1.5); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,1:end),finalEstimate(2,1:end),'r-x','linewidth',1.5);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',2,'markerfacecolor','r','markeredgecolor','k','markersize',10);
plot(finalEstimate(1,rooms.room1.doorRegionStride),...
    finalEstimate(2,rooms.room1.doorRegionStride),'yo','linewidth',2,...
    'markeredgecolor','y','markersize',10);
plot(finalEstimate(1,rooms.room2.doorRegionStride),...
    finalEstimate(2,rooms.room2.doorRegionStride),'yo','linewidth',2,...
    'markeredgecolor','y','markersize',10);
plot([rooms.room1.door.xmin rooms.room1.door.xmax ...
    rooms.room1.door.xmax rooms.room1.door.xmin rooms.room1.door.xmin], ...
    [rooms.room1.door.ymin rooms.room1.door.ymin rooms.room1.door.ymax ...
    rooms.room1.door.ymax rooms.room1.door.ymin],'g-','linewidth',2);
plot([rooms.room2.door.xmin rooms.room2.door.xmax ...
    rooms.room2.door.xmax rooms.room2.door.xmin rooms.room2.door.xmin], ...
    [rooms.room2.door.ymin rooms.room2.door.ymin rooms.room2.door.ymax ...
    rooms.room2.door.ymax rooms.room2.door.ymin],'g-','linewidth',2);
hold off;
legend('error state KF with ZUPT','non-recursive Bayesian map-matching based MHT','start','stop');
set(legend,'fontsize',16,'interpreter','latex','location','northeast');
%% room 2 hypothesis illustrations
% this is just for the sake of illustration - it has no effect on the algorithm
lastFigNumber = lastFigNumber+1;
figure(lastFigNumber); clf;
set(gcf,'position',[564 61 927 670],'name','Room 2 hypotheses (for the sake of illustration)');
imshow(img); hold on;
plot(mu(1,1:end),mu(2,1:end),'b-x','linewidth',1.5); % this is IMU (EKF+DR+ZUPT) solution
%plot(muUS(1,1:end-18*upsamplingRate),muUS(2,1:end-18*upsamplingRate),'b-.','linewidth',3); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,1:end),finalEstimate(2,1:end),'r-x','linewidth',1.5);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',2,'markerfacecolor','r','markeredgecolor','k','markersize',16);
plot(finalEstimate(1,end),finalEstimate(2,end),'gs','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',16);
% max 64 - just for the sake of illustration. see the code below.
rooms.room2.numberOfHyp = 64;
for q=1:rooms.room2.numberOfHyp % starts with strideNumber-350 and decreases 1 by 1
    plot(finalEstimate(1,rooms.room2.doorRegionStride+q),...
    finalEstimate(2,rooms.room2.doorRegionStride+q),'ko','linewidth',2,'markersize',10);
    fprintf('strideNumber - %i = stride #%i = hypothesis #%i\n',351-q,...
        rooms.room2.firstStride+q, q);
end
plot([rooms.room1.door.xmin rooms.room1.door.xmax ...
    rooms.room1.door.xmax rooms.room1.door.xmin rooms.room1.door.xmin], ...
    [rooms.room1.door.ymin rooms.room1.door.ymin rooms.room1.door.ymax ...
    rooms.room1.door.ymax rooms.room1.door.ymin],'g-','linewidth',2);
plot(finalEstimate(1,rooms.room1.doorRegionStride),...
    finalEstimate(2,rooms.room1.doorRegionStride),'yo','linewidth',2,...
    'markeredgecolor','y','markersize',10);
plot([rooms.room2.door.xmin rooms.room2.door.xmax ...
    rooms.room2.door.xmax rooms.room2.door.xmin rooms.room2.door.xmin], ...
    [rooms.room2.door.ymin rooms.room2.door.ymin rooms.room2.door.ymax ...
    rooms.room2.door.ymax rooms.room2.door.ymin],'g-','linewidth',2);
plot(finalEstimate(1,rooms.room2.doorRegionStride),...
    finalEstimate(2,rooms.room2.doorRegionStride),'yo','linewidth',2,...
    'markeredgecolor','y','markersize',10);
hold off;
%% Room 2 (Room 216) all hypothesis probability computations
% In order to determine which stride leaves the room, we assume that the 
% user leaves the room and return the hallway in angles that is 90 degrees 
% and also +/- 5,10 degrees. Following angles are manually calculated by
% exploiting MATLAB's illustrative environment. All values can esily be
% computed automatically. Note that angles are determined using the current
% strides and the previous stride.
close all; clc;
rooms.room2.numberOfHyp = 64; % 64
hypRotAngle = zeros(1,rooms.room2.numberOfHyp);
hypRotAngle(1)=159; hypRotAngle(2)=181;hypRotAngle(3)=183;hypRotAngle(4)=-152;
hypRotAngle(5)=-121;hypRotAngle(6)=-106;hypRotAngle(7)=-102;hypRotAngle(8)=-66;
hypRotAngle(9)=31;hypRotAngle(10)=75;hypRotAngle(11)=78;hypRotAngle(12)=85;
hypRotAngle(13)=-156;hypRotAngle(14)=-114;hypRotAngle(15)=-114;
hypRotAngle(16)=-79;hypRotAngle(17)=26; hypRotAngle(18)=71; hypRotAngle(19)=79;
hypRotAngle(20)=131; hypRotAngle(21)=-130; hypRotAngle(22)=-109;
hypRotAngle(23)=-116; hypRotAngle(24)=-34; hypRotAngle(25)=51;
hypRotAngle(26)=68; hypRotAngle(27)=91; hypRotAngle(28)=-162;
hypRotAngle(29)=-123; hypRotAngle(30)=-113; hypRotAngle(31)=-69;
hypRotAngle(32)=24; hypRotAngle(33)=63; hypRotAngle(34)=56;
hypRotAngle(35)=106; hypRotAngle(36)=-159; hypRotAngle(37)=-124;
hypRotAngle(38)=-124; hypRotAngle(39)=-40; hypRotAngle(40)=35;
hypRotAngle(41)=47; hypRotAngle(42)=71; hypRotAngle(43)=156;
hypRotAngle(44)=-141; hypRotAngle(45)=-139; hypRotAngle(46)=-81;
hypRotAngle(47)=7; hypRotAngle(48)=78; hypRotAngle(49)=88; % hypRotAngle(48)=10
hypRotAngle(50)=16; hypRotAngle(51)=-86; hypRotAngle(52)=-157;
hypRotAngle(53)=-179;hypRotAngle(54)=-139;hypRotAngle(55)=-31;
hypRotAngle(56)=38;hypRotAngle(57)=67;hypRotAngle(58)=54;
hypRotAngle(59)=6;hypRotAngle(60)=-29;hypRotAngle(61)=-49;
hypRotAngle(62)=-56;hypRotAngle(63)=-42;hypRotAngle(64)=25;
% -10 hyp 61 w/ 6.44, -5 hyp 61 w/ 7.40, 0 hyp 33 w/ 6.37
% +5 hyp 63 w 7.72, +10 hyp 62 w/ 9.9
roomExitAngle = 10;
hypRotAngle = hypRotAngle + roomExitAngle; % -10 -5 hypRotAngle +5 +10

% for each hypothesis, the rotation matrix is wrt to actual door location
hypRotMatrix = zeros(2,2,rooms.room2.numberOfHyp);
for i=1:rooms.room2.numberOfHyp
    hypRotMatrix(:,:,i) = [cosd(hypRotAngle(i)) -sind(hypRotAngle(i));
                           sind(hypRotAngle(i)) cosd(hypRotAngle(i))];
end

finalEstimateRotated = zeros(2,strideNumber,rooms.room2.numberOfHyp);
finalEstimateTemp = zeros(2,strideNumber);

for i = 1:strideNumber
    finalEstimateTemp(:,i) = finalEstimate(:,i) - finalEstimate(:,1);
end
for i=1:rooms.room2.numberOfHyp
    for idx = 1:strideNumber
        finalEstimateRotated(:,idx,i) = hypRotMatrix(:,:,i)*...
            [finalEstimateTemp(1,idx) finalEstimateTemp(2,idx)]';
    end
end
for j=1:rooms.room2.numberOfHyp
    for i=1:strideNumber
        finalEstimateRotated(:,i,j) = finalEstimateRotated(:,i,j) + ...
            finalEstimate(:,1);
    end
end

%door216Location = [760.4018;999.5193];
hypTranslation = zeros(2,rooms.room2.numberOfHyp);
finalEstimateRotatedTranslated=zeros(2,strideNumber,rooms.room2.numberOfHyp);
% following translation vectors are calculated according to room 1 door coordinates
% recall that we only need room door coordinates in non-recusive bayesian
% map-matching with MHT approach unlike PF that need room wall coordinates
% in addition to hallway wall coordinates
rooms.room2.door.x = 998; rooms.room2.door.y = 766;
hypTranslation(:,1) = [rooms.room2.door.x; rooms.room2.door.y] - ...
                       finalEstimateRotated(:,rooms.room2.firstStride,1);
% hypTranslation(:,1) = [-278;-93]; % room 1 door coordinates determine this
% hypTranslation(:,1) = [-265;-110]; % + 10 angle
% hypTranslation(:,1) = [-286;-67]; % - 10 angle
finalEstimateRotatedTranslated(:,:,1) = finalEstimateRotated(:,:,1) + hypTranslation(:,1);
% (strideNumber - rooms.room1.doorRegionStride - 1) = 350
% (strideNumber - rooms.room1.doorRegionStride - i) = 349, 348, ...
for i=2:rooms.room2.numberOfHyp
    hypTranslation(:,i) = finalEstimateRotated(:,strideNumber - ...
        (strideNumber - rooms.room2.doorRegionStride - i),i) - ...
        finalEstimateRotatedTranslated(:,strideNumber - ...
        (strideNumber - rooms.room2.doorRegionStride - 1),1);
    finalEstimateRotatedTranslated(:,:,i) = finalEstimateRotated(:,:,i)-...
        hypTranslation(:,i);
end

lastFigNumber = subsequent_strides_inside_room(img,maxColors,...
    finalEstimateRotatedTranslated,strideNumber,...
    rooms.room2.doorRegionStride,subsequentStrides,color,...
    rooms.room2.numberOfHyp,finalEstimateRotated,choice,lastFigNumber);
% SUBSEQUENT STRIDE PROBABILITIES
probStrides = zeros(rooms.room2.numberOfHyp,subsequentStrides);
for i=1:rooms.room2.numberOfHyp
    for j=1:subsequentStrides
        probStrides(i,j) = imcrop_taha_subpixel_accuracy(choice,...
            finalEstimateRotatedTranslated(:,strideNumber-(strideNumber - ...
            rooms.room2.doorRegionStride - i)+j,i),0,0,0,0);
        if (isnan(probStrides(i,j)))
            probStrides(i,j) = 0.004;
        end
    end
end

lastFigNumber = illustration_of_probabilities(subsequentStrides,...
    probStrides,lastFigNumber,color,rooms.room2.numberOfHyp);
% ========= DETERMINING the LAST STRIDE IN ROOM 2 (Room 216) ================
% total probabilities of all hypotheses
hypTotalProbs = sum(probStrides');
[maxValue,rooms.room2.lastHyp] = max(hypTotalProbs);
rooms.room2.lastStride = rooms.room2.doorRegionStride + rooms.room2.lastHyp;
rooms.room2.numberOfHypInsideRoom = rooms.room2.lastStride - ...
    rooms.room2.firstStride + 1;
rooms.room2.maxAngle = hypRotAngle(rooms.room2.numberOfHypInsideRoom);
rooms.room2.maxTranslation = hypTranslation(:,rooms.room2.numberOfHypInsideRoom);
lastFigNumber = lastFigNumber+1;
figure(lastFigNumber); clf;
set(gcf,'Name','Determining the last stride in room 2'); hold on;
plot(rooms.room2.lastHyp, hypTotalProbs(rooms.room2.lastHyp),...
    'ro','markersize',12,'linewidth',1.2);
plot(1:size(probStrides,1), hypTotalProbs,'linewidth',1.3);
xlabel('hypothesis','interpreter','latex','fontsize',15);
ylabel('total probability','interpreter','latex','fontsize',15);
grid on; axis([1 size(probStrides,1) 0 10]); hold off;
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
set(gca,'GridLineStyle','--');
legend(['$\displaystyle \sum_{i=1}^{10} h_{', num2str(rooms.room2.lastHyp), ...
    '}^{',num2str(roomExitAngle),'}(i) = ', num2str(maxValue),'$']);
set(legend,'fontsize',13,'interpreter','latex','location','north');
% SUMMARY of ROOM 1 visit
fprintf('The pedestrian stepped inside door 2 region at stride %i.\n',...
    rooms.room2.doorRegionStride);
fprintf('Map-matching will be deactivated for strides in the room.\n');
fprintf('First stride in room 2 is stride #%i that is hypothesis 1.\n',...
    rooms.room2.firstStride);
fprintf('Last stride in room 2 is stride #%i that is hypothesis #%i.\n',...
    rooms.room2.lastStride,rooms.room2.lastHyp);
fprintf('# of strides = # of hypotheses in room 2 is %i.\n',...
    rooms.room2.numberOfHypInsideRoom);
title(['Hypothesis ',num2str(rooms.room2.lastHyp),...
    ' is the last stride in room 1'],'interpreter','latex',...
    'fontsize',14);
%% now last stride in room 2 is determined so firstly we can 
% correct room trajectory left behind in an intuitive manner, and secondly
% subsequent strides of last stride should be exposed to map-matching as
% they are located in the hallway
% ============= STEP 1 - ROOM TRAJECTORY CORRECTION ========================
% gradual correction for the trajectory in the room
% the hypothesis (stride) with the highest probability has the max rotation 
% and translation. the angle and the shift is gradually distributed to the 
% strides in the room. the very first stride in the room is exposed to least 
% rotation and shift.
% save continue_from_here_2.mat;
% load continue_from_here_2.mat;
lastFigNumber = lastFigNumber+1;
figure(lastFigNumber); set(gcf,'position',[564 61 927 670],...
    'name','Gradual correction of room 2 (room 216)');
imshow(img); hold on;
if (choice == 1 || choice == 2)
plot(mu(1,1:end),mu(2,1:end),'b-x','linewidth',1.5); % this is IMU (EKF+DR+ZUPT) solution
%plot(muUS(1,1:end-18*upsamplingRate),muUS(2,1:end-18*upsamplingRate),'b-.','linewidth',3); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,1:end),finalEstimate(2,1:end),'r-x','linewidth',1.5);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',2,'markerfacecolor','r','markeredgecolor','k','markersize',16);
plot(finalEstimate(1,end),finalEstimate(2,end),'gs','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',16);
end
firstStride = rooms.room2.firstStride;
lastStride = rooms.room2.lastStride; % 16
roomStrides = lastStride-firstStride;
translationReference = finalEstimateRotatedTranslated(:,lastStride,...
    rooms.room2.lastHyp);
for i = 0:roomStrides
    angleTemp = rooms.room2.maxAngle*(roomStrides-i)/roomStrides;
    rotation_matrixTemp = [cosd(angleTemp) -sind(angleTemp);
                           sind(angleTemp) cosd(angleTemp)];
    finalEstimateRotated(:,lastStride-i,rooms.room2.lastHyp) = ...
        rotation_matrixTemp*[finalEstimateTemp(1,lastStride-i) ...
                             finalEstimateTemp(2,lastStride-i)]';
    translationTemp = rooms.room2.maxTranslation*(roomStrides-i)/roomStrides;
    finalEstimateRotatedTranslated(:,lastStride-i,rooms.room2.lastHyp) = ...
        finalEstimateRotated(:,lastStride-i,rooms.room2.lastHyp) - ...
        translationTemp; % translation for hypothesis k-2
    if (i == 0)
        translationFix = translationReference - ...
            finalEstimateRotatedTranslated(:,lastStride-i,rooms.room2.lastHyp);
    end
    plot(finalEstimateRotatedTranslated(1,lastStride-i,...
        rooms.room2.lastHyp)+translationFix(1),...
        finalEstimateRotatedTranslated(2,lastStride-i,...
        rooms.room2.lastHyp)+translationFix(2),...
        'gx','markersize',12,'linewidth',1.8);
    finalEstimate(:,lastStride-i) = finalEstimateRotatedTranslated(:,...
        lastStride-i,rooms.room2.lastHyp)+translationFix;
    fprintf('%i ', lastStride-i);
end
legend('error state KF with ZUPT','non-recursive Bayesian filter','start','stop');
set(legend,'fontsize',16,'interpreter','latex','location','northwest');
hold off;
fprintf('\n');
%% Room 216 traversal is over. The pedestrian is walking to Room 221 now.
% ============= STEP 2 - REACTIVATE MAP-MATCHING ========================
% as we know the last stride in room 1, map-matching is activated for
% subsequent strides automatically until the pedestrian is headed to room 1
xShift = -338; yShift = -50;
rooms.room3.door.xmin = rooms.room2.door.xmin + xShift;
rooms.room3.door.xmax = rooms.room2.door.xmax + xShift;
rooms.room3.door.ymin = rooms.room2.door.ymin + yShift;
rooms.room3.door.ymax = rooms.room2.door.ymax + yShift;
% map-matching will be deactivated automatically when pedestrian is headed 
% to the room 2.
rooms.room3.door.insideRegion = false;
for i = lastStride:strideNumber % strideNumber
        x = (mu(1,i) - nx) : 1 : (mu(1,i) + px);
        y = (mu(2,i) - ny) : 1 : (mu(2,i) + py);
        [X,Y] = meshgrid(x,y);
        f_likelihood = exp(-0.5*(((X-mu(1,i))./measSigmaX).^2 ...
            + ((Y-mu(2,i))./measSigmaY).^2));
%         fprintf('%i\n',i);
        deltaImuPosition = position_wrt_previous_two_strides...
            (mu(:,i-2),mu(:,i-1),mu(:,i)); % delta sensor likelihood
        % now we need to slice the f_staticFloor correctly
        imuPosition = sensor_likelihood_wrt_previous_two_estimates...
            (finalEstimate(:,i-2),finalEstimate(:,i-1),deltaImuPosition);
        f_staticFloor(:,:,i) = imcrop_taha_subpixel_accuracy...
            (choice,imuPosition,nx,px,ny,py);
        if (rooms.room3.door.insideRegion == true)
                f_staticFloor(:,:,i) = 255;
        end
        
        posterior = f_likelihood.*f_staticFloor(:,:,i);
        posterior(:,:,i) = posterior/max(max(posterior));
        maxValue = max(max(posterior(:,:,i)));
        [rowsOfMaxes,colsOfMaxes] = find(posterior(:,:,i) == maxValue);
        finalEstimate(:,i) = [mean(colsOfMaxes); mean(rowsOfMaxes)] - ...
                             [nx; ny] + imuPosition - [1;1];
                         
        % automatically checking if the user is headed to next room
        if (~rooms.room3.door.insideRegion && ...
                room_door_region_check(finalEstimate(:,i),...
                rooms.room3.door.xmin,rooms.room3.door.xmax,...
                rooms.room3.door.ymin,rooms.room3.door.ymax))
            rooms.room3.door.insideRegion = true;
            fprintf('Stride #%i is at room 3 entrance.\n',i);
            rooms.room3.doorRegionStride = i;
        end
        % if the user is at door 2 region, next stride will not affected by
        % map-matching, which means MHT has to start to detect the last
        % stride in the room
end
rooms.room3.firstStride = rooms.room3.doorRegionStride+1;
lastFigNumber = lastFigNumber + 1;
figure(lastFigNumber); clf; set(gcf,'position',[564 61 927 670]);
set(gcf,'name','room 2 trajectory corrected, map-matching reactivated, room 3 entering stride detected')
imshow(img); hold on;
plot(mu(1,1:end),mu(2,1:end),'b-x','linewidth',1.5); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,1:end),finalEstimate(2,1:end),'r-x','linewidth',1.5);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',2,'markerfacecolor','r','markeredgecolor','k','markersize',10);
plot(finalEstimate(1,rooms.room1.doorRegionStride),...
    finalEstimate(2,rooms.room1.doorRegionStride),'yo','linewidth',2,...
    'markeredgecolor','y','markersize',10);
plot(finalEstimate(1,rooms.room2.doorRegionStride),...
    finalEstimate(2,rooms.room2.doorRegionStride),'yo','linewidth',2,...
    'markeredgecolor','y','markersize',10);
plot(finalEstimate(1,rooms.room3.doorRegionStride),...
    finalEstimate(2,rooms.room3.doorRegionStride),'yo','linewidth',2,...
    'markeredgecolor','y','markersize',10);
plot([rooms.room1.door.xmin rooms.room1.door.xmax ...
    rooms.room1.door.xmax rooms.room1.door.xmin rooms.room1.door.xmin], ...
    [rooms.room1.door.ymin rooms.room1.door.ymin rooms.room1.door.ymax ...
    rooms.room1.door.ymax rooms.room1.door.ymin],'g-','linewidth',2);
plot([rooms.room2.door.xmin rooms.room2.door.xmax ...
    rooms.room2.door.xmax rooms.room2.door.xmin rooms.room2.door.xmin], ...
    [rooms.room2.door.ymin rooms.room2.door.ymin rooms.room2.door.ymax ...
    rooms.room2.door.ymax rooms.room2.door.ymin],'g-','linewidth',2);
plot([rooms.room3.door.xmin rooms.room3.door.xmax ...
    rooms.room3.door.xmax rooms.room3.door.xmin rooms.room3.door.xmin], ...
    [rooms.room3.door.ymin rooms.room3.door.ymin rooms.room3.door.ymax ...
    rooms.room3.door.ymax rooms.room3.door.ymin],'g-','linewidth',2);
hold off;
legend('error state KF with ZUPT','non-recursive Bayesian map-matching based MHT','start','stop');
set(legend,'fontsize',16,'interpreter','latex','location','northeast');
%% room 3 hypothesis illustrations
% this is just for the sake of illustration - it has no effect on the algorithm
lastFigNumber = lastFigNumber+1;
figure(lastFigNumber); set(gcf,'position',[564 61 927 670]); clf;
imshow(img); hold on;
plot(mu(1,1:end),mu(2,1:end),'b-x','linewidth',1.5); % this is IMU (EKF+DR+ZUPT) solution
%plot(muUS(1,1:end-18*upsamplingRate),muUS(2,1:end-18*upsamplingRate),'b-.','linewidth',3); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,1:end),finalEstimate(2,1:end),'r-x','linewidth',1.5);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',2,'markerfacecolor','r','markeredgecolor','k','markersize',16);
plot(finalEstimate(1,end),finalEstimate(2,end),'gs','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',16);
% max 64 - just for the sake of illustration. see the code below.
rooms.room3.numberOfHyp = 48; % 48
for q=1:rooms.room3.numberOfHyp % starts with strideNumber-350 and decreases 1 by 1
    plot(finalEstimate(1,rooms.room3.doorRegionStride+q),...
    finalEstimate(2,rooms.room3.doorRegionStride+q),'ko','linewidth',2,'markersize',10);
    fprintf('strideNumber - %i = stride #%i = hypothesis #%i\n',351-q,...
        rooms.room3.firstStride+q, q);
end
plot([rooms.room1.door.xmin rooms.room1.door.xmax ...
    rooms.room1.door.xmax rooms.room1.door.xmin rooms.room1.door.xmin], ...
    [rooms.room1.door.ymin rooms.room1.door.ymin rooms.room1.door.ymax ...
    rooms.room1.door.ymax rooms.room1.door.ymin],'g-','linewidth',2);
plot(finalEstimate(1,rooms.room1.doorRegionStride),...
    finalEstimate(2,rooms.room1.doorRegionStride),'yo','linewidth',2,...
    'markeredgecolor','y','markersize',10);
plot([rooms.room2.door.xmin rooms.room2.door.xmax ...
    rooms.room2.door.xmax rooms.room2.door.xmin rooms.room2.door.xmin], ...
    [rooms.room2.door.ymin rooms.room2.door.ymin rooms.room2.door.ymax ...
    rooms.room2.door.ymax rooms.room2.door.ymin],'g-','linewidth',2);
plot(finalEstimate(1,rooms.room2.doorRegionStride),...
    finalEstimate(2,rooms.room2.doorRegionStride),'yo','linewidth',2,...
    'markeredgecolor','y','markersize',10);
plot([rooms.room3.door.xmin rooms.room3.door.xmax ...
    rooms.room3.door.xmax rooms.room3.door.xmin rooms.room3.door.xmin], ...
    [rooms.room3.door.ymin rooms.room3.door.ymin rooms.room3.door.ymax ...
    rooms.room3.door.ymax rooms.room3.door.ymin],'g-','linewidth',2);
plot(finalEstimate(1,rooms.room3.doorRegionStride),...
    finalEstimate(2,rooms.room3.doorRegionStride),'yo','linewidth',2,...
    'markeredgecolor','y','markersize',10);
hold off;
%% Room 3 all hypothesis probability computations
% In order to determine which stride leaves the
% room, we assume that the user leaves the room and return the hallway in
% right angle that is 90 degrees and also + and - 5,10 degrees.
close all; clc;
rooms.room3.numberOfHyp = 48; % 48
hypRotAngle = zeros(1,rooms.room3.numberOfHyp);
hypRotAngle(1)=176; hypRotAngle(2)=130;hypRotAngle(3)=87;hypRotAngle(4)=77;
hypRotAngle(5)=83;hypRotAngle(6)=74;hypRotAngle(7)=123;hypRotAngle(8)=-130;
hypRotAngle(9)=-85;hypRotAngle(10)=-97;hypRotAngle(11)=-109;hypRotAngle(12)=-48;
hypRotAngle(13)=77;hypRotAngle(14)=82;hypRotAngle(15)=74;hypRotAngle(16)=58;
hypRotAngle(17)=81; hypRotAngle(18)=-159; hypRotAngle(19)=-108;
hypRotAngle(20)=-98; hypRotAngle(21)=-112; hypRotAngle(22)=-98;
hypRotAngle(23)=14; hypRotAngle(24)=72; hypRotAngle(25)=72;
hypRotAngle(26)=72; hypRotAngle(27)=53; hypRotAngle(28)=150;
hypRotAngle(29)=-123; hypRotAngle(30)=-95; hypRotAngle(31)=-113;
hypRotAngle(32)=-125; hypRotAngle(33)=-40; hypRotAngle(34)=63;
hypRotAngle(35)=66; hypRotAngle(36)=57; hypRotAngle(37)=73;
hypRotAngle(38)=-160; hypRotAngle(39)=-110; hypRotAngle(40)=-110;
hypRotAngle(41)=-131; hypRotAngle(42)=-127; hypRotAngle(43)=-70;
hypRotAngle(44)=-37; hypRotAngle(45)=-65; hypRotAngle(46)=-116;
hypRotAngle(47)=-117; hypRotAngle(48)=-121;
roomExitAngle = -5;
% -10 yields h1 w/ 8.28, -5 yields h44 w/ 9.74, 0 yields h44 w/ 7.86, 
% +5 yields h30 w/ 5.69, +10 yields h30 w/ 5.88
hypRotAngle = hypRotAngle + roomExitAngle; % -10 -5 hypRotAngle +5 +10 

% for each hypothesis, the rotation matrix is wrt to actual door location
hypRotMatrix = zeros(2,2,rooms.room3.numberOfHyp);
for i=1:rooms.room3.numberOfHyp
    hypRotMatrix(:,:,i) = [cosd(hypRotAngle(i)) -sind(hypRotAngle(i));
                           sind(hypRotAngle(i)) cosd(hypRotAngle(i))];
end

finalEstimateRotated = zeros(2,strideNumber,rooms.room3.numberOfHyp);
finalEstimateTemp = zeros(2,strideNumber);

for i = 1:strideNumber
    finalEstimateTemp(:,i) = finalEstimate(:,i) - finalEstimate(:,1);
end
for i=1:rooms.room3.numberOfHyp
    for idx = 1:strideNumber
        finalEstimateRotated(:,idx,i) = hypRotMatrix(:,:,i)*...
            [finalEstimateTemp(1,idx) finalEstimateTemp(2,idx)]';
    end
end
for j=1:rooms.room3.numberOfHyp
    for i=1:strideNumber
        finalEstimateRotated(:,i,j) = finalEstimateRotated(:,i,j) + ...
            finalEstimate(:,1);
    end
end

%door216Location = [760.4018;999.5193];
hypTranslation = zeros(2,rooms.room3.numberOfHyp);
finalEstimateRotatedTranslated=zeros(2,strideNumber,rooms.room3.numberOfHyp);
% following translation vectors are calculated according to room 1 door coordinates
% recall that we only need room door coordinates in non-recusive bayesian
% map-matching with MHT approach unlike PF that need room wall coordinates
% in addition to hallway wall coordinates
rooms.room3.door.x = 660; rooms.room3.door.y = 704;
hypTranslation(:,1) = [rooms.room3.door.x; rooms.room3.door.y] - ...
                       finalEstimateRotated(:,rooms.room3.firstStride,1);
% hypTranslation(:,1) = [-278;-93]; % room 1 door coordinates determine this
% hypTranslation(:,1) = [-265;-110]; % + 10 angle
% hypTranslation(:,1) = [-286;-67]; % - 10 angle
finalEstimateRotatedTranslated(:,:,1) = finalEstimateRotated(:,:,1) + hypTranslation(:,1);
% (strideNumber - rooms.room1.doorRegionStride - 1) = 350
% (strideNumber - rooms.room1.doorRegionStride - i) = 349, 348, ...
for i=2:rooms.room3.numberOfHyp
    hypTranslation(:,i) = finalEstimateRotated(:,strideNumber - ...
        (strideNumber - rooms.room3.doorRegionStride - i),i) - ...
        finalEstimateRotatedTranslated(:,strideNumber - ...
        (strideNumber - rooms.room3.doorRegionStride - 1),1);
    finalEstimateRotatedTranslated(:,:,i) = finalEstimateRotated(:,:,i)-...
        hypTranslation(:,i);
end

lastFigNumber = subsequent_strides_inside_room(img,maxColors,...
    finalEstimateRotatedTranslated,strideNumber,...
    rooms.room3.doorRegionStride,subsequentStrides,color,...
    rooms.room3.numberOfHyp,finalEstimateRotated,choice,lastFigNumber);
% SUBSEQUENT STRIDE PROBABILITIES - room 3
probStrides = zeros(rooms.room3.numberOfHyp,subsequentStrides);
for i=1:rooms.room3.numberOfHyp
    for j=1:subsequentStrides
        probStrides(i,j) = imcrop_taha_subpixel_accuracy(choice,...
            finalEstimateRotatedTranslated(:,strideNumber-(strideNumber - ...
            rooms.room3.doorRegionStride - i)+j,i),0,0,0,0);
        if (isnan(probStrides(i,j)))
            probStrides(i,j) = 0.004;
        end
    end
end

lastFigNumber = illustration_of_probabilities(subsequentStrides,...
    probStrides,lastFigNumber,color,rooms.room3.numberOfHyp);
% ============= determining which stride is the last one in the room ========
% total probabilities of all hypotheses
hypTotalProbs = sum(probStrides');
[maxValue,rooms.room3.lastHyp] = max(hypTotalProbs);
rooms.room3.lastStride = rooms.room3.doorRegionStride + rooms.room3.lastHyp;
rooms.room3.numberOfHypInsideRoom = rooms.room3.lastStride - ...
    rooms.room3.firstStride + 1;
rooms.room3.maxAngle = hypRotAngle(rooms.room3.numberOfHypInsideRoom);
rooms.room3.maxTranslation = hypTranslation(:,rooms.room3.numberOfHypInsideRoom);
lastFigNumber = lastFigNumber+1;
figure(lastFigNumber); clf; hold on;
plot(rooms.room3.lastHyp, hypTotalProbs(rooms.room3.lastHyp),...
    'ro','markersize',12,'linewidth',1.2);
plot(1:size(probStrides,1), hypTotalProbs,'linewidth',1.3);
xlabel('hypothesis','interpreter','latex','fontsize',15);
ylabel('total probability','interpreter','latex','fontsize',15);
grid on; axis([1 size(probStrides,1) 0 10]); hold off;
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
set(gca,'GridLineStyle','--');
legend(['$\displaystyle \sum_{i=1}^{10} h_{', num2str(rooms.room3.lastHyp), ...
    '}^{',num2str(roomExitAngle),'}(i) = ', num2str(maxValue),'$']);
set(legend,'fontsize',13,'interpreter','latex','location','north');
% SUMMARY of ROOM 1 visit
fprintf('The pedestrian stepped inside door 3 region at stride %i.\n',...
    rooms.room3.doorRegionStride);
fprintf('Map-matching will be deactivated for strides in the room.\n');
fprintf('First stride in room 3 is stride #%i that is hypothesis 1.\n',...
    rooms.room3.firstStride);
fprintf('Last stride in room 3 is stride #%i that is hypothesis #%i.\n',...
    rooms.room3.lastStride,rooms.room3.lastHyp);
fprintf('# of strides = # of hypotheses in room 3 is %i.\n',...
    rooms.room3.numberOfHypInsideRoom);
%% now last stride in room 3 is determined so firstly we can 
% correct room trajectory left behind in an intuitive manner, and secondly
% subsequent strides of last stride should be exposed to map-matching as
% they are located in the hallway
% ============= STEP 1 - ROOM TRAJECTORY CORRECTION ========================
% gradual correction for the trajectory in the room
% the hypothesis (stride) with the highest probability has the max rotation 
% and translation. the angle and the shift is gradually distributed to the 
% strides in the room. the very first stride in the room is exposed to least 
% rotation and shift.
% load continue_from_here_3.mat
lastFigNumber = lastFigNumber+1;
figure(lastFigNumber); set(gcf,'position',[564 61 927 670],...
    'name','Gradual correction of room 3 (room 221)');
imshow(img); hold on;
if (choice == 1 || choice == 2)
plot(mu(1,1:end),mu(2,1:end),'b-x','linewidth',1.5); % this is IMU (EKF+DR+ZUPT) solution
%plot(muUS(1,1:end-18*upsamplingRate),muUS(2,1:end-18*upsamplingRate),'b-.','linewidth',3); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,1:end),finalEstimate(2,1:end),'r-x','linewidth',1.5);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',2,'markerfacecolor','r','markeredgecolor','k','markersize',16);
plot(finalEstimate(1,end),finalEstimate(2,end),'gs','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',16);
end
firstStride = rooms.room3.firstStride;
lastStride = rooms.room3.lastStride; % 16
roomStrides = lastStride-firstStride;
translationReference = finalEstimateRotatedTranslated(:,lastStride,...
    rooms.room3.lastHyp);
for i = 0:roomStrides
    angleTemp = rooms.room3.maxAngle*(roomStrides-i)/roomStrides;
    rotation_matrixTemp = [cosd(angleTemp) -sind(angleTemp);
                           sind(angleTemp) cosd(angleTemp)];
    finalEstimateRotated(:,lastStride-i,rooms.room3.lastHyp) = ...
        rotation_matrixTemp*[finalEstimateTemp(1,lastStride-i) ...
                             finalEstimateTemp(2,lastStride-i)]';
    translationTemp = rooms.room3.maxTranslation*(roomStrides-i)/roomStrides;
    finalEstimateRotatedTranslated(:,lastStride-i,rooms.room3.lastHyp) = ...
        finalEstimateRotated(:,lastStride-i,rooms.room3.lastHyp) - ...
        translationTemp; % translation for hypothesis k-2
    if (i == 0)
        translationFix = translationReference - ...
            finalEstimateRotatedTranslated(:,lastStride-i,rooms.room3.lastHyp);
    end
    plot(finalEstimateRotatedTranslated(1,lastStride-i,...
        rooms.room3.lastHyp)+translationFix(1),...
        finalEstimateRotatedTranslated(2,lastStride-i,...
        rooms.room3.lastHyp)+translationFix(2),...
        'gx','markersize',12,'linewidth',1.8);
    finalEstimate(:,lastStride-i) = finalEstimateRotatedTranslated(:,...
        lastStride-i,rooms.room3.lastHyp)+translationFix;
    fprintf('%i ', lastStride-i);
end
legend('error state KF with ZUPT','non-recursive Bayesian filter','start','stop');
set(legend,'fontsize',16,'interpreter','latex','location','northwest');
hold off;
fprintf('\n');
%% Room 221 traversal is over. The pedestrian is walking to Room 233 now.
% ============= STEP 2 - REACTIVATE MAP-MATCHING ========================
% as we know the last stride in room 1, map-matching is activated for
% subsequent strides automatically until the pedestrian is headed to room 4
xShift = -385; yShift = -205;
rooms.room4.door.xmin = rooms.room3.door.xmin + xShift - 5;
rooms.room4.door.xmax = rooms.room3.door.xmax + xShift + 5;
rooms.room4.door.ymin = rooms.room3.door.ymin + yShift;
rooms.room4.door.ymax = rooms.room3.door.ymax + yShift;
% map-matching will be deactivated automatically when pedestrian is headed 
% to the room 2.
rooms.room4.door.insideRegion = false;
for i = lastStride:strideNumber % strideNumber
        x = (mu(1,i) - nx) : 1 : (mu(1,i) + px);
        y = (mu(2,i) - ny) : 1 : (mu(2,i) + py);
        [X,Y] = meshgrid(x,y);
        f_likelihood = exp(-0.5*(((X-mu(1,i))./measSigmaX).^2 ...
            + ((Y-mu(2,i))./measSigmaY).^2));
%         fprintf('%i\n',i);
        deltaImuPosition = position_wrt_previous_two_strides...
            (mu(:,i-2),mu(:,i-1),mu(:,i)); % delta sensor likelihood
        % now we need to slice the f_staticFloor correctly
        imuPosition = sensor_likelihood_wrt_previous_two_estimates...
            (finalEstimate(:,i-2),finalEstimate(:,i-1),deltaImuPosition);
        f_staticFloor(:,:,i) = imcrop_taha_subpixel_accuracy...
            (choice,imuPosition,nx,px,ny,py);
        if (rooms.room4.door.insideRegion == true)
                f_staticFloor(:,:,i) = 255;
        end
        
        posterior = f_likelihood.*f_staticFloor(:,:,i);
        posterior(:,:,i) = posterior/max(max(posterior));
        maxValue = max(max(posterior(:,:,i)));
        [rowsOfMaxes,colsOfMaxes] = find(posterior(:,:,i) == maxValue);
        finalEstimate(:,i) = [mean(colsOfMaxes); mean(rowsOfMaxes)] - ...
                             [nx; ny] + imuPosition - [1;1];
                         
        % automatically checking if the user is headed to next room
        if (~rooms.room4.door.insideRegion && ...
                room_door_region_check(finalEstimate(:,i),...
                rooms.room4.door.xmin,rooms.room4.door.xmax,...
                rooms.room4.door.ymin,rooms.room4.door.ymax))
            rooms.room4.door.insideRegion = true;
            fprintf('Stride #%i is at room 4 entrance.\n',i);
            rooms.room4.doorRegionStride = i;
        end
        % if the user is at door region, next stride will not affected by
        % map-matching, which means MHT has to start to detect the last
        % stride in the room
end
rooms.room4.firstStride = rooms.room4.doorRegionStride+1;
lastFigNumber = lastFigNumber + 1;
figure(lastFigNumber); clf; set(gcf,'position',[564 61 927 670]);
set(gcf,'name','room 2 trajectory corrected, map-matching reactivated, room 3 entering stride detected')
imshow(img); hold on;
plot(mu(1,1:end),mu(2,1:end),'b-x','linewidth',1.5); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,1:end),finalEstimate(2,1:end),'r-x','linewidth',1.5);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',2,'markerfacecolor','r','markeredgecolor','k','markersize',10);
plot(finalEstimate(1,rooms.room1.doorRegionStride),...
    finalEstimate(2,rooms.room1.doorRegionStride),'yo','linewidth',2,...
    'markeredgecolor','y','markersize',10);
plot(finalEstimate(1,rooms.room2.doorRegionStride),...
    finalEstimate(2,rooms.room2.doorRegionStride),'yo','linewidth',2,...
    'markeredgecolor','y','markersize',10);
plot(finalEstimate(1,rooms.room3.doorRegionStride),...
    finalEstimate(2,rooms.room3.doorRegionStride),'yo','linewidth',2,...
    'markeredgecolor','y','markersize',10);
plot(finalEstimate(1,rooms.room4.doorRegionStride),...
    finalEstimate(2,rooms.room4.doorRegionStride),'yo','linewidth',2,...
    'markeredgecolor','y','markersize',10);
plot([rooms.room1.door.xmin rooms.room1.door.xmax ...
    rooms.room1.door.xmax rooms.room1.door.xmin rooms.room1.door.xmin], ...
    [rooms.room1.door.ymin rooms.room1.door.ymin rooms.room1.door.ymax ...
    rooms.room1.door.ymax rooms.room1.door.ymin],'g-','linewidth',2);
plot([rooms.room2.door.xmin rooms.room2.door.xmax ...
    rooms.room2.door.xmax rooms.room2.door.xmin rooms.room2.door.xmin], ...
    [rooms.room2.door.ymin rooms.room2.door.ymin rooms.room2.door.ymax ...
    rooms.room2.door.ymax rooms.room2.door.ymin],'g-','linewidth',2);
plot([rooms.room3.door.xmin rooms.room3.door.xmax ...
    rooms.room3.door.xmax rooms.room3.door.xmin rooms.room3.door.xmin], ...
    [rooms.room3.door.ymin rooms.room3.door.ymin rooms.room3.door.ymax ...
    rooms.room3.door.ymax rooms.room3.door.ymin],'g-','linewidth',2);
plot([rooms.room4.door.xmin rooms.room4.door.xmax ...
    rooms.room4.door.xmax rooms.room4.door.xmin rooms.room4.door.xmin], ...
    [rooms.room4.door.ymin rooms.room4.door.ymin rooms.room4.door.ymax ...
    rooms.room4.door.ymax rooms.room4.door.ymin],'g-','linewidth',2);
hold off;
legend('error state KF with ZUPT','non-recursive Bayesian map-matching based MHT','start','stop');
set(legend,'fontsize',16,'interpreter','latex','location','northeast');
%% room 4 hypothesis illustrations
% this is just for the sake of illustration - it has no effect on the algorithm
lastFigNumber = lastFigNumber+1;
figure(lastFigNumber); set(gcf,'position',[564 61 927 670]); clf;
imshow(img); hold on;
plot(mu(1,1:end),mu(2,1:end),'b-x','linewidth',1.5); % this is IMU (EKF+DR+ZUPT) solution
%plot(muUS(1,1:end-18*upsamplingRate),muUS(2,1:end-18*upsamplingRate),'b-.','linewidth',3); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,1:end),finalEstimate(2,1:end),'r-x','linewidth',1.5);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',2,'markerfacecolor','r','markeredgecolor','k','markersize',16);
plot(finalEstimate(1,end),finalEstimate(2,end),'gs','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',16);
% max 64 - just for the sake of illustration. see the code below.
rooms.room4.numberOfHyp = 56;
for q=1:rooms.room4.numberOfHyp
    plot(finalEstimate(1,rooms.room4.doorRegionStride+q),...
    finalEstimate(2,rooms.room4.doorRegionStride+q),'ko','linewidth',2,'markersize',10);
    fprintf('strideNumber - %i = stride #%i = hypothesis #%i\n',351-q,...
        rooms.room4.firstStride+q, q);
end
plot([rooms.room1.door.xmin rooms.room1.door.xmax ...
    rooms.room1.door.xmax rooms.room1.door.xmin rooms.room1.door.xmin], ...
    [rooms.room1.door.ymin rooms.room1.door.ymin rooms.room1.door.ymax ...
    rooms.room1.door.ymax rooms.room1.door.ymin],'g-','linewidth',2);
plot(finalEstimate(1,rooms.room1.doorRegionStride),...
    finalEstimate(2,rooms.room1.doorRegionStride),'yo','linewidth',2,...
    'markeredgecolor','y','markersize',10);
plot([rooms.room2.door.xmin rooms.room2.door.xmax ...
    rooms.room2.door.xmax rooms.room2.door.xmin rooms.room2.door.xmin], ...
    [rooms.room2.door.ymin rooms.room2.door.ymin rooms.room2.door.ymax ...
    rooms.room2.door.ymax rooms.room2.door.ymin],'g-','linewidth',2);
plot(finalEstimate(1,rooms.room2.doorRegionStride),...
    finalEstimate(2,rooms.room2.doorRegionStride),'yo','linewidth',2,...
    'markeredgecolor','y','markersize',10);
plot([rooms.room3.door.xmin rooms.room3.door.xmax ...
    rooms.room3.door.xmax rooms.room3.door.xmin rooms.room3.door.xmin], ...
    [rooms.room3.door.ymin rooms.room3.door.ymin rooms.room3.door.ymax ...
    rooms.room3.door.ymax rooms.room3.door.ymin],'g-','linewidth',2);
plot(finalEstimate(1,rooms.room3.doorRegionStride),...
    finalEstimate(2,rooms.room3.doorRegionStride),'yo','linewidth',2,...
    'markeredgecolor','y','markersize',10);
plot([rooms.room4.door.xmin rooms.room4.door.xmax ...
    rooms.room4.door.xmax rooms.room4.door.xmin rooms.room4.door.xmin], ...
    [rooms.room4.door.ymin rooms.room4.door.ymin rooms.room4.door.ymax ...
    rooms.room4.door.ymax rooms.room4.door.ymin],'g-','linewidth',2);
plot(finalEstimate(1,rooms.room4.doorRegionStride),...
    finalEstimate(2,rooms.room4.doorRegionStride),'yo','linewidth',2,...
    'markeredgecolor','y','markersize',10);
hold off;
%% Room 4 all hypothesis probability computations
% In order to determine which stride leaves the
% room, we assume that the user leaves the room and return the hallway in
% right angle that is 90 degrees and also + and - 5,10 degrees.
rooms.room4.numberOfHyp = 54;
close all; clc;
subsequentStrides = 15;
hypRotAngle = zeros(1,rooms.room4.numberOfHyp);
hypRotAngle(1)=-175; hypRotAngle(2)=-170;hypRotAngle(3)=-179;hypRotAngle(4)=135;
hypRotAngle(5)=83;hypRotAngle(6)=17;hypRotAngle(7)=-25;hypRotAngle(8)=-65;
hypRotAngle(9)=-97;hypRotAngle(10)=-130;hypRotAngle(11)=178;hypRotAngle(12)=174;
hypRotAngle(13)=170;hypRotAngle(14)=166;hypRotAngle(15)=136;hypRotAngle(16)=125;
hypRotAngle(17)=150; hypRotAngle(18)=165; hypRotAngle(19)=173;
hypRotAngle(20)=-105; hypRotAngle(21)=-73; hypRotAngle(22)=-25;
hypRotAngle(23)=-15; hypRotAngle(24)=-58; hypRotAngle(25)=-122;
hypRotAngle(26)=-160; hypRotAngle(27)=175; hypRotAngle(28)=148;
hypRotAngle(29)=68; hypRotAngle(30)=25; hypRotAngle(31)=12;
hypRotAngle(32)=45; hypRotAngle(33)=89; hypRotAngle(34)=132;
hypRotAngle(35)=160; hypRotAngle(36)=171; hypRotAngle(37)=179;
hypRotAngle(38)=-142; hypRotAngle(39)=-122; hypRotAngle(40)=-80;
hypRotAngle(41)=-74; hypRotAngle(42)=-32; hypRotAngle(43)=-9;
hypRotAngle(44)=-9; hypRotAngle(45)=30; hypRotAngle(46)=30;
hypRotAngle(47)=1; hypRotAngle(48)=-22; hypRotAngle(49)=18;
hypRotAngle(50)=58; hypRotAngle(51)=20; hypRotAngle(52)=-45;
hypRotAngle(53)=-90; hypRotAngle(54)=-43; hypRotAngle(55)=2;
hypRotAngle(56)=68;
roomExitAngle = 20;
% -25 yields h45  w/ 7.15, -20 yields h55  w/ 8.55, -15 yields h45  w/ 7.59, 
% -10 yields h11  w/ 7.50, -5 yields h1  w/ 7.39, 0 yields h1  w/ 8.11,
% +5 yields h1  w/ 8.46, +10 yields h43  w/ 8.29, +15 yields h43  w/ 8.38,
% +20 yields h54  w/ 13.11, +25 yields h44  w/ 9.10
hypRotAngle = hypRotAngle + roomExitAngle; % -10 -5 hypRotAngle +5 +10 

% for each hypothesis, the rotation matrix is wrt to actual door location
hypRotMatrix = zeros(2,2,rooms.room4.numberOfHyp);
for i=1:rooms.room4.numberOfHyp
    hypRotMatrix(:,:,i) = [cosd(hypRotAngle(i)) -sind(hypRotAngle(i));
                           sind(hypRotAngle(i)) cosd(hypRotAngle(i))];
end

finalEstimateRotated = zeros(2,strideNumber,rooms.room4.numberOfHyp);
finalEstimateTemp = zeros(2,strideNumber);

for i = 1:strideNumber
    finalEstimateTemp(:,i) = finalEstimate(:,i) - finalEstimate(:,1);
end
for i=1:rooms.room4.numberOfHyp
    for idx = 1:strideNumber
        finalEstimateRotated(:,idx,i) = hypRotMatrix(:,:,i)*...
            [finalEstimateTemp(1,idx) finalEstimateTemp(2,idx)]';
    end
end
for j=1:rooms.room4.numberOfHyp
    for i=1:strideNumber
        finalEstimateRotated(:,i,j) = finalEstimateRotated(:,i,j) + ...
            finalEstimate(:,1);
    end
end

%door216Location = [760.4018;999.5193];
hypTranslation = zeros(2,rooms.room4.numberOfHyp);
finalEstimateRotatedTranslated=zeros(2,strideNumber,rooms.room4.numberOfHyp);
% following translation vectors are calculated according to room 1 door coordinates
% recall that we only need room door coordinates in non-recusive bayesian
% map-matching with MHT approach unlike PF that need room wall coordinates
% in addition to hallway wall coordinates
rooms.room4.door.x = 274; rooms.room4.door.y = 500;
hypTranslation(:,1) = [rooms.room4.door.x; rooms.room4.door.y] - ...
                       finalEstimateRotated(:,rooms.room4.firstStride,1);
% hypTranslation(:,1) = [-278;-93]; % room 1 door coordinates determine this
% hypTranslation(:,1) = [-265;-110]; % + 10 angle
% hypTranslation(:,1) = [-286;-67]; % - 10 angle
finalEstimateRotatedTranslated(:,:,1) = finalEstimateRotated(:,:,1) + hypTranslation(:,1);
% (strideNumber - rooms.room1.doorRegionStride - 1) = 350
% (strideNumber - rooms.room1.doorRegionStride - i) = 349, 348, ...
for i=2:rooms.room4.numberOfHyp
    hypTranslation(:,i) = finalEstimateRotated(:,strideNumber - ...
        (strideNumber - rooms.room4.doorRegionStride - i),i) - ...
        finalEstimateRotatedTranslated(:,strideNumber - ...
        (strideNumber - rooms.room4.doorRegionStride - 1),1);
    finalEstimateRotatedTranslated(:,:,i) = finalEstimateRotated(:,:,i)-...
        hypTranslation(:,i);
end

lastFigNumber = subsequent_strides_inside_room(img,maxColors,...
    finalEstimateRotatedTranslated,strideNumber,...
    rooms.room4.doorRegionStride,subsequentStrides,color,...
    rooms.room4.numberOfHyp,finalEstimateRotated,choice,lastFigNumber);
% SUBSEQUENT STRIDE PROBABILITIES - room 4
probStrides = zeros(rooms.room4.numberOfHyp,subsequentStrides);
for i=1:rooms.room4.numberOfHyp
    for j=1:subsequentStrides
        probStrides(i,j) = imcrop_taha_subpixel_accuracy(choice,...
            finalEstimateRotatedTranslated(:,strideNumber-(strideNumber - ...
            rooms.room4.doorRegionStride - i)+j,i),0,0,0,0);
        if (isnan(probStrides(i,j)))
            probStrides(i,j) = 0.004;
        end
    end
end

lastFigNumber = illustration_of_probabilities(subsequentStrides,...
    probStrides,lastFigNumber,color,rooms.room4.numberOfHyp);
% ============= determining which stride is the last one in the room ========
% total probabilities of all hypotheses
hypTotalProbs = sum(probStrides');
[maxValue,rooms.room4.lastHyp] = max(hypTotalProbs);
rooms.room4.lastStride = rooms.room4.doorRegionStride + rooms.room4.lastHyp;
rooms.room4.numberOfHypInsideRoom = rooms.room4.lastStride - ...
    rooms.room4.firstStride + 1;
rooms.room4.maxAngle = hypRotAngle(rooms.room4.numberOfHypInsideRoom);
rooms.room4.maxTranslation = hypTranslation(:,rooms.room4.numberOfHypInsideRoom);
lastFigNumber = lastFigNumber+1;
figure(lastFigNumber); clf; hold on;
plot(rooms.room4.lastHyp, hypTotalProbs(rooms.room4.lastHyp),...
    'ro','markersize',12,'linewidth',1.2);
plot(1:size(probStrides,1), hypTotalProbs,'linewidth',1.3);
xlabel('hypothesis','interpreter','latex','fontsize',15);
ylabel('total probability','interpreter','latex','fontsize',15);
grid on; axis([1 size(probStrides,1) 0 subsequentStrides]); hold off;
% set(gca,'xtick',[1:10],'xticklabel',{'1','2','3','4','5','6','7','8','9','10'},'fontsize',10,'TickLabelInterpreter', 'latex');
% set(gca,'ytick',[0:0.2:1],'yticklabel',{'','','','','',''},'fontsize',10,'TickLabelInterpreter', 'latex');
set(gca,'GridLineStyle','--');
legend(['$\displaystyle \sum_{i=1}^{10} h_{', num2str(rooms.room4.lastHyp), ...
    '}^{',num2str(roomExitAngle),'}(i) = ', num2str(maxValue),'$']);
set(legend,'fontsize',13,'interpreter','latex','location','north');
% SUMMARY of ROOM 1 visit
fprintf('The pedestrian stepped inside door 4 region at stride %i.\n',...
    rooms.room4.doorRegionStride);
fprintf('Map-matching will be deactivated for strides in the room.\n');
fprintf('First stride in room 4 is stride #%i that is hypothesis 1.\n',...
    rooms.room4.firstStride);
fprintf('Last stride in room 4 is stride #%i that is hypothesis #%i.\n',...
    rooms.room4.lastStride,rooms.room4.lastHyp);
fprintf('# of strides = # of hypotheses in room 3 is %i.\n',...
    rooms.room4.numberOfHypInsideRoom);
% now last stride in room 4 is determined so firstly we can 
% correct room trajectory left behind in an intuitive manner, and secondly
% subsequent strides of last stride should be exposed to map-matching as
% they are located in the hallway
%% ============= STEP 1 - ROOM TRAJECTORY CORRECTION ========================
% gradual correction for the trajectory in the room
% the hypothesis (stride) with the highest probability has the max rotation 
% and translation. the angle and the shift is gradually distributed to the 
% strides in the room. the very first stride in the room is exposed to least 
% rotation and shift.
lastFigNumber = lastFigNumber+1;
figure(lastFigNumber); set(gcf,'position',[564 61 927 670],...
    'name','Gradual correction of room 4 (room 233)');
imshow(img); hold on;
if (choice == 1 || choice == 2)
plot(mu(1,1:end),mu(2,1:end),'b-x','linewidth',1.5); % this is IMU (EKF+DR+ZUPT) solution
%plot(muUS(1,1:end-18*upsamplingRate),muUS(2,1:end-18*upsamplingRate),'b-.','linewidth',3); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,1:end),finalEstimate(2,1:end),'r-x','linewidth',1.5);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',2,'markerfacecolor','r','markeredgecolor','k','markersize',16);
plot(finalEstimate(1,end),finalEstimate(2,end),'gs','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',16);
end
firstStride = rooms.room4.firstStride;
lastStride = rooms.room4.lastStride; % 16
roomStrides = lastStride-firstStride;
translationReference = finalEstimateRotatedTranslated(:,lastStride,...
    rooms.room4.lastHyp);
for i = 0:roomStrides
    angleTemp = rooms.room4.maxAngle*(roomStrides-i)/roomStrides;
    rotation_matrixTemp = [cosd(angleTemp) -sind(angleTemp);
                           sind(angleTemp) cosd(angleTemp)];
    finalEstimateRotated(:,lastStride-i,rooms.room4.lastHyp) = ...
        rotation_matrixTemp*[finalEstimateTemp(1,lastStride-i) ...
                             finalEstimateTemp(2,lastStride-i)]';
    translationTemp = rooms.room4.maxTranslation*(roomStrides-i)/roomStrides;
    finalEstimateRotatedTranslated(:,lastStride-i,rooms.room4.lastHyp) = ...
        finalEstimateRotated(:,lastStride-i,rooms.room4.lastHyp) - ...
        translationTemp; % translation for hypothesis k-2
    if (i == 0)
        translationFix = translationReference - ...
            finalEstimateRotatedTranslated(:,lastStride-i,rooms.room4.lastHyp);
    end
    plot(finalEstimateRotatedTranslated(1,lastStride-i,...
        rooms.room4.lastHyp)+translationFix(1),...
        finalEstimateRotatedTranslated(2,lastStride-i,...
        rooms.room4.lastHyp)+translationFix(2),...
        'gx','markersize',12,'linewidth',1.8);
    finalEstimate(:,lastStride-i) = finalEstimateRotatedTranslated(:,...
        lastStride-i,rooms.room4.lastHyp)+translationFix;
    fprintf('%i ', lastStride-i);
end
legend('error state KF with ZUPT','non-recursive Bayesian filter','start','stop');
set(legend,'fontsize',16,'interpreter','latex','location','northwest');
hold off;
fprintf('\n');
%% Room 221 traversal is over. Remaining path is on the hallway.
% ============= STEP 2 - REACTIVATE MAP-MATCHING ========================
for i = lastStride:strideNumber % strideNumber
        x = (mu(1,i) - nx) : 1 : (mu(1,i) + px);
        y = (mu(2,i) - ny) : 1 : (mu(2,i) + py);
        [X,Y] = meshgrid(x,y);
        f_likelihood = exp(-0.5*(((X-mu(1,i))./measSigmaX).^2 ...
            + ((Y-mu(2,i))./measSigmaY).^2));
%         fprintf('%i\n',i);
        deltaImuPosition = position_wrt_previous_two_strides...
            (mu(:,i-2),mu(:,i-1),mu(:,i)); % delta sensor likelihood
        % now we need to slice the f_staticFloor correctly
        imuPosition = sensor_likelihood_wrt_previous_two_estimates...
            (finalEstimate(:,i-2),finalEstimate(:,i-1),deltaImuPosition);
        f_staticFloor(:,:,i) = imcrop_taha_subpixel_accuracy...
            (choice,imuPosition,nx,px,ny,py);
        
        posterior = f_likelihood.*f_staticFloor(:,:,i);
        posterior(:,:,i) = posterior/max(max(posterior));
        maxValue = max(max(posterior(:,:,i)));
        [rowsOfMaxes,colsOfMaxes] = find(posterior(:,:,i) == maxValue);
        finalEstimate(:,i) = [mean(colsOfMaxes); mean(rowsOfMaxes)] - ...
                             [nx; ny] + imuPosition - [1;1];
end

lastFigNumber = lastFigNumber + 1;
figure(lastFigNumber); clf; set(gcf,'position',[564 61 927 670]);
set(gcf,'name','room 4 trajectory corrected, map-matching reactivated')
imshow(img); hold on;
plot(mu(1,1:end),mu(2,1:end),'b-x','linewidth',1.5); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,1:end),finalEstimate(2,1:end),'r-x','linewidth',1.5);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',1.5,...
    'markerfacecolor','r','markeredgecolor','k','markersize',10);
plot(finalEstimate(1,end),finalEstimate(2,end),'gs','linewidth',1.5,...
    'markerfacecolor','g','markeredgecolor','k','markersize',10); % this is IMU (EKF+DR+ZUPT) solution
hold off;
legend('error state KF with ZUPT','non-recursive Bayesian map-matching based MHT','start','stop');
set(legend,'fontsize',16,'interpreter','latex','location','northeast');
