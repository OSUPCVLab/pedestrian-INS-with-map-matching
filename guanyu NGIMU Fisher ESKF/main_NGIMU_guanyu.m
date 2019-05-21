% modified from Fischer's tutorial - NGIMU

clear all; close all; clc;
% Read data from csv files saved by NGIMU data capture software.
% Data should include timestamps (seconds), 3 axis accelerations (m/s^2), 
% 3 axis gyroscopic rates of turn (rad/s). Notice that gyro is in degree/s
% and acc data is in terms of g while time is in seconds in NGIMU data.

fprintf('There are 4 experiments. Enter 1 or 2 or 3 or 4 to run one of them.\n');
fprintf('e.g., Enter 1 to run experiment 1.\n')
e = input('Which experiment would you like to run: ');
while (e ~= 1 && e ~= 2 && e ~= 3 && e ~= 4)
    fprintf('There is no experiment %i. Try again by entering [1-2-3-4]\n',e);
    e = input('Which experiment would you like to run: ');
end
gyroDataForBiasEstimationName = 'experiment_data_NGIMU/bolz_hall_gyro_data_for_bias_estimation_NGIMU_%i.csv';
gyroDataForBiasEstimationName = sprintf(gyroDataForBiasEstimationName,e);
pedestrianTraverseName = 'experiment_data_NGIMU/bolz_hall_NGIMU_%i.csv';
pedestrianTraverseName = sprintf(pedestrianTraverseName,e);
dataTaha = csvread(gyroDataForBiasEstimationName,1,0);
gyroData = deg2rad(dataTaha(:,2:4));
% compute gyro bias from static shoe data
gyro_bias = compute_gyro_bias(gyroData(:,1), gyroData(:,2), gyroData(:,3));
clear dataTaha gyroData
dataTaha = csvread(pedestrianTraverseName,1,0);
timestamp = dataTaha(:,1)';
gyro_s = deg2rad(dataTaha(:,2:4)'); % gyro data in sensor frame
g = 9.8; % 1g = 9.8m/s^2
acc_s = dataTaha(:,5:7)' .* g; % acc data in sensor frame
% NGIMU sensor coordinate frame rotation to Razor IMU - inverse y axis
%acc_s(2,:) = -acc_s(2,:);
data_size = length(gyro_s);

clear accX accY accZ gyroX gyroY gyroZ dataTaha
%% FISCHER related code start point: Initialise parameters.
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
consequtiveZUPTsamples = 30; % a parameter to determine stride sample, if you set it to high values you might miss the stride
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
%=============== END of pedestrian foot-mounted INS with FISCHER'S ESKF ===========
%% Rotate position estimates and plot.
angle = 0;
sf = 1; % scale factor
if (e == 1)
        sf = 1.1; angle = 178;
else if (e == 2)
        sf = 1.1; angle = 178;
    else if (e == 3)
            sf = 1.11; angle = 175;
        else if (e == 4)
                sf = 1.06; angle = 183;
            end
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
% if (e == 1)
%     %pos_r(2,:) = -pos_r(2,:);
% else if (e == 2)
%         pos_r(2,:) = -pos_r(2,:);
%     else if (e == 3)
%             pos_r(2,:) = -pos_r(2,:);
%         end
%     end
% end

%% results
distance = total_distance_calculation(pos_r(1,:), pos_r(2,:));
fprintf('Total traveled distance is %.3f meters.\n', sum(distance));

figure(1); set(figure(1),'position',[6 212 560 420]);
box on;
plot(pos_r(1,:),pos_r(2,:),'r','LineWidth',2);
hold on;
plot(pos_r(1,strideIndexes), pos_r(2,strideIndexes), 'bx','linewidth',2);
start = plot(pos_r(1,1),pos_r(2,1),'Marker','^','LineWidth',2,'LineStyle','none');
stop = plot(pos_r(1,end),pos_r(2,end),'Marker','o','LineWidth',2,'LineStyle','none');
xlabel('x (m)'); ylabel('y (m)');
title('Estimated 2D path'); legend([start;stop],'Start','End');
axis equal; grid; hold off;
% illustrating floor plan plots after this point
nOfBuildings = 1; % bolz and caldwell
buildingMaps = 'bolz_hall_2nd_floor_rotated.jpg';
img = rgb2gray(imread(buildingMaps));
height = size(img,1); width = size(img,2);
startPixelCoordinates = [1364 + 1; 738 + 1];
meter2pixelConstant = 200 / 9.7536; % unit conversion constant for bolz hall traverse

temp1 = repmat(startPixelCoordinates,1,data_size);
temp2 = meter2pixelConstant.*pos_r;
temp3 = meter2pixelConstant.*pos_r;
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
