% modified from Fischer's tutorial - NGIMU

clear all;
close all;
% Read data from csv files saved by NGIMU data capture software.
% Data should include timestamps (seconds), 3 axis accelerations (m/s^2), 
% 3 axis gyroscopic rates of turn (rad/s). Notice that gyro is in degree/s
% and acc data is in terms of g while time is in seconds in NGIMU data.

choice = input('Enter 1 for Bolz Hall, 2 for Caldwell, 3 for around PCV: ');
if (choice == 1)
    gyroDataForBiasEstimationName = '../bolz_hall_gyro_data_for_bias_estimation_NGIMU_2.csv';
else if (choice == 2)
    gyroDataForBiasEstimationName = '../caldwell_lab_gyro_data_for_bias_estimation_wired_57600_multiple.txt';
    else if (choice == 3)
            gyroDataForBiasEstimationName = '../around pcv lab multiple times/around_pcv_gyro_data_for_bias_estimation_wired_57600_multiple.txt';
    else
        fprintf('Invalid input!\n');
        pause();
    end
    end
end

dataTaha = csvread(gyroDataForBiasEstimationName,1,0);
gyroData = deg2rad(dataTaha(:,2:4));
% compute gyro bias from static shoe data
gyro_bias = compute_gyro_bias(gyroData(:,1), gyroData(:,2), gyroData(:,3));
clear dataTaha gyroData

if (choice == 1)
    pedestrianTraverseName = '../bolz_hall_NGIMU_2.csv';
    else if (choice == 2)
        pedestrianTraverseName = '../caldwell_lab_wired_57600_multiple.txt';
        %pedestrianTraverseName = '../caldwell_lab_wired_57600.txt';
        else if (choice == 3)
                pedestrianTraverseName = '../around pcv lab multiple times/around_pcv_wired_57600_multiple.txt';
    else
        fprintf('Invalid input!\n');
        pause();
    end
        end
end

dataTaha = csvread(pedestrianTraverseName,1,0);
timestamp = dataTaha(:,1)';
gyro_s = deg2rad(dataTaha(:,2:4)'); % gyro data in sensor frame
g = 9.8; % 1g = 9.8m/s^2
acc_s = dataTaha(:,5:7)' .* g; % acc data in sensor frame
% NGIMU sensor coordinate frame rotation to Razor IMU - inverse y axis
%acc_s(2,:) = -acc_s(2,:);
data_size = length(gyro_s);

clear accX accY accZ gyroX gyroY gyroZ dataTaha
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
    consequtiveZUPTsamples = 30;
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
angle = 0;
sf = 1; % scale factor
if (choice == 1)
    sf = 1.1; % 1.09 for bolz 1.11 it was 1.15 for _2 1.16 for _4 1.06 for _5
    angle = 178; % 187 or 182.
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
    %pos_r(2,:) = -pos_r(2,:);
else if (choice == 2)
        pos_r(2,:) = -pos_r(2,:);
    else if (choice == 3)
            pos_r(2,:) = -pos_r(2,:);
        end
    end
end

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
startPixelCoordinates{1} = [1364 + 1; 738 + 1];
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

figure(13); set(figure(13),'position',[481 14 1114 799]);
set(gcf,'position',[564 61 927 670]);
imshow(img);

%% import static floor plan pdfs from previous studies
load('static_floor_pdfs.mat');
bolzPDF = bolzPDF + 1; caldwellPDF = caldwellPDF + 1;

global staticFloorPDF;
staticFloorPDF = cell(1,nOfBuildings);
staticFloorPDF{1} = double(bolzPDF)./255; staticFloorPDF{2} = double(caldwellPDF)./255;
staticFloorPDF{3} = staticFloorPDF{1}(1:780,1:465);
delete bolzPDF caldwellPDF

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
for i=1:strideNumber % strideNumber
    if (i < till) % 3 for constant velocity MM, 4 for constant acceleration MM
        % nx = 30; px = 30; ny = 30; py = 30;
        x = (mu(1,i) - nx) : 1 : (mu(1,i) + px);
        y = (mu(2,i) - ny) : 1 : (mu(2,i) + py);
        [X,Y] = meshgrid(x,y);
        f_likelihood = exp(-0.5*(((X-mu(1,i))./measSigmaX).^2 + ((Y-mu(2,i))./measSigmaY).^2));
        % i need to write my own subpixel crop function
        f_staticFloor(:,:,i) = imcrop_taha_subpixel_accuracy(choice,mu(:,i),nx,px,ny,py);
        posterior = f_likelihood.*f_staticFloor(:,:,i);
        posterior(:,:,i) = posterior/max(max(posterior));
        maxValue = max(max(posterior(:,:,i)));
        [rowsOfMaxes,colsOfMaxes] = find(posterior(:,:,i) == maxValue);
        finalEstimate(:,i) = [mean(colsOfMaxes); mean(rowsOfMaxes)] - [nx; ny] + mu(:,i);
    else
        % apply MM first, and then compute tempPost dist with theoretical_multiplication_of_two_gaussians() 
        % fcn. Finally apply static floor plan pdf to find resulting posterior distribution
        v = finalEstimate(:,i-1)-finalEstimate(:,i-2);
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
figure(5); set(figure(5),'position',[481 14 1114 799]);
set(gcf,'position',[564 61 927 670]);
imshow(img); hold on;
% upsamplingRate = 1; muUS = zeros(size(mu,1),upsamplingRate*size(mu,2));
% muUS(1,:) = interp(mu(1,:),upsamplingRate);
% muUS(2,:) = interp(mu(2,:),upsamplingRate);
if (choice == 1 || choice == 2)
plot(mu(1,1:end),mu(2,1:end),'b-.','linewidth',3); % this is IMU (EKF+DR+ZUPT) solution
%plot(muUS(1,1:end-18*upsamplingRate),muUS(2,1:end-18*upsamplingRate),'b-.','linewidth',3); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,1:end),finalEstimate(2,1:end),'r-','linewidth',3);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',3,'markerfacecolor','r','markeredgecolor','k','markersize',10);
plot(finalEstimate(1,end),finalEstimate(2,end),'gs','linewidth',3,'markerfacecolor','g','markeredgecolor','k','markersize',10);
else (choice == 3)
plot(mu(1,1:end-30),mu(2,1:end-30), 'b-.','linewidth',2); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,:),finalEstimate(2,:), 'r-','linewidth',2);
plot(finalEstimate(1,1),finalEstimate(2,1),'ro','linewidth',2,'markerfacecolor','r','markeredgecolor','k','markersize',10);
plot(finalEstimate(1,end-30),finalEstimate(2,end-30), 'gs','linewidth',2,'markerfacecolor','g','markeredgecolor','k','markersize',10);
end
hold off;
legend('INS with ZUPT aid','non-recursive estimation','start','stop');
set(legend,'fontsize',16,'interpreter','latex','location','northeast');

%%
figure(6); set(gcf,'position',[371 151 535 422]);
imshow(staticFloorPDF{choice},'colormap',jet(255));
colormap jet; set(gca, 'YDir', 'reverse'); hold on;
plot(mu(1,:),mu(2,:), 'w-','linewidth',2); % this is IMU (EKF+DR+ZUPT) solution
plot(finalEstimate(1,:),finalEstimate(2,:), 'k-','linewidth',2);
%plot(finalEstimate(1,till),finalEstimate(2,till), 'ko-','linewidth',2);
hold off;

%% compute angles
angles = zeros(1,strideNumber-1);
for i=2:strideNumber
    angles(i-1) = rad2deg(atan((pos_r(2,strideIndexes(i))-pos_r(2,strideIndexes(i-1)))...
        / (pos_r(1,strideIndexes(i)) - pos_r(1,strideIndexes(i-1)))));
end
figure(7);
plot(2:strideNumber,angles,'b.-'); grid on;