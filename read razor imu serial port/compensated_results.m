clear all;
close all;
clc;

accRange = 255;
gToMoverSsquare = 9.806;

experiment = cell(1,7);
experiment{1} = 'sensor_data_SEL.txt';
experiment{2} = 'area1.txt'; experiment{3} = 'area2.txt'; experiment{4} = 'road.txt';
experiment{5} = 'around_scott_lab.txt'; experiment{6} = 'ives_drive.txt';
experiment{7} = 'pavement_portland_st.txt';
k = 1; % select which experiment
data = load(experiment{k});
%data = load('sensor_data.txt');
timeStamps = data(:,7);
compAccX = -(data(:,8) / accRange);
compAccY = -(data(:,9) / accRange);
compAccZ = -(data(:,10) / accRange);
gyroX = data(:,11); gyroY = data(:,12); gyroZ = data(:,13);
magX = data(:,14); magY = data(:,15); magZ = data(:,16);

time = zeros(length(timeStamps),1);
time(1) = 0;
for i=2:length(time)
    time(i) = time(i-1) + timeStamps(i);
end

realAccX = compAccX * gToMoverSsquare;
realAccY = compAccY * gToMoverSsquare;

velX = taha_integrate(realAccX, timeStamps, 0);
velY = taha_integrate(realAccY, timeStamps, 0);

posX = taha_integrate(velX, timeStamps, 0);
posY = taha_integrate(velY, timeStamps, 0);
posLine = sqrt(posX.^2 + posY.^2);

distance = zeros(size(posX)); % this is what Dr Yilmaz wants to see
for i=1:length(distance)
    if (i == 1)
        distance(i) = sqrt(posX(i)^2+posY(i)^2);
    else
        distance(i) = sqrt( (posX(i) - posX(i-1))^2 + (posY(i) - posY(i-1))^2 );
    end
end
cumDistance = zeros(size(distance));
for i=1:length(distance)
    cumDistance(i) = sum(distance(1:i));
end

if (k == 1)
    length1 = 48; length2 = 37;
    fprintf('Experiment 1: Region next to SEL\n');
    fprintf('Distance in real world is %d.\n', 2*(length1+length2));
    
else if (k == 2)
    length1 = 48; length2 = 37;
    fprintf('Experiment 2: Field 1 at the park on Olentangy River Rd\n');
    fprintf('Distance in real world is %d.\n', 2*(length1+length2));
else if (k == 3)
    length1 = 48; length2 = 37;
    fprintf('Experiment 3: Road\n');
    fprintf('Distance in real world is %d.\n', 2*(length1+length2));
else if (k==4)
           
else if (k==5)
    l1 = 106.86;
    l2 = 139.06 - l1;
    l3 = 245.86 - l1 - l2;
    l4 = 276.03 - l1 - l2 - l3;
    fprintf('Experiment 5: Around Scott Lab.\n')
    fprintf('Distance in real world is %d.\n', 276.03);
else if (k==6)
    fprintf('Experiment 6: Ives Drive straight line\n')
    fprintf('Distance in real world is %d.\n', 100.11);
else if (k==7)
    fprintf('Experiment 7: Portland St pavement straight line\n')
    fprintf('Distance in real world is %d.\n', 258.43);        
    end
    end
    end
    end
    end
    end
end
fprintf('Computed cumulative distance is %d.\n', cumDistance(end));

yaw = data(:,3);

figure(1);
set(figure(1),'position',[15 390 432 420]);
subplot(311);
plot(time, compAccX, 'k-');
grid on; title('acc x (g)');
subplot(312);
plot(time, compAccY, 'k-');
grid on; title('acc y (g)');
subplot(313);
plot(time, compAccZ, 'k-');
grid on; title('acc z (g)');
xlabel('Time (s)');

figure(2);
set(figure(2),'position',[475 390 432 420]);
subplot(311);
plot(time, velX, 'b-');
grid on; title('velocity x (m/s)');
subplot(312);
plot(time, velY, 'b-');
grid on; title('velocity y (m/s)');
xlabel('Time (s)');

figure(3);
set(figure(3),'position',[925 390 432 420]);
subplot(311);
plot(time, posX, 'r-');
grid on; title('position x (m)');
subplot(312);
plot(time, posY, 'r-');
grid on; title('position y (m)');
xlabel('Time (s)');
subplot(313);
plot(time, posLine, 'r-');
grid on; title('position sqrt(x^2+y^2)');
xlabel('Time (s)');

figure(4);
set(gcf,'position',[378 85 493 323]);
plot(time, yaw, 'k-');
xlabel('Time(s)');
ylabel('yaw angle');
grid on;
axis([0 time(end) -180 180]);

figure(5);
set(gcf,'position',[880 85 700 323]);
subplot(121);
plot(posX, posY, 'k-');
xlabel('x');
ylabel('y');
grid on;
set(gca,'position',[0.0700    0.1352    0.4157    0.7898]);
subplot(122);
plot(time, cumDistance, 'k-');
grid on;
xlabel('Time (s)');
title('Cumulative distance');
set(gca,'position',[0.5357    0.1352    0.4457    0.7898]);

figure(6);
subplot(321);
plot(time,gyroX,'r-');
xlabel('Time(s)');
ylabel('gyroX');
subplot(323);
plot(time,gyroY,'r-');
xlabel('Time(s)');
ylabel('gyroY');
subplot(325);
plot(time,gyroZ,'r-');
xlabel('Time(s)');
ylabel('gyroZ');
subplot(322);
plot(time,magX,'r-');
xlabel('Time(s)');
ylabel('magX');
subplot(324);
plot(time,magY,'r-');
xlabel('Time(s)');
ylabel('magY');
subplot(326);
plot(time,magZ,'r-');
xlabel('Time(s)');
ylabel('magZ');