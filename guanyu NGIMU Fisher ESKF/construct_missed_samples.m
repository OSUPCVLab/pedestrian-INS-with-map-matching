function [xReconstructed, time] = construct_missed_samples(xCaptured, G_Dt, packetCount)
n1 = length(G_Dt); % number of Xbee captured samples
n2 = number_of_missed_samples(packetCount);
if (n2 == 0)
    fprintf('There is no sample missed in Xbee wireless transfer.\n');
else
    fprintf('Number of samples missed is %d\n', n2);
%     fprintf('Missed sample indexes are:\n');
%     for i=1:n2
%         fprintf('%d\n', missedIndexes(i));
%     end
end
n = n1 + n2; % number of Razor IMU captured samples
xReconstructed = zeros(n,1); xReconstructed(1) = xCaptured(1);
G_DtReconstructed = zeros(n,1); G_DtReconstructed(1) = G_Dt(1);
G_DtMissed = 0.02; % you can set 0.021 too.
j = 0; % index used to reconstruct interpolated sequence
missedSamplesIndexes = zeros(n2,1);
for i=2:n1
    diff = packetCount(i) - packetCount(i-1);
    if (diff == 2)
        j = j + 1; missedSamplesIndexes(j) = i;
        xReconstructed(i+j) = xCaptured(i);
        xReconstructed(i+j-1) = (xCaptured(i)+xCaptured(i-1))/2.0; % interpolation
        G_DtReconstructed(i+j) = G_Dt(i);
        G_DtReconstructed(i+j-1) = G_DtMissed;
    else
        xReconstructed(i+j) = xCaptured(i);
        G_DtReconstructed(i+j) = G_Dt(i);
    end
end
fprintf('Missed samples indexes are:\n');
for i=1:n2
    if (i ~= n2)
        fprintf('%d, ', missedSamplesIndexes(i));
    else
        fprintf('%d\n', missedSamplesIndexes(i));
    end
end
time = construct_time_axis(G_DtReconstructed);