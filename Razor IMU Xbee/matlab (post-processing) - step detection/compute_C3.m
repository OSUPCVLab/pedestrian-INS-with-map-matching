function C3 = compute_C3(gyroMag, T)

n = length(gyroMag);
C3 = zeros(n,1);
for i=1:n
    if ( gyroMag(i) < T )
        C3(i) = 1;
    end
end
fprintf('C_3 is the condition 3 obtained by thresholding gyro magnitude.\n');
end