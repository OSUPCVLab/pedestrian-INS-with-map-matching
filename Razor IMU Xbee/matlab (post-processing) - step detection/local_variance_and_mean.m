function [localVariance, localMean] = local_variance_and_mean(accMag, w)
n = length(accMag);
k = (w-1) / 2;
localVariance = zeros(n,1);
localMean = zeros(n,1);
for i=1:n
    sum = 0;
    s = 0; % samples in the temporal window count
    for j=-k:1:k
        if ( (i+j) > 0 && (i+j) < n+1 ) % temporal window doesn't fit at the beginning and end
            sum = sum + accMag(i+j);
            s = s+1;
        end
    end
    localMean(i) = sum / s;
    sum = 0;
    for j=-k:1:k
        if ( (i+j) > 0 && (i+j) < n+1 ) % temporal window doesn't fit at the beginning and end
            sum = sum + (accMag(i+j)-localMean(i))^2;
        end
    end
    localVariance(i) = sum / w;
end