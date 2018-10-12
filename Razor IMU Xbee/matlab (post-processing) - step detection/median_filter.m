function stanceMedian = median_filter(stance, w)

n = length(stance);
k = (w-1) / 2;
stanceMedian = zeros(n,1);
for i=1:n
    nOf1s = 0; nOf0s = 0;
    s = 0; % samples in the temporal window count
    for j=-k:1:k
        if ( (i+j) > 0 && (i+j) < n+1 ) % temporal window doesn't fit at the beginning and end
            if (stance(i+j) == 0)
                nOf0s = nOf0s + 1;
            else if (stance(i+j) == 1)
                    nOf1s = nOf1s + 1;
                end
            end
            s = s+1;
        end
        if (mod(s,2) == 0) % at the beginning and end number of samples in the temporal window can be even; those parts correspond to stance motion so assign them 1s automatically
            stanceMedian(i) = 1;
        else
            if (nOf0s < nOf1s)
                stanceMedian(i) = 1;
            else
                stanceMedian(i) = 0;
            end
        end
    end
end
fprintf('Run a median filter for %d samples to obtain stance and swing phases.\n', w);
end