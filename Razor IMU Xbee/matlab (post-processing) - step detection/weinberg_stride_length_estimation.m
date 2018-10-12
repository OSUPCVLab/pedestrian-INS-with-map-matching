function estimatedStrideLengths = weinberg_stride_length_estimation(accMagFilt, steps, K, w, time)

n = length(accMagFilt);
k = (w-1) / 2;
numberOfSteps = length(steps);
estimatedStrideLengths = zeros(numberOfSteps,1);
stepCount = 0;
for i=steps
    s = 0; % samples in the temporal window count
    for j=-k:1:k % scan temporal window
        if ( (i+j) > 0 && (i+j) < n+1 ) % temporal window doesn't fit to the beginning and end
            if (s == 0)
                minElement = accMagFilt(i+j);
                maxElement = accMagFilt(i+j);
            else
                if (accMagFilt(i+j) < minElement)
                    minElement = accMagFilt(i+j);
                end
                if (accMagFilt(i+j) > maxElement)
                    maxElement = accMagFilt(i+j);
                end
            end
            s = s+1;
        end
    end
    stepCount = stepCount + 1;
    estimatedStrideLengths(stepCount) = K * (maxElement - minElement) ^ (0.25);
end

if (stepCount == numberOfSteps)
    fprintf('Weinberg stride estimation complete successfully.\n');
else
    fprintf('There is a problem in Weinberg stride estimation.\n');
end

for i=1:numberOfSteps
    fprintf('Step %i that occurs at sample %i (t=%.3f) is estimated to be %.3fm\n',...
        i, steps(i), time(steps(i)), estimatedStrideLengths(i));
end

fprintf('Total traveled distance in %d steps is %.3f\n', ...
    numberOfSteps, sum(estimatedStrideLengths));
fprintf('Average step length is %.3f\n', mean(estimatedStrideLengths));
fprintf('------------- end of Weinberg stride estimation algorithm -----------\n');

end