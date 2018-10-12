function vel = ZUPT(acc, stanceMedian, time)
n = length(acc);
vel = zeros(n,1);
moveStarted = false;
for i=2:n
    if ( (stanceMedian(i-1) == 1 && stanceMedian(i) == 0) || (i == n) )
        if (~moveStarted)
            iSwingStart = i;
        else if (moveStarted)
                iStanceEnd = i-1; % now it's time to apply ZUPT
                if (i==n)
                    iStanceEnd = i;
                end % now it's time to apply ZUPT
                %----------------- ZUPT -----------------------
                mu = mean(vel(iStanceStart:iStanceEnd)); % mean velocity in stance phase
                vel(iStanceStart:iStanceEnd) = vel(iStanceStart:iStanceEnd) - mu;
                vel(iSwingStart:iSwingEnd) = vel(iSwingStart:iSwingEnd) ...
                    - (mu / (time(iSwingEnd) - time(iSwingStart))) ...
                    * (time(iSwingStart:iSwingEnd) - time(iSwingStart));
                %---------------end of ZUPT -------------------
                iSwingStart = i;
            end
        end
        moveStarted = true;
    else if (stanceMedian(i-1) == 0 && stanceMedian(i) == 1)
            iSwingEnd = i-1;
            iStanceStart = i;
        end
    end
    if (moveStarted)
        vel(i) = vel(i-1) + acc(i)*(time(i)-time(i-1));
    end
end