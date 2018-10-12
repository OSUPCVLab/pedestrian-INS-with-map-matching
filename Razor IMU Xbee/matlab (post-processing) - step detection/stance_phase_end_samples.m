function stanceEnd = stance_phase_end_samples(stanceMedian)
n = length(stanceMedian);
j = 1;
for i=2:n
    if ( stanceMedian(i-1)==1 && stanceMedian(i)==0 )
        stanceEnd(j) = i;
        j = j+1;
    end
end