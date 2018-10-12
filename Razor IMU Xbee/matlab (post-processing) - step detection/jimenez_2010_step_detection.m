function steps = jimenez_2010_step_detection(stanceMedian)
n = length(stanceMedian);
steps = [];
a = 0;
for i=2:n
    if (stanceMedian(i-1) == 0 && stanceMedian(i) == 1)
        a = a + 1;
        steps(a) = i;
    end
end
fprintf('There are %d steps detected.\n', a);
end