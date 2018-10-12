function steps = jimenez_2009_step_detection(B1,B2,w,T2)
n = length(B1);
k = (w-1)/2;
steps = [];
a = 0;
for i=2:n
    stancePhaseFlag = false;
    if ((B1(i) < B1(i-1)))
        for j=1:k
            if (i+j <= n)
                if (B2(i+j) == T2)
                    stancePhaseFlag = true;
                    break;
                end
            end
        end
        if (stancePhaseFlag)
            a = a + 1;
            steps(a) = i;
        end
    end
end
fprintf('There are %d steps detected.\n', a);