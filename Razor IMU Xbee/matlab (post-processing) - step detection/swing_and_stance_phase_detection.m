function [B1, B2] = swing_and_stance_phase_detection(localVariance, T1, T2)
n = length(localVariance);
B1 = zeros(n,1); B2 = zeros(n,1);
for i=1:n
    if (sqrt(localVariance(i)) > T1)
        B1(i) = T1;
    end
    if (sqrt(localVariance(i)) < T2)
        B2(i) = T2;
    end
end
end