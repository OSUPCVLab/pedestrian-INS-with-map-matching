function C1 = compute_C1(x, T1, T2)

n = length(x);
C1 = zeros(n,1);
for i=1:n
    if ( (x(i) > T1) && (x(i) < T2) )
        C1(i) = 1;
    end
end
fprintf('C_1 is the condition 1 obtained by double thresholding acceleration magnitude.\n');
end