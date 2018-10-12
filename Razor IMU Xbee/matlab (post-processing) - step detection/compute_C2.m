function C2 = compute_C2(localVariance, T)

n = length(localVariance);
C2 = zeros(n,1);
for i=1:n
    if ( sqrt(localVariance(i)) < T )
        C2(i) = 1;
    end
end
fprintf('C_2 is the condition 2 obtained by thresholding ');
fprintf('standart deviation of acceleration magnitude.\n');
end