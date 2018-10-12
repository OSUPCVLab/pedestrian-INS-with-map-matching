function stance = and_all_conditions(C1, C2, C3)

n = length(C1);
stance = zeros(n,1);
for i=1:n
    if (C1(i) && C2(i) && C3(i))
        stance(i) = 1;
    end
end
fprintf('C_1 & C_2 & C_3\n');
end