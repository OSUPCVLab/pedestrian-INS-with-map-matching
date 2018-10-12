function out = find_integral_2d(in, T)

n = size(in);
out = 0;
for i=1:n(1)
    for j=1:n(2)
        out = out + in(i,j)*T*T;
    end
end

end