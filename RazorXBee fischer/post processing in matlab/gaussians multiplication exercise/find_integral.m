function out = find_integral(in, T)

n = length(in);
out = 0;
for i=1:n
    out = out + in(i)*T;
end

end