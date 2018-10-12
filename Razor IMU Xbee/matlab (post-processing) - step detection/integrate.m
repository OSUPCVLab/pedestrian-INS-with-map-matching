function x = integrate(xdot, time)
n = length(xdot);
x = zeros(n,1);
initialState = 0;
for i=1:n
    if (i==1)
        x(i) = initialState + 0.020 * xdot(i);
    else
        x(i) = x(i-1) + (time(i) - time(i-1)) * xdot(i);
    end
end
    