function x = taha_integrate(xdot, dt, initial)

n = length(xdot);
x = zeros(n,1);
x(1) = initial;

for i=2:n
    x(i) = x(i-1) + xdot(i)*dt(i);
end
