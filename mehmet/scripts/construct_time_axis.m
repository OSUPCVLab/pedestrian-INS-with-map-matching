function time = construct_time_axis(G_Dt)
n = length(G_Dt);
time = zeros(n,1);
time(1) = 0;
for i=2:n
    time(i) = time(i-1) + G_Dt(i);
end
end