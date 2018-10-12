function distance = total_distance_calculation(posX, posY)
n = length(posX);
distance = zeros(n,1);
for i=1:n
    if (i==1)
        distance(i) = 0;
    else
        distance(i) = sqrt( (posX(i)-posX(i-1))^2 + (posY(i)-posY(i-1))^2);
    end
end