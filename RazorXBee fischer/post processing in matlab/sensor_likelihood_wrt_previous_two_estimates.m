function imuPosition = sensor_likelihood_wrt_previous_two_estimates...
    (estimateKmin2,estimateKmin1,deltaImuPosition)
s1 = line_slope(estimateKmin1,estimateKmin2);
%pt1 = projection_on_line(origin,s1,pt);
%pt2 = projection_on_line(origin,s2,pt);
% (1) now we obtained the projected points.
% (2) change the basis: (i) translation (ii) rotation
%pt1 = [pt1(1) - origin(1); pt1(2) - origin(2)];
%pt2 = [pt2(1) - origin(1); pt2(2) - origin(2)];
%origin = [0; 0];
angle = atan(s1); angleDeg = rad2deg(angle);
% fprintf('%.2f\n', angleDeg);
% if (angle < 0)
%     angle = pi - abs(angle);
% end
R = [cos(angle) sin(angle); -sin(angle) cos(angle)];
imuPosition = R'*deltaImuPosition;
if (estimateKmin1(1) - estimateKmin2(1) < 0) % x is decreasing
    imuPosition = -imuPosition;
end
imuPosition = [imuPosition(1) + estimateKmin1(1); imuPosition(2) + estimateKmin1(2)];