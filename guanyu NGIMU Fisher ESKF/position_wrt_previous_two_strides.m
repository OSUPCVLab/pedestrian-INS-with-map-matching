function ptNewFrame = position_wrt_previous_two_strides(ptMin2,ptMin1,pt)
s1 = line_slope(ptMin1,ptMin2);
%pt1 = projection_on_line(origin,s1,pt);
%pt2 = projection_on_line(origin,s2,pt);
% (1) now we obtained the projected points.
% (2) change the basis: (i) translation (ii) rotation
%pt1 = [pt1(1) - origin(1); pt1(2) - origin(2)];
%pt2 = [pt2(1) - origin(1); pt2(2) - origin(2)];
pt = [pt(1) - ptMin1(1); pt(2) - ptMin1(2)];
%origin = [0; 0];
angle = atan(s1); %angleDeg = rad2deg(angle);
%fprintf('%.2f\n', angleDeg);
% if (angle < 0)
%     angle = pi - abs(angle);
% end
R = [cos(angle) sin(angle); -sin(angle) cos(angle)];
ptNewFrame = R*pt;
if (ptMin1(1) - ptMin2(1) < 0) % x is decreasing
    ptNewFrame = -ptNewFrame;
end

%R = inv(R);

%ptNewFrame = [sqrt((pt2(2) - pt(2))^2 + (pt2(1) - pt(1))^2);
%              sqrt((pt1(2) - pt(2))^2 + (pt1(1) - pt(1))^2)];