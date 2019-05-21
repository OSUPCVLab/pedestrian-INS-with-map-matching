function projected = projection_on_line(origin,s,pt)
A = [s -1; 1 s];
projected = inv(A) * [s*origin(1) - origin(2); pt(1) + s*pt(2)];