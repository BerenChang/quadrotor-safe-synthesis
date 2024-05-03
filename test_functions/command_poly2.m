function desired = command_poly1(t)

p = [-0.0001    0.0028   -0.0464    0.3702   -1.3808    1.9084   -0.1141    0.0002];

t1 = [t^7, t^6, t^5, t^4, t^3, t^2, t, 1];
t2 = [7*t^6, 6*t^5, 5*t^4, 4*t^3, 3*t^2, 2*t, 1, 0];
t3 = [42*t^5, 30*t^4, 20*t^3, 12*t^2, 6*t, 2, 0, 0];
t4 = [210*t^4, 120*t^3, 60*t^2, 24*t, 6, 0, 0, 0];
t5 = [840*t^3, 360*t^2, 120*t, 24, 0, 0, 0, 0];

desired.x = (p*t1')*ones(3,1);

desired.v = (p*t2')*ones(3,1);

desired.x_2dot = (p*t3')*ones(3,1);

desired.x_3dot = (p*t4')*ones(3,1);

desired.x_4dot = (p*t5')*ones(3,1);

w = 0;  % 2 * pi / 10;
desired.w = w;
desired.b1 = [cos(w * t), sin(w * t), 0]';
desired.b1_dot = w * [-sin(w * t), cos(w * t), 0]';
desired.b1_2dot = w^2 * [-cos(w * t), -sin(w * t), 0]';

end