function desired = command_poly1(t)

desired.x = [0.1*t^3; 0.2*t^3; 0.3*t^3];

desired.v = [0.3*t^2; 0.6*t^2; 0.9*t^2];

desired.x_2dot = [0.6*t; 1.2*t; 1.8*t];

desired.x_3dot = [0.6; 1.2; 1.8];

desired.x_4dot = [0;0;0];

w = 0;  % 2 * pi / 10;
desired.w = w;
desired.b1 = [cos(w * t), sin(w * t), 0]';
desired.b1_dot = w * [-sin(w * t), cos(w * t), 0]';
desired.b1_2dot = w^2 * [-cos(w * t), -sin(w * t), 0]';

end