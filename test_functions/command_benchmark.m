function desired = command_benchmark(t)

A = 0.4;
B = 0.4;
C = 0.6;

a = 1;
b = pi;
c = pi;
alt = -1;

% t = linspace(0, 2*pi, 2*pi*100+1);
% x = A * sin(a * t + d);
% y = B * sin(b * t);
% z = alt + C * cos(2 * t);
% plot3(x, y, z);

desired.x = [A * t, B * sin(b * t), C * cos(c * t)]';

desired.v = [A, ...
    B * b * cos(b * t), ...
    C * c * -sin(c * t)]';

desired.x_2dot = [0, ...
    B * b^2 * -sin(b * t), ...
    C * c^2 * -cos(c * t)]';

desired.x_3dot = [0, ...
    B * b^3 * -cos(b * t), ...
    C * c^3 * sin(c * t)]';

desired.x_4dot = [0, ...
    B * b^4 * sin(b * t), ...
    C * c^4 * cos(c * t)]';

w = pi;
desired.b1 = [cos(w * t), sin(w * t), 0]';
desired.b1_dot = w * [-sin(w * t), cos(w * t), 0]';
desired.b1_2dot = w^2 * [-cos(w * t), -sin(w * t), 0]';

end