x0 = [0.075, 0.2, 0.00495914]';  % [0, 0, 0]';
v0 = [2.34375, 6.25, 0.15497299]';  %  x1.v;  %  [0, 0, 0]';
R0 = eye(3);
W0 = [0, 0, 0]';
X0 = [x0; v0; W0; reshape(R0,9,1)];

desired.x = [0, 0, 0.3]';
desired.v = [0, 0, 0]';
desired.x_dot = [0, 0, 0]';
desired.x_2dot = [0, 0, 0]';
desired.x_3dot = [0, 0, 0]';
desired.x_4dot = [0, 0, 0]';








[f, M, error, calculated] = position_control(X0, desired, k, param);