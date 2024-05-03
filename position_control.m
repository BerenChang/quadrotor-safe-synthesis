function [f, M, error, calculated] ...
    = position_control(X, desired, k, param)
% calculated based on Trajectory tracking controller for quadrotors
% without velocity and angular velocity measurements
% Manually differentiate

% [f, M, error, calculated] = position_control(X, desired, k, param)
%
%   Inputs:
%    X: (24x1 matrix) states of the system (x, v, R, W, ei, eI)
%    desired: (struct) desired states
%    k: (struct) control gains
%    param: (struct) parameters such as m, g, J in a struct
%
%  Outputs:
%    f: (scalar) required motor force
%    M: (3x1 matrix) control moment required to reach desired conditions
%    error: (struct) errors for attitude and position control (for data
%    logging)
%    calculated: (struct) calculated desired commands (for data logging)

% Unpack states
[x, v, R, W] = split_to_states(X);

m = param.m;
g = param.g;
e1 = [1, 0, 0]';
e3 = [0, 0, 1]';

error.x = x - desired.x;
error.v = v - desired.v;

F = - k.x * error.x - k.v * error.v + m * g * e3 + m * desired.x_2dot;
nF = norm(F);
F1 = F(1);
F2 = F(2);
F3 = F(3);
f1 = F1/nF;
f2 = F2/nF;
f3 = F3/nF;

b3 = R * e3;
f = dot(F, b3);
ev_dot = - g * e3 + f / m * b3 - desired.x_2dot;
F_dot = - k.x * error.v - k.v * ev_dot + m * desired.x_3dot;
nF_dot = F'*F_dot/nF;
F1_dot = F_dot(1);
F2_dot = F_dot(2);
F3_dot = F_dot(3);
f1_dot = F1_dot/nF - F1*nF_dot/nF^2;
f2_dot = F2_dot/nF - F2*nF_dot/nF^2;
f3_dot = F3_dot/nF - F3*nF_dot/nF^2;

b3_dot = R * hat(W) * e3;
f_dot = dot(F_dot, b3) + dot(F, b3_dot);
ev_ddot = f_dot / m * b3 + f / m * b3_dot - desired.x_3dot;
F_ddot = - k.x * ev_dot - k.v * ev_ddot + m * desired.x_4dot;
nF_ddot = (F_dot'*F_dot + F'*F_ddot)/nF - (F'*F_dot)^2/nF^3;
F1_ddot = F_ddot(1);
F2_ddot = F_ddot(2);
F3_ddot = F_ddot(3);
f1_ddot = F1_ddot/nF - F1_dot*nF_dot/nF^2 - (F1_dot*nF_dot+F1*nF_ddot)/nF^2 + 2*F1*nF_dot^2/nF^3;
f2_ddot = F2_ddot/nF - F2_dot*nF_dot/nF^2 - (F2_dot*nF_dot+F2*nF_ddot)/nF^2 + 2*F2*nF_dot^2/nF^3;
f3_ddot = F3_ddot/nF - F3_dot*nF_dot/nF^2 - (F3_dot*nF_dot+F3*nF_ddot)/nF^2 + 2*F3*nF_dot^2/nF^3;

b1d = [f3+f2^2/(1+f3); -f1*f2/(1+f3); -f1];
b2d = [-f1*f2/(1+f3); f3+f1^2/(1+f3); -f2];
b3d = [f1; f2; f3];
Rd = [b1d, b2d, b3d];

b1d_dot = [f3_dot+2*f2*f2_dot/(1+f3)-f2^2*f3_dot/(1+f3)^2; 
    -(f1_dot*f2+f1*f2_dot)/(1+f3) + f1*f2*f3_dot/(1+f3)^2;
    -f1_dot];
b2d_dot = [b1d_dot(2); 
    f3_dot+2*f1*f1_dot/(1+f3)-f1^2*f3_dot/(1+f3)^2; 
    -f2_dot];
b3d_dot = [f1_dot; f2_dot; f3_dot];
Rd_dot = [b1d_dot, b2d_dot, b3d_dot];

b1d_ddot = [f3_ddot + (2*f2_dot^2+2*f2*f2_ddot)/(1+f3) - (4*f2*f2_dot*f3_dot+f2^2*f3_ddot)/(1+f3)^2 + (2*f2^2*f3_dot^2)/(1+f3)^3;
    -(f1_ddot*f2+2*f1_dot*f2_dot+f1*f2_ddot)/(1+f3) + (2*f1_dot*f2*f3_dot+2*f1*f2_dot*f3_dot+f1*f2*f3_ddot)/(1+f3)^2 - 2*f1*f2*f3_dot^2/(1+f3)^3;
    -f1_ddot];
b2d_ddot = [b1d_ddot(2);
    f3_ddot + (2*f1_dot^2+2*f1*f1_ddot)/(1+f3) - (4*f1*f1_dot*f3_dot+f1^2*f3_ddot)/(1+f3)^2 + (2*f1^2*f3_dot^2)/(1+f3)^3;
    -f2_ddot];
b3d_ddot = [f1_ddot; f2_ddot; f3_ddot];
Rd_ddot = [b1d_ddot, b2d_ddot, b3d_ddot];

Wd = vee(Rd' * Rd_dot);
Wd_dot = vee(Rd' * Rd_ddot - hat(Wd)^2);

W3 = dot(R * e3, Rd * Wd);
W3_dot = dot(R * e3, Rd * Wd_dot) ...
    + dot(R * hat(W) * e3, Rd * Wd);

%% Run attitude controller
[M, error.R, error.W] = attitude_control(R, W, Rd, Wd, Wd_dot, k, param);

%% Saving data
calculated.b3 = Rd*e3;
calculated.b3_dot = Rd_dot*e3;
calculated.b3_ddot = Rd_ddot*e3;
calculated.b1 = Rd*e1;
calculated.R = Rd;
calculated.W = Wd;
calculated.W_dot = Wd_dot;
calculated.W3 = W3;
calculated.W3_dot = W3_dot;

calculated.Rd_dot = Rd_dot;
calculated.Rd_ddot = Rd_ddot;
end