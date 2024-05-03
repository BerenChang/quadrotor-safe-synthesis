function [M, eR, eW] = attitude_control( ...
    R, W, Rd, Wd, Wd_dot, k, param)
% [M, eR, eW] = attitude_control(R, W, Rd, Wd, Wddot, k, param)
%
% Attitude controller
% 
%   Inputs:
%    R: (3x3 matrix) current attitude in SO(3)
%    W: (3x1 matrix) current angular velocity
%    Rd: (3x3 matrix) desired attitude in SO(3)
%    Wd: (3x1 matrix) desired body angular velocity
%    Wd_dot: (3x1 matrix) desired body angular acceleration
%    k: (struct) control gains
%    param: (struct) parameters such as m, g, J in a struct
%
%  Outputs:
%    M: (3x1 matrix) control moment required to reach desired conditions
%    eR: (3x1 matrix) attitude error
%    eW: (3x1 matrix) angular velocity error

eR = 1 / 2 * vee(Rd' * R - R' * Rd);
eW = W - R' * Rd * Wd;

% M = - k.R * eR ...
%     - k.W * eW ...
%     + hat(R' * Rd * Wd) * param.J * R' * Rd * Wd ...
%     + param.J * R' * Rd * Wd_dot;

M = - k.R * eR ...
    - k.W * eW ...
    + hat(W) * param.J * W ...
    - param.J * hat(W) * R' * Rd * Wd ...
    + param.J * R' * Rd * Wd_dot;

end