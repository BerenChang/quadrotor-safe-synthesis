function Xdot = eom(t, X, k, param)
t;

e3 = [0, 0, 1]';
m = param.m;
J = param.J;

[~, v, R, W] = split_to_states(X);

desired = command(t);
[f, M, ~, ~] = position_control(X, desired, k, param);

xdot = v;
vdot = - param.g * e3 ...
    + f / m * R * e3;
Wdot = J \ (-hat(W) * J * W + M);
Rdot = R * hat(W);

Xdot=[xdot; vdot; Wdot; reshape(Rdot,9,1)];
end