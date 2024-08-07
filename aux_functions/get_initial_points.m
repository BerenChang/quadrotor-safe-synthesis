function initial = get_initial_points(t, k, anneal_options, ...
    param, delta, init_n, Points_Array, tau, show_count)

rng("default");

initial.init_n = init_n;
initial.initial_points = zeros(18, init_n);
initial.initial_eul = zeros(3, init_n);
initial.psi_0_list = zeros(1,init_n);
initial.V1_0_list = zeros(1,init_n);
initial.eW0_norm_list = zeros(1,init_n);

% error bound
%{
M11 = 0.5*[k.x, -param.c1; 
        -param.c1, param.m];
M12 = 0.5*[k.x, param.c1; 
        param.c1, param.m];
W1 = [param.c1*k.x/param.m, -param.c1*k.v/2/param.m; 
    -param.c1*k.v/2/param.m, k.v - param.c1];
M21 = 0.5*[k.R, -param.c2; 
        -param.c2, param.J_min];
M22 = 0.5*[2*k.R/(2-param.psi_bar), param.c2; 
        param.c2, param.J_max];
W2 = [param.c2*k.R/param.J_max, -param.c2*k.W/2/param.J_min;
      -param.c2*k.W/2/param.J_min, k.W - param.c2];

param.alpha0 = min([min(eig(inv(sqrtm(M12))*W1*inv(sqrtm(M12)))), min(eig(inv(sqrtm(M22))*W2*inv(sqrtm(M22))))]);
param.alpha1 = norm([param.c1/param.m 1]*inv(sqrtm(M11)))*norm([k.x k.v]*inv(sqrtm(M11)))*norm([1 0]*inv(sqrtm(M21)));
param.beta = min(eig(inv(sqrtm(M22))*W2*inv(sqrtm(M22))))/2;
param.alpha2 = param.B*norm([param.c1/param.m 1]*inv(sqrtm(M11)))*norm([1 0]*inv(sqrtm(M21)));
param.tm = log(param.alpha0/2/param.beta)/(param.alpha0/2-param.beta);
% tm = min([tm_uncrop, T]);
%}

initial_point_count = 1;
while initial_point_count ~= init_n+1

% Initial conditions

xd_0 = DesiredTrajectory(t(1),Points_Array,tau,param);
x0 = xd_0.x + delta.x*(2*rand(3,1)-1);  % [0, 0, 0]';
v0 = [0, 0, 0]' + delta.v*(2*rand(3,1)-1);  %  x1.v;  %  [0, 0, 0]';
eul0 = delta.R*(2*rand(3,1)-1);
R0 = expm(hat(eul0));
W0 = [0, 0, 0]' + delta.W*(2*rand(3,1)-1);
X0 = [x0; v0; W0; reshape(R0,9,1)];

%{
xd_0 = DesiredTrajectory(t(1),Points_Array,tau,param);
x0 = mvnrnd(xd_0.x,0.1*eye(3),1)';
v0 = mvnrnd([0 0 0],0.1*eye(3),1)';
eul0 = mvnrnd([0 0 0],0.1*eye(3),1);
R0 = expm(hat(eul0));
W0 = mvnrnd([0 0 0],0.1*eye(3),1)';
X0 = [x0; v0; W0; reshape(R0,9,1)];
%}

% lyapunov at t=0
[~, ~, error0, calculated0] = position_control(X0, xd_0, k, param);
ex_0 = error0.x;
ev_0 = error0.v;
eR_0 = error0.R;
eW_0 = error0.W;
Rd_0 = calculated0.R;
psi_0 = get_psi(R0, Rd_0);
[V1_0, ~] = lyapunov(param, k, ex_0, ev_0, eR_0, eW_0, psi_0);
% V_0 = V1_0 + V2_0;

condition1 = (psi_0 <= anneal_options.alpha_psi * anneal_options.psi_bar);
condition2 = (0.5*eW_0'*param.J*eW_0 <= k.R * (1 - anneal_options.alpha_psi) * anneal_options.psi_bar);
condition3 = (V1_0 <= anneal_options.V1_0);

[condition1, condition2, condition3];

if condition1 && condition2 && condition3
    initial.psi_0_list(initial_point_count) = psi_0;
    initial.V1_0_list(initial_point_count) = V1_0;
    initial.eW0_norm_list(initial_point_count) = 0.5*eW_0'*param.J*eW_0;
    initial.initial_points(:, initial_point_count) = X0;
    initial.initial_eul(:, initial_point_count) = eul0;
    initial_point_count = initial_point_count + 1;
    if show_count
        initial_point_count - 1
    end
end

end

% uniform position bound
% initial.Lp = norm([1 0]*inv(sqrtm(M11)))*Lu(param.V1_bar, param.V2_bar, param);
% initial.Lv = norm([0 1]*inv(sqrtm(M11)))*Lu(param.V1_bar, param.V2_bar, param);
% initial.Lf = norm([k.x k.v]*inv(sqrtm(M11)))*Lu(param.V1_bar, param.V2_bar, param);
% initial.F_bound = param.m*norm(am) + initial.Lf;

end