function [initial, param, M11] = get_initial_points_cond(t, k, param, delta, init_n, Points_Array, tau)

initial.initial_points = zeros(18, init_n);
initial.initial_eul = zeros(3, init_n);
initial.psi_0_list = zeros(1,init_n);
initial.V1_0_list = zeros(1,init_n);
initial.eW0_norm_list = zeros(1,init_n);
initial.conditions = zeros(3, init_n);
initial.ex_list = zeros(3,init_n);

for i = 1:init_n
i

% Initial conditions
% rng("default");
%{
xd_0 = DesiredTrajectory(t(1),Points_Array,tau,param);
x0 = xd_0.x + delta.x*(2*rand(3,1)-1);  % [0, 0, 0]';
% x0 = [1; 1; 0.9] + (delta.x+0.02)*(2*rand(3,1)-1);
v0 = [0, 0, 0]' + delta.v*(2*rand(3,1)-1);  %  x1.v;  %  [0, 0, 0]';
eul0 = delta.R*(2*rand(3,1)-1);
R0 = expm(hat(eul0));
W0 = [0, 0, 0]' + delta.W*(2*rand(3,1)-1);
X0 = [x0; v0; W0; reshape(R0,9,1)];
%}

xd_0 = DesiredTrajectory(t(1),Points_Array,tau,param);
x0 = mvnrnd(xd_0.x,0.01*eye(3),1)';
v0 = mvnrnd([0 0 0],0.1*eye(3),1)';
eul0 = mvnrnd([0 0 0],0.1*eye(3),1);
R0 = expm(hat(eul0));
W0 = mvnrnd([0 0 0],0.1*eye(3),1)';
X0 = [x0; v0; W0; reshape(R0,9,1)];

% lyapunov parameters
param.psi_bar = 0.01;  % psi(R(:,:,1), d.R(:,:,1));
param.c1 = min([sqrt(k.x*param.m), 4*param.m*k.x*k.v / (k.v^2+4*param.m*k.x)]);
param.c2 = min([sqrt(k.R*param.J_min), 4*param.J_min^2*k.R*k.W / (param.J_max*k.W^2+4*param.J_min^2*k.R)]);
param.c1 = 0.5*param.c1;
param.c2 = 0.3*param.c2;

% lyapunov at t=0
[~, ~, error0, calculated0] = position_control(X0, xd_0, k, param);
ex_0 = error0.x;
ev_0 = error0.v;
eR_0 = error0.R;
eW_0 = error0.W;
Rd_0 = calculated0.R;
psi_0 = get_psi(R0, Rd_0);
[V1_0, V2_0] = lyapunov(param, k, ex_0, ev_0, eR_0, eW_0, psi_0);
V_0 = V1_0 + V2_0;

% error bound
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

% ROA
param.norm_eW_bound = 2*k.R/param.J_max*(1 - param.psi_bar);
param.V1_bar = 0.1;
param.V2_bar = k.R*param.psi_bar + 2*param.c2*sqrt(k.R*param.psi_bar*(1-param.psi_bar)/param.J_max);

condition1 = (psi_0 <= param.psi_bar);
condition2 = (norm(eW_0)^2 <= param.norm_eW_bound);
condition3 = (V1_0 <= param.V1_bar);
    
initial.psi_0_list(i) = psi_0;
initial.V1_0_list(i) = V1_0;
initial.eW0_norm_list(i) = norm(eW_0)^2;
initial.initial_points(:, i) = X0;
initial.initial_eul(:, i) = eul0;
initial.conditions(:, i) = [condition1; condition2; condition3];
initial.ex_list(:, i) = ex_0;

end

% uniform position bound
initial.Lp = norm([1 0]*inv(sqrtm(M11)))*Lu(param.V1_bar, param.V2_bar, param.tm, param);

initial.init_n = init_n;

%% plot
cond1_index = (initial.conditions(1,:) == 1);
cond2_index = (initial.conditions(2,:) == 1);
cond3_index = (initial.conditions(3,:) == 1);

cond_index = cond1_index;

cla()
% initial_index = 1:initial.init_n;

scatter3(initial.initial_points(1,~cond_index), ...
        initial.initial_points(2,~cond_index), ...
        initial.initial_points(3,~cond_index), 1, 'k', 'filled');
hold on
scatter3(initial.initial_points(1,cond_index), ...
        initial.initial_points(2,cond_index), ...
        initial.initial_points(3,cond_index), 50, 'r', 'filled');
hold off

%{
scatter3(initial.ex_list(1,~cond_index), ...
        initial.ex_list(2,~cond_index), ...
        initial.ex_list(3,~cond_index), 1, 'k', 'filled');
hold on
scatter3(initial.ex_list(1,cond_index), ...
        initial.ex_list(2,cond_index), ...
        initial.ex_list(3,cond_index), 50, 'r', 'filled');
hold off
%}

end