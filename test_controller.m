%%

% close all;
addpath('aux_functions');
addpath('test_functions');
addpath('desired_trajectories');
addpath('plotting');

%% Load the desired trajectory
% "BezierCurveParameters.mat"
% "BezierCurveParameters_1.mat"
% BezierCurveParameters = "BezierCurveParameters_3.mat";
BezierCurveParameters = "/home/hybridsystemlab/Documents/MATLAB/quadrotor-safe-synthesis/TrajectoryGeneration/GeneratedTrajectoryData.mat";
load(BezierCurveParameters);

%% Simulation parameters
T = sum(tau);
time_step = 1e-2;
t = 0:time_step:T;
N = length(t);
param.g = 9.81;

% Quadrotor parameters
% param.J = diag([0.02, 0.02, 0.04]);
% param.m = 2;

param.J = diag([0.0820, 0.0845, 0.1377]);
param.m = 4.34;

% param.J = diag([1.43e-5, 1.43e-5, 2.89e-5]);
% param.m = 0.033;

param.J_min = min(diag(param.J));
param.J_max = max(diag(param.J));

%% compute the desired trajectory factorial
Control_Points_temp=Points_Array(:,:,1);
n_fact=size(Control_Points_temp,2)-1;
param.factorial_list = factorial(1:n_fact);
param.factorial_list = [1, param.factorial_list];

%% Controller gains
k.x = 99.7258;  % 10;
k.v = 4.6855;  % 8;

% Attitude
k.R = 94.7636; % 0.003;  % 1.5;
k.W = 2.0664; % 0.0005;  % 0.35;

%% trajectory parameter
param = get_B(t, param, Points_Array, tau);
param.psi_bar = 0.002;
param.V1_bar = 0.1;

%% get initial points
init_n = 10;
delta.x = 0.3; % random draw initial points
delta.v = 0.3;
delta.R = 0.1;
delta.W = 30;
tic;
[initial, param, M11] = get_initial_points(t, k, param, delta, init_n, Points_Array, tau);
toc
% [initial, param, M11] = get_initial_points_cond(t, k, param, delta, init_n, Points_Array, tau);

plot_initial_condition_3d(param, initial);

%% plot initial points
figure;
scatter3(initial.initial_points(1,:),initial.initial_points(2,:),initial.initial_points(3,:));
figure;
scatter3(initial.initial_points(4,:),initial.initial_points(5,:),initial.initial_points(6,:));
figure;
scatter3(initial.initial_eul(1,:),initial.initial_eul(2,:),initial.initial_eul(3,:));
figure;
scatter3(initial.initial_points(7,:),initial.initial_points(8,:),initial.initial_points(9,:));

%% Numerical integrations for all initial points
ep_list = zeros(initial.init_n,N);
eW_list = zeros(initial.init_n,N);
x_list = zeros(initial.init_n,3,N);
ev_list = zeros(initial.init_n,N);
f_list = zeros(initial.init_n,N);
v_list = zeros(initial.init_n, 3,N);

comp_time = zeros(1,initial.init_n);

for j = 1:initial.init_n
X0 = initial.initial_points(:, j);
tic;
[t, X] = ode45(@(t, XR) eom(t, XR, k, param, Points_Array, tau), t, X0, ...
    odeset('RelTol', 1e-6, 'AbsTol', 1e-6));
comp_time(j) = toc;

% Post processing

% Create empty arrays to save data
[e, d, R, f, M] = generate_output_arrays(N);

% Unpack the outputs of ode45 function
x = X(:, 1:3)';
v = X(:, 4:6)';
W = X(:, 7:9)';

x_list(j,:,:) = x;
v_list(j,:,:) = v;

for i = 1:N
    R(:,:,i) = reshape(X(i,10:18), 3, 3);
    
    des = DesiredTrajectory(t(i),Points_Array,tau,param);
    % des = command(t(i));
    [f(i), M(:,i), err, calc] = position_control(X(i,:)', des, ...
        k, param);
    
    % Unpack errors
    e.x(:,i) = err.x;
    e.v(:,i) = err.v;
    e.R(:,i) = err.R;
    e.W(:,i) = err.W;
    
    % Unpack desired values
    d.x(:,i) = des.x;
    d.v(:,i) = des.v;
    d.b1(:,i) = des.b1;
    d.R(:,:,i) = calc.R;
    % d.R_dot(:,:,i) = calc.Rd_dot;
    % d.R_ddot(:,:,i) = calc.Rd_ddot;
end

ep_list(j,:) = vecnorm(e.x);
eW_list(j,:) = vecnorm(e.W);
ev_list(j,:) = vecnorm(e.v);
f_list(j,:) = f;

end

%% get Lyapunov values
[V1, V2, V, V_bound, uniform_V_bound, error_bound_p_list] = get_lyapunov(t, k, N, param, e, R, d, M11);

%% Plot data

% plot error
% plot_error(t, e);
% plot_error_norm(t, e);

% trajectory
plot_traj(d.x, x_list, initial.Lp, X, false, false);

% plot zoom-in trajectory
% plot_traj(d.x, x_list, initial.Lp, X, false, true);

% lyapunov
% plot_lyapunov(V1, V2, V, V_bound, uniform_V_bound);

% initial condition satisfaction
% plot_initial_condition(t, param, ep_list, initial);

%% satisfaction of three initial condition, present in 3d
plot_initial_condition_3d(param, initial);

%% plot position error within Lp bound
% plot_Lp(t, ep_list, initial);
% plot_Lv(t, ev_list, initial);
% plot_Lf(t, f_list, initial, param.B);

plot_pvf(t, ep_list, ev_list, f_list, initial);

%% v_max


%% comparison between two position error bounds from stability analysis
%{
% Plot each curve with different colors and line styles
% Generate distinguishable colors
colors = [0, 0.4470, 0.7410;     % blue
          0.8500, 0.3250, 0.0980; % orange
          0.9290, 0.6940, 0.1250; % yellow
          0.4940, 0.1840, 0.5560; % purple
          0.4660, 0.6740, 0.1880; % green
          0.3010, 0.7450, 0.9330; % light blue
          0.6350, 0.0780, 0.1840; % dark red
          0.8500, 0.3250, 0.0980; % orange (again)
          0.75, 0.75, 0;           % gold
          0, 0.5, 0];              % dark green 

figure;
hold on;

yline(V(1)/min(eig(M11)), 'Color', [0.4, 0.2, 0.4], 'LineWidth', 1.5);
yline(initial.Lp, 'Color', [0.8, 0.2, 0.4], 'LineWidth', 1.5);

for i = 1:initial.init_n
    error_p = squeeze(ep_list(i,:));
    plot(t(1:200), error_p(1:200), 'Color', colors(i, :), 'LineWidth', 1.5);
end

% Add labels and title
xlabel('$t$ (second)','interpreter','latex');
ylabel('$\|e_p\|$ (meter)','interpreter','latex');
title('Norm of position error vs time','interpreter','latex');

% Add legend
legend('Bound1','Bound2');

% Adjust plot appearance
yscale log;
grid on;
box on;
set(gca, 'FontName', 'Arial');
set(gca, 'FontSize', 12);
set(gca, 'LineWidth', 1.2);
set(gca, 'TickDir', 'out');
set(gca, 'TickLength', [0.02, 0.02]);
set(gca, 'XMinorTick', 'on');
set(gca, 'YMinorTick', 'on');
set(gca,'TickLabelInterpreter','latex');

hold off;
%}

