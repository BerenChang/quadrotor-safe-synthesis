%%
close all;
addpath('aux_functions');
addpath('test_functions');
addpath('desired_trajectories');
addpath('plotting');

%%
% "BezierCurveParameters.mat"
% "BezierCurveParameters_1.mat"
BezierCurveParameters = "BezierCurveParameters_2.mat";
load(BezierCurveParameters);

%% Simulation parameters
T = sum(tau);
t = 0:1e-2:T;
N = length(t);
param.g = 9.81;

% Quadrotor parameters
% param.J = diag([0.02, 0.02, 0.04]);
% param.m = 2;

% param.J = diag([0.0820, 0.0845, 0.1377]);
% param.m = 4.34;

param.J = diag([1.43e-5, 1.43e-5, 2.89e-5]);
param.m = 0.033;

param.J_min = min(diag(param.J));
param.J_max = max(diag(param.J));

%% Controller gains
k.x = 10;  % 10;
k.v = 0.1;  % 8;

% Attitude
k.R = 6.5; % 0.003;  % 1.5;
k.W = 0.1; % 0.0005;  % 0.35;

%% trajectory parameter
param = get_B(t, param, Points_Array, tau);

%% get initial points
initial_point_number = 10;
delta.x = 0.3; % random draw initial points
delta.v = 0.3;
delta.R = 0.5;
delta.W = 0.5;
[initial, param, M11] = get_initial_points(t, k, param, delta, initial_point_number,Points_Array,tau);

%% Numerical integrations for all initial points
ep_list = zeros(initial_point_number,N);
eW_list = zeros(initial_point_number,N);
x_list = zeros(initial_point_number,3,N);

comp_time = zeros(1,initial_point_number);

for j = 1:initial_point_number
X0 = initial.initial_points(:, j);
tic;
[t, X] = ode78(@(t, XR) eom(t, XR, k, param, Points_Array, tau), t, X0, ...
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

for i = 1:N
    R(:,:,i) = reshape(X(i,10:18), 3, 3);
    
    des = DesiredTrajectory(t(i),Points_Array,tau);
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
end

%% get Lyapunov values
[V1, V2, V, V_bound, uniform_V_bound, error_bound_p_list] = get_lyapunov(t, k, N, param, e, R, d, M11);

%% Plot data

% plot error
plot_error(t, e);
plot_error_norm(t, e);

% trajectory
plot_traj(d.x, x_list, initial.Lp, false, X);

% lyapunov
plot_lyapunov(V1, V2, V, V_bound, uniform_V_bound);

% initial condition satisfaction
plot_initial_condition(t, param, ep_list, initial, initial_point_number);
