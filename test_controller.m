%%
% close all;
addpath('aux_functions');
addpath('test_functions');
addpath('desired_trajectories');
addpath('plotting');
addpath('gain_tuning');
addpath('TrajectoryGeneration_Modified');
rng('default');
%% Quadrotor parameters
param.g = 9.81;

% param.J = diag([0.02, 0.02, 0.04]);
% param.m = 2;

% param.J = diag([2.31e-5, 2.31e-5, 4.125e-5]);
% param.m = 0.033;

param.J = diag([0.0820, 0.0845, 0.1377]);
param.m = 4.34;

param.J_min = min(diag(param.J));
param.J_max = max(diag(param.J));

%% tuning through optimization
anneal_options.initial_solution = [10, 10, 10, 10, 0.5, 0.5];
anneal_options.lower_bound = [0, 0, 0, 0, 0, 0];
anneal_options.upper_bound = [30, 30, 30, 30, 1, 1];
anneal_options.runtime_n = 100;
anneal_options.vm = [2;2;2];
anneal_options.am = [1;1;10];
anneal_options.psi_bar = 0.005;
anneal_options.alpha_psi = 0.4;
anneal_options.V1_0 = 0.4;
anneal_options.w1 = 15;
anneal_options.w2 = 1;
anneal_options.w3 = 1;

annealing_output = annealing(anneal_options, param);

%% Controller gains
k.x = annealing_output.opt_k(1);  % 10;
k.v = annealing_output.opt_k(2);  % 8;
k.R = annealing_output.opt_k(3); % 0.003;  % 1.5;
k.W = annealing_output.opt_k(4); % 0.0005;  % 0.35;

%% trajectory generation
% t_cpu_bounds = annealing_output.opt_bounds.t_cpu_bounds;
generated_trajectory = trajectory_synthesis(annealing_output, anneal_options, param);

%% Simulation parameters
T = sum(generated_trajectory.tau);
time_step = 1e-2;
t = 0:time_step:T;
N = length(t);

%% compute the desired trajectory factorial
Control_Points_temp=generated_trajectory.Points_Array(:,:,1);
n_fact=size(Control_Points_temp,2)-1;
param.factorial_list = factorial(1:n_fact);
param.factorial_list = [1, param.factorial_list];

%% trajectory parameter
param = get_B(t, param, generated_trajectory.Points_Array, generated_trajectory.tau);
param.psi_bar = anneal_options.psi_bar;
param.V1_bar = anneal_options.V1_0;
param.alpha_psi = anneal_options.alpha_psi;

%% get initial points
init_n = 50;
delta.x = 0.3; % random draw initial points
delta.v = 0.3;
delta.R = 0.5;
delta.W = 0.5;
param.c1 = annealing_output.opt_bounds.c1;
param.c2 = annealing_output.opt_bounds.c2;
tic;
initial = get_initial_points(t, k, anneal_options, param, ...
    delta, init_n, generated_trajectory.Points_Array, generated_trajectory.tau, true);
toc
% [initial, param, M11] = get_initial_points_cond(t, k, param, delta, init_n, Points_Array, tau);

%% Numerical integrations for all initial points
ep_list = zeros(initial.init_n,N);
eW_list = zeros(initial.init_n,N);
x_list = zeros(initial.init_n,3,N);
ev_list = zeros(initial.init_n,N);
f_list = zeros(initial.init_n,N);
v_list = zeros(initial.init_n, 3,N);
Fd3_list = zeros(initial.init_n,N);
lyap_list_V = zeros(initial.init_n,N);
lyap_list_V1 = zeros(initial.init_n,N);
lyap_list_V2 = zeros(initial.init_n,N);

comp_time = zeros(1,initial.init_n);

for j = 1:initial.init_n
X0 = initial.initial_points(:, j);
tic;
[t, X] = ode45(@(t, XR) eom(t, XR, k, param, generated_trajectory.Points_Array, generated_trajectory.tau), t, X0, ...
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
    
    des = DesiredTrajectory(t(i),generated_trajectory.Points_Array,generated_trajectory.tau,param);
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
    Fd3_list(j,i) = calc.Fd3;
end

ep_list(j,:) = vecnorm(e.x);
eW_list(j,:) = vecnorm(e.W);
ev_list(j,:) = vecnorm(e.v);
f_list(j,:) = f;

lyap = get_lyapunov(k, N, param, e, R, d);
lyap_list_V(j,:) = lyap.V;
lyap_list_V1(j,:) = lyap.V1;
lyap_list_V2(j,:) = lyap.V2;

end

%% Plot data

% initial
plot_initial_condition_3d(k, anneal_options, initial);
plot_initial_position_attitude(initial);
plot_initial_position(initial);
plot_rotm(initial);

% environment and desired trajectory
generate_outputs_plots(annealing_output, anneal_options, generated_trajectory, param);

% trajectory
plot_traj(d.x, x_list, annealing_output.opt_bounds.Lp, X, false, 10);

% lyapunov
% lyap = get_lyapunov(t, k, N, param, e, R, d, M11);
plot_lyapunov(t, lyap_list_V1, annealing_output.opt_bounds.Lu);

% plot position error within Lp bound
plot_pvf(t, ep_list, ev_list, f_list, Fd3_list, initial, annealing_output.opt_bounds, 10);

% plot vx vy vz and v_bound
plot_v(t, v_list, initial, anneal_options.vm, 1);

