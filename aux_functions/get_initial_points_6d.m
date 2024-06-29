function initial = get_initial_points_6d(t, k, anneal_options, ...
    param, delta, init_n, Points_Array, tau, delta_position)

rng("default");

initial.init_n = init_n;
initial.initial_points = zeros(18, init_n);
initial.initial_eul = zeros(3, init_n);
initial.psi_0_list = zeros(1,init_n);
initial.V1_0_list = zeros(1,init_n);
initial.eW0_norm_list = zeros(1,init_n);
initial.ep0_list = zeros(3,init_n);
initial.ev0_list = zeros(1,init_n);
initial.eR0_list = zeros(3,init_n);
initial.safe_list = zeros(1,init_n);

tic;
for i = 1:init_n

% eR = eW = 0, sample for ep and ev
xd_0 = DesiredTrajectory(t(1),Points_Array,tau,param);
x0 = xd_0.x;  % [0, 0, 0]';
v0 = [0, 0, 0]';  %  x1.v;  %  [0, 0, 0]';
eul0 = [0, 0, 0]';
W0 = [0, 0, 0]';

if (delta_position(1) == 1)
    x0 = x0 + delta.x*(2*rand(3,1)-1);
end
if (delta_position(2) == 1)
    v0 = v0 + delta.v*(2*rand(3,1)-1);
end
if (delta_position(3) == 1)
    eul0 = eul0 + delta.R*(2*rand(3,1)-1);
end
if (delta_position(4) == 1)
    W0 = W0 + delta.W*(2*rand(3,1)-1);
end

R0 = expm(hat(eul0));
X0 = [x0; v0; W0; reshape(R0,9,1)];

% lyapunov at t=0
[~, ~, error0, calculated0] = position_control(X0, xd_0, k, param);
ep_0 = error0.x;
ev_0 = error0.v;
eR_0 = error0.R;
eW_0 = error0.W;
Rd_0 = calculated0.R;
psi_0 = get_psi(R0, Rd_0);
[V1_0, ~] = lyapunov(param, k, ep_0, ev_0, eR_0, eW_0, psi_0);
% V_0 = V1_0 + V2_0;

initial.psi_0_list(i) = psi_0;
initial.V1_0_list(i) = V1_0;
initial.eW0_norm_list(i) = 0.5*eW_0'*param.J*eW_0;
initial.ep0_list(:,i) = ep_0;
initial.eR0_list(:,i) = eR_0;
initial.ev0_list(i) = norm(ev_0);
initial.initial_points(:, i) = X0;
initial.initial_eul(:, i) = eul0;

condition1 = (psi_0 <= anneal_options.alpha_psi * anneal_options.psi_bar);
condition2 = (0.5*eW_0'*param.J*eW_0 <= k.R * (1 - anneal_options.alpha_psi) * anneal_options.psi_bar);
condition3 = (V1_0 <= anneal_options.V1_0);

if condition1 && condition2 && condition3
    initial.safe_list(i) = 1;
end

end

sampling_time = toc;

fprintf("Sampling time: %0.2f \n", sampling_time);
fprintf("Number of unsafe samples: %0.2f \n", initial.init_n - sum(initial.safe_list));
fprintf("Average sampling time per sample: %0.8f \n", sampling_time/initial.init_n);

end