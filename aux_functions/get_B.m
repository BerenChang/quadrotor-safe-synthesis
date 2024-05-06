function param = get_B(t, param, Points_Array, tau)
N = length(t);
B_list = zeros(1,N);
for i = 1:N
    des = DesiredTrajectory(t(i),Points_Array,tau,param);
    B_list(i) = norm(param.m*(des.x_2dot + [0;0;param.g]));
end
param.B = max(B_list);
end