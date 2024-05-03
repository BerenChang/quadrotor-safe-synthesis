function [V1, V2, V, V_bound, uniform_V_bound, error_bound_p_list] = get_lyapunov(t, k, N, param, e, R, d, M11)

psi_list = zeros(1,N);
for i = 1:N
    psi_list(i) = get_psi(R(:,:,i), d.R(:,:,i));
end

V1 = zeros(1,N);
V2 = zeros(1,N);
for i = 1:N
    psi_i = get_psi(R(:,:,i), d.R(:,:,i));
    [V1(i), V2(i)] = lyapunov(param, k, e.x(:,i), e.v(:,i), e.R(:,i), e.W(:,i), psi_i);
    V1(i) = 0.5*k.x*dot(e.x(:,i), e.x(:,i)) + 0.5*param.m*dot(e.v(:,i), e.v(:,i)) + param.c1*dot(e.x(:,i), e.v(:,i));
    V2(i) = 0.5*dot(e.W(:,i), param.J*e.W(:,i)) + k.R * get_psi(R(:,:,i), d.R(:,:,i)) + param.c2*dot(e.R(:,i), e.W(:,i));
end

V = V1 + V2;

V_bound = zeros(1,N);
for i = 1:N
    V_bound(i) = (L(V1(1), V2(1), t(i), param))^2;
end

error_bound_p_list = zeros(1,N);
for i = 1:N
    error_bound_p_list(i) = norm([1 0]*inv(sqrtm(M11)))*L(V1(1), V2(1), t(i), param);
end

uniform_V_bound = (Lu(V1(1), V2(1), param.tm, param))^2;

end