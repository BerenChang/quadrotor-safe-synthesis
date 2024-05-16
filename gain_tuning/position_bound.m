function objective = position_bound(k, anneal_options, param)

kp = k(1);
kv = k(2);
kR = k(3);
kW = k(4);
d1 = k(5);
d2 = k(6);

J = param.J;
m = param.m;

lambda_m_J = min(eig(J));
% lambda_M_J = max(eig(J));

am = anneal_options.am;
% B = m*norm(am);
psi_bar = anneal_options.psi_bar;
alpha_psi = anneal_options.alpha_psi;
V1_0 = anneal_options.V1_0;
c1 = d1*min([sqrt(kp*m), 4*m*kp*kv/(kv^2+4*m*kp)]);
c2 = d2*min([sqrt(kR*lambda_m_J), 4*(lambda_m_J)*kR*kW/((kW^2)+4*(lambda_m_J)*kR)]);

M1t = 0.5*[kp*eye(3,3), c1*eye(3,3); c1*eye(3,3), m*eye(3,3)];
W1t = [(1/m)*c1*kp*eye(3,3) (0.5/m)*c1*kv*eye(3,3);(0.5/m)*c1*kv*eye(3,3) (kv-c1)*eye(3,3)];
M2_1t = 0.5*[kR*eye(3,3) c2*eye(3,3);c2*eye(3,3) J];
M2_2t = 0.5*[(2*kR/(2-psi_bar))*eye(3,3) c2*eye(3,3);c2*eye(3,3) J];
W2t = [c2*kR*inv(J) 0.5*c2*kW*inv(J);0.5*c2*kW*inv(J) (kW-c2)*eye(3,3)];

% bound on V2(0) in terms of psi_1 (bar{V}_{2})
V2_0 = (kR+2*c2*sqrt((kR/lambda_m_J)*alpha_psi*(1-alpha_psi)))*psi_bar; 

% bound on V(0)
V_0 = V1_0+V2_0; 

% computing the coefficients of the bound
x1t = min(eig(inv(sqrtm(M1t))*W1t*inv(sqrtm(M1t))));
x2t = min(eig(inv(sqrtm(M2_2t))*W2t*inv(sqrtm(M2_2t))));

% this corresponds to beta
ct = x2t/2;

% this corresponds to alpha_0
a0t = min([x1t,x2t]);

% this corresponds to alpha_1*sqrt(V_2(0))
a1t = sqrt(2/(2-psi_bar))*norm([c1/m*eye(3) eye(3)]*inv(sqrtm(M1t)))*norm([kp*eye(3) kv*eye(3)]*inv(sqrtm(M1t)))*norm([eye(3) zeros(3,3)]*inv(sqrtm(M2_1t)))*sqrt(V2_0);

% this corresponds to alpha_2*sqrt(V_2(0))
a2t = m*norm(am)*sqrt(2/(2-psi_bar))*norm([c1/m*eye(3,3) eye(3,3)]*inv(sqrtm(M1t)))*norm([eye(3,3) zeros(3,3)]*inv(sqrtm(M2_1t)))*sqrt(V2_0);

% finding tm and the uniform bound depending on 0.5*alpha_0 and beta
if 0.5*a0t ~= ct
    X = 0.5*a2t*ct/(ct-0.5*a0t);
    Y = 0.5*a0t*sqrt(V_0)+(0.5*a0t*0.5*a2t/(ct-0.5*a0t));
    tbt1 = log(X/Y)/(ct-0.5*a0t);
    tbt = max([tbt1,0]);
    Bound1t = 0.5*a2t*((exp(-ct*tbt)-exp(-0.5*a0t*tbt))/(0.5*a0t-ct));
else
    tbt1 = (0.5*a2t-0.5*a0t*sqrt(V_0))/(0.25*a0t*a2t);
    tbt = max([tbt1,0]);
    Bound1t = 0.5*a2t*tbt*exp(-0.5*a0t*tbt);
end

% bounds on sqrt(V) 
Bound0t = (exp(0.5*a1t/ct));
Lu = Bound0t*(sqrt(V_0)*exp(-0.5*a0t*tbt)+Bound1t);

%Lp, Lv and Lf

Lp = norm([eye(3,3) zeros(3,3)]*inv(sqrtm(M1t)))*Lu;
Lv = norm([zeros(3,3),eye(3,3)]*inv(sqrtm(M1t)))*Lu;
Lf = norm([kp*eye(3,3),kv*eye(3,3)]*inv(sqrtm(M1t)))*Lu;
% F_bound = m*norm(am)+Lf;

objective = anneal_options.w1*Lp + anneal_options.w2*Lv + anneal_options.w3*Lf;

end
