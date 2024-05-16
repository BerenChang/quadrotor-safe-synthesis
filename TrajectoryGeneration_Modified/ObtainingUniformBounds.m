clc
clear all
close all


e3=[0;0;1];


% inertia matrix
%J=diag([1.43e-5,1.43e-5,2.89e-5]); % crazyfly
J=diag([0.0820, 0.0845, 0.1377]); % CDC
%J=diag([0.43,0.43, 1.02])*0.01;
%J=diag([0.007, 0.007, 0.012]);
%J=diag([0.03, 0.03, 0.03]);
%J=diag([0.0196,0.0196,0.0264]);
lambda_m_J=min(eig(J));
lambda_M_J=max(eig(J));


g=9.81;

%mass
%m=0.033; % crazyfly
m=4.34;  % CDC
%m=0.755;
%m=1.4;
%m=0.8;

% acceleration bound

% CrazyFly gain parameters and parameters for the theoretical bound
%  vm=[5;5;5];
%  am=[0.1;0.1;10];
%  kp=2;
%  kv=0.1;
%  kR=2;
%  kw=0.1;
%  V1_0=0.05; % V1(0)
%  psi_bar=0.02;
%  alpha_psi=0.9;
% c1=0.5*min([sqrt(kp*m), 4*m*kp*kv/(kv^2+4*m*kp)]);
% c2=0.3*min([sqrt(kR*lambda_m_J), 4*(lambda_m_J)*kR*kw/((kw^2)+4*(lambda_m_J)*kR)]);


% % CDCPaper parameters
vm=[2;2;2];
am=[0.1;0.1;10];
kp=29.3705; % 10;
kv=5.4928; % 5;
kR=16.3785; % 30;
kw=1.4777; % 5;
psi_bar=0.005;
alpha_psi=0.9;
V1_0=0.4;
c1=0.4915*min([sqrt(kp*m), 4*m*kp*kv/(kv^2+4*m*kp)]);
c2=0.7039*min([sqrt(kR*lambda_m_J), 4*(lambda_m_J)*kR*kw/((kw^2)+4*(lambda_m_J)*kR)]);


tic;

% this is M1
M1t=0.5*[kp*eye(3,3), c1*eye(3,3); c1*eye(3,3), m*eye(3,3)];

% this is W1

W1t=[(1/m)*c1*kp*eye(3,3) (0.5/m)*c1*kv*eye(3,3);(0.5/m)*c1*kv*eye(3,3) (kv-c1)*eye(3,3)];


% this is M2_1
M2_1t=0.5*[kR*eye(3,3) c2*eye(3,3);c2*eye(3,3) J];


% this is M2_2
M2_2t=0.5*[(2*kR/(2-psi_bar))*eye(3,3) c2*eye(3,3);c2*eye(3,3) J];

% this is W2
W2t=[c2*kR*inv(J) 0.5*c2*kw*inv(J);0.5*c2*kw*inv(J) (kw-c2)*eye(3,3)];






% bound on V2(0) in terms of psi_1 (bar{V}_{2})
V2_0=(kR+2*c2*sqrt((kR/lambda_m_J)*alpha_psi*(1-alpha_psi)))*psi_bar;

% bound on V(0)
V_0=V1_0+V2_0; 



% computing the coefficients of the bound

x1t=min(eig(inv(sqrtm(M1t))*W1t*inv(sqrtm(M1t))));

x2t=min(eig(inv(sqrtm(M2_2t))*W2t*inv(sqrtm(M2_2t))));

% this corresponds to beta
ct=x2t/2;

% this corresponds to alpha_0
a0t=min([x1t,x2t]);

% this corresponds to alpha_1*sqrt(V_2(0))
a1t=sqrt(2/(2-psi_bar))*norm([c1/m*eye(3) eye(3)]*inv(sqrtm(M1t)))*norm([kp*eye(3) kv*eye(3)]*inv(sqrtm(M1t)))*norm([eye(3) zeros(3,3)]*inv(sqrtm(M2_1t)))*sqrt(V2_0);

% this corresponds to alpha_2*sqrt(V_2(0))
a2t=m*norm(am)*sqrt(2/(2-psi_bar))*norm([c1/m*eye(3,3) eye(3,3)]*inv(sqrtm(M1t)))*norm([eye(3,3) zeros(3,3)]*inv(sqrtm(M2_1t)))*sqrt(V2_0);






% finding tm and the uniform bound depending on 0.5*alpha_0 and beta
if 0.5*a0t~=ct
X=0.5*a2t*ct/(ct-0.5*a0t);
Y=0.5*a0t*sqrt(V_0)+(0.5*a0t*0.5*a2t/(ct-0.5*a0t));
tbt1=log(X/Y)/(ct-0.5*a0t);
tbt=max([tbt1,0]);
Bound1t=0.5*a2t*((exp(-ct*tbt)-exp(-0.5*a0t*tbt))/(0.5*a0t-ct));

else
tbt1=(0.5*a2t-0.5*a0t*sqrt(V_0))/(0.25*a0t*a1t);
tbt=max([tbt1,0]);
Bound1t=0.5*a2t*tbt*exp(-0.5*a0t*tbt);

end


% bounds on sqrt(V) 
Bound0t=(exp(0.5*a1t/ct));
sqrt_V_Uniform_Bound_t=Bound0t*(sqrt(V_0)*exp(-0.5*a0t*tbt)+Bound1t);


%L_p, L_v and L_f

L_p=norm([eye(3,3) zeros(3,3)]*inv(sqrtm(M1t)))*sqrt_V_Uniform_Bound_t;

L_v=norm([zeros(3,3),eye(3,3)]*inv(sqrtm(M1t)))*sqrt_V_Uniform_Bound_t;

L_f=norm([kp*eye(3,3),kv*eye(3,3)]*inv(sqrtm(M1t)))*sqrt_V_Uniform_Bound_t;

F_bound=m*norm(am)+L_f;

t_cpu_bounds=toc;

mass=m;
save("UniformBoundsData","L_p","L_v","L_f","F_bound","mass","J","vm","am","kp","kv","kR","kw","psi_bar","alpha_psi","V1_0","V2_0","c1","c2","t_cpu_bounds");










% plotting the time-varying and uniform bound

hold on
tspan = 0:0.01:10;
t=tspan;
if 0.5*a0t~=ct
Lt=Bound0t.*sqrt(V_0)*exp(-0.5*a0t*t)+exp(0.5*a1t/ct).*0.5*a2t*((exp(-ct*t)-exp(-0.5*a0t*t))/(0.5*a0t-ct));
else
Lt=Bound0t.*sqrt(V_0)*exp(-0.5*a0t*t)+exp(0.5*a1t/ct).*0.5*a2t*t.*exp(-0.5*a0t*t);
end


yline(sqrt_V_Uniform_Bound_t,':r','linewidth',2,'displayname','L_u')
hold on
plot(t,Lt,'r','linewidth',2,'displayname','L(t)')
legend




return;


