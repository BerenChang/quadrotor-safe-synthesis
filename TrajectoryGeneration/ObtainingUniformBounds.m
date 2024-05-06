clc
clear all
close all


e3=[0;0;1];


% inertia matrix
%J=diag([1.43e-5,1.43e-5,2.89e-5]);
J=diag([0.0820, 0.0845, 0.1377]);
%J=diag([0.43,0.43, 1.02])*0.01;
%J=diag([0.007, 0.007, 0.012]);
%J=diag([0.03, 0.03, 0.03]);
%J=diag([0.0196,0.0196,0.0264]);
lambda_m_J=min(eig(J));
lambda_M_J=max(eig(J));


g=9.81;

%mass
%m=0.033;
m=4.34;
%m=0.755;
%m=1.4;
%m=0.8;

% acceleration bound

% CrazyFly gain parameters and parameters for the theoretical bound
% B=0.7;
% kp=1;
% kv=0.1;
% kR=10;
% kw=0.1;
% psi_1=0.01;
% c1=0.5*min([sqrt(kp*m), 4*m*kp*kv/(kv^2+4*m*kp)]);
% c2=0.3*min([sqrt(kR*lambda_m_J), 4*(lambda_m_J^2)*kR*kw/(lambda_M_J*(kw^2)+4*(lambda_m_J^2)*kR)]);
% y1_0=0.05; % V1(0)


% % CDCPaper parameters
vm=[5;5;5];
am=[1;1;10];
kp=100;
kv=1;
kR=100;
kw=1;
psi_1=0.002;
V1_0=0.01;
c1=0.5*min([sqrt(kp*m), 4*m*kp*kv/(kv^2+4*m*kp)]);
c2=0.35*min([sqrt(kR*lambda_m_J), 4*(lambda_m_J^2)*kR*kw/(lambda_M_J*(kw^2)+4*(lambda_m_J^2)*kR)]);



















 
M1_1=(0.5*[kp -c1;-c1 m]);
M1_2=(0.5*[kp c1;c1 m]);
M2_1=(0.5*[kR -c2;-c2 lambda_m_J]);
M2_2=(0.5*[2*kR/(2-psi_1) c2;c2 lambda_M_J]);
W1=[(1/m)*c1*kp -(0.5/m)*c1*kv;-(0.5/m)*c1*kv kv-c1];

W2=([c2*kR/lambda_M_J -0.5*c2*kw/lambda_m_J;-0.5*c2*kw/lambda_m_J kw-c2]);


M1=0.5*[kp*eye(3,3), c1*eye(3,3);c1*eye(3,3), m*eye(3,3)];


% smallest and largest eigenvalues of M1_1, M1_2, M2_1, M2_2, W1, and W2
l_m_M1_1=min(eig(M1_1));
l_m_M2_1=min(eig(M2_1));
l_M_M1_2=max(eig(M1_2));
l_M_M2_2=max(eig(M2_2));
l_m_W1=min(eig(W1));
l_m_W2=min(eig(W2));


V2_0=kR*psi_1+2*c2*sqrt((kR/lambda_M_J)*psi_1*(1-psi_1)); % bound on V2(0) in terms of psi_1
V_0=V1_0+V2_0; % bound on V(0)





x1=min(eig(inv(sqrtm(M1_2))*W1*inv(sqrtm(M1_2))));

x2=min(eig(inv(sqrtm(M2_2))*W2*inv(sqrtm(M2_2))));


a0n=min([x1,x2]);
a1n=norm([c1/m 1]*inv(sqrtm(M1_1)))*norm([kp kv]*inv(sqrtm(M1_1)))*norm([1 0]*inv(sqrtm(M2_1)))*sqrt(V2_0);
cn=x2/2;
a2n=m*norm(am)*norm([c1/m 1]*inv(sqrtm(M1_1)))*norm([1 0]*inv(sqrtm(M2_1)))*sqrt(V2_0);

Bound0n=(exp(0.5*a1n/cn));
if 0.5*a0n~=cn
tbn=log(0.5*a0n/cn)/(0.5*a0n-cn);
Bound1n=exp(0.5*a1n/cn)*0.5*a2n*((exp(-cn*tbn)-exp(-0.5*a0n*tbn))/(0.5*a0n-cn));

else
tbn=1/(0.5*a0n*exp(1));
Bound1n=exp(0.5*a1n/cn)*0.5*a2n*tbn*exp(-0.5*a0n*tbn);
end



% bounds on sqrt(V) and norm(ep)
sqrt_V_Uniform_Bound_new=(Bound0n*sqrt(V_0)+Bound1n);
%tspan = 0:0.05:200;
%[t,y] = ode45(@(t,y)-(a0n-a1n*exp(-cn*t))*y+a2n*exp(-cn*t)*sqrt(y), tspan, y_0);
%sqrt_V_Uniform_Bound_new=max(sqrt(y))
L_p=norm([1 0]*inv(sqrtm(M1_1)))*sqrt_V_Uniform_Bound_new
L_v=norm([0 1]*inv(sqrtm(M1_1)))*sqrt_V_Uniform_Bound_new
L_f=norm([kp kv]*inv(sqrtm(M1_1)))*sqrt_V_Uniform_Bound_new
mass=m;
F_bound=m*norm(am)+L_f;
save("UniformBoundsData","L_p","L_v","L_f","F_bound","mass","J","vm","am","kp","kv","kR","kw","psi_1","V1_0","c1","c2");



