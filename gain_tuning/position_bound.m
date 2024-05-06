function ep_Uniform_new = position_bound(k)

kp = k(1);
kv = k(2);
kR = k(3);
kw = k(4);
d1 = k(5);
d2 = k(6);

% inertia matrix
% J=diag([1.43e-5,1.43e-5,2.89e-5]);
J=diag([0.0820, 0.0845, 0.1377]);
%J=diag([0.43,0.43, 1.02])*0.01;
%J=diag([0.007, 0.007, 0.012]);
%J=diag([0.03, 0.03, 0.03]);
%J=diag([0.0196,0.0196,0.0264]);
lambda_m_J=min(eig(J));
lambda_M_J=max(eig(J));


%mass
% m=0.033;
m=4.34;
%m=0.755;
%m=1.4;
%m=0.8;

B=43.8319;
psi_1=0.002;
c1=d1*min([sqrt(kp*m), 4*m*kp*kv/(kv^2+4*m*kp)]);
c2=d2*min([sqrt(kR*lambda_m_J), 4*(lambda_m_J^2)*kR*kw/(lambda_M_J*(kw^2)+4*(lambda_m_J^2)*kR)]);
 
M1_1=(0.5*[kp -c1;-c1 m]);
M1_2=(0.5*[kp c1;c1 m]);
M2_1=(0.5*[kR -c2;-c2 lambda_m_J]);
M2_2=(0.5*[2*kR/(2-psi_1) c2;c2 lambda_M_J]);
W1=[(1/m)*c1*kp -(0.5/m)*c1*kv;-(0.5/m)*c1*kv kv-c1];
W2=([c2*kR/lambda_M_J -0.5*c2*kw/lambda_m_J;-0.5*c2*kw/lambda_m_J kw-c2]);

y2_0=kR*psi_1+2*c2*sqrt((kR/lambda_M_J)*psi_1*(1-psi_1)); % bound on V2(0) in terms of psi_1
y1_0=0.1; % V1(0)
y_0=y1_0+y2_0; % bound on V(0)

x1=min(eig(inv(sqrtm(M1_2))*W1*inv(sqrtm(M1_2))));
x2=min(eig(inv(sqrtm(M2_2))*W2*inv(sqrtm(M2_2))));

a0n=min([x1,x2]);
a1n=norm([c1/m 1]*inv(sqrtm(M1_1)))*norm([kp kv]*inv(sqrtm(M1_1)))*norm([1 0]*inv(sqrtm(M2_1)))*sqrt(y2_0);
cn=x2/2;
a2n=B*norm([c1/m 1]*inv(sqrtm(M1_1)))*norm([1 0]*inv(sqrtm(M2_1)))*sqrt(y2_0);

Bound0n=(exp(0.5*a1n/cn));
tbn=log(0.5*a0n/cn)/(0.5*a0n-cn);
Bound1n=exp(0.5*a1n/cn)*0.5*a2n*((exp(-cn*tbn)-exp(-0.5*a0n*tbn))/(0.5*a0n-cn));

% bounds on sqrt(V) and norm(ep)
sqrt_V_Uniform_Bound_new=(Bound0n*sqrt(y_0)+Bound1n);
ep_Uniform_new=norm([1 0]*inv(sqrtm(M1_1)))*sqrt_V_Uniform_Bound_new;

end
