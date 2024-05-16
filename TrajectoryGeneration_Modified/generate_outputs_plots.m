function generate_outputs_plots(annealing_output, anneal_options, generated_trajectory, param)

Lp = annealing_output.opt_bounds.Lp;
Lv = annealing_output.opt_bounds.Lv;
% Lf = annealing_output.opt_bounds.Lf;
F_bound = annealing_output.opt_bounds.F_bound;
mass = param.m;
J = param.J;
vm = anneal_options.vm;
am = anneal_options.am;
kp = annealing_output.opt_k(1);
kv = annealing_output.opt_k(2);
kR = annealing_output.opt_k(3);
kw = annealing_output.opt_k(4);
psi_bar = anneal_options.psi_bar;
alpha_psi = anneal_options.alpha_psi;
V1_0 = anneal_options.V1_0;
V2_0 = annealing_output.opt_bounds.V2_0;
c1 = annealing_output.opt_bounds.c1;
c2 = annealing_output.opt_bounds.c2;
t_cpu_bounds = annealing_output.opt_bounds.t_cpu_bounds;

alpha = generated_trajectory.alpha;
C_sample = generated_trajectory.C_sample;
Nv = generated_trajectory.Nv;
epsilon = generated_trajectory.epsilon;
alpha_T = generated_trajectory.alpha_T;
T0 = generated_trajectory.T0;
N_pts = generated_trajectory.N_pts;
X_Array = generated_trajectory.X_Array;
R_Array = generated_trajectory.R_Array;
Points_Array = generated_trajectory.Points_Array;
tau = generated_trajectory.tau;
XulArray = generated_trajectory.XulArray;
XuuArray = generated_trajectory.XuuArray;
Xtl = generated_trajectory.Xtl;
Xtu = generated_trajectory.Xtu;
Xsl = generated_trajectory.Xsl;
Xsu = generated_trajectory.Xsu;
t_c1 = generated_trajectory.t_c1;
t_c2 = generated_trajectory.t_c2;

sXu=size(XulArray);
Nu=sXu(2); % number of obstacles

M=size(X_Array,2); % number of waypoints
N_seg=M-1;% number of segments 

N=100; % number of points per array for plotting


T=sum(tau); % total time horizon
t=linspace(0,T,N); % time array
X=zeros(3,N); % position array
V=zeros(3,N); % velocity array
A=zeros(3,N); % acceleration array
Je=zeros(3,N); % jerk array % I'm using a different notation as J is inertia matrix
S=zeros(3,N); % snap array




% function DesiredTrajectory outputs p, pdot, pddot,pdddot, and pddddot as 
% functions of time. Function DesiredTrajectory requires the arrays 
%Points_Array (control points), and tau (duration of each segment) 

for i=1:N
    [X(:,i),V(:,i),A(:,i),Je(:,i),S(:,i)]=DesiredTrajectoryAndDerivatives(t(i),Points_Array,tau); 
    
end


figure % plotting the map only
hold on
plotcube(transpose(Xtu-Xtl),transpose(Xtl),0.5,[0,0,1])
for i=1:Nu
plotcube(transpose(XuuArray(:,i)-XulArray(:,i)),transpose(XulArray(:,i)),0.5,[1 0 0]) 
end

scatter3(1,1,1,'*')

set(gca,'fontsize',15)
set(gca,'ticklabelinterpreter','latex')
xlabel('$x$ [m]','interpreter','latex')
ylabel('$y$ [m]','interpreter','latex')
zlabel('$z$ [m]','interpreter','latex')
xlim([Xsl(1),Xsu(1)])
ylim([Xsl(2),Xsu(2)])
zlim([Xsl(3),Xsu(3)])
box on
grid on
view(-100,30)


figure %plotting the safety tube within the map
hold on

plotcube(transpose(Xtu-Xtl),transpose(Xtl),0.05,[0,0,1])
for i=1:Nu
plotcube(transpose(XuuArray(:,i)-XulArray(:,i)),transpose(XulArray(:,i)),0.05,[1 0 0]) 
end

for k=1:M
 plotcube(transpose(2*R_Array(:,k)),transpose(X_Array(:,k)-R_Array(:,k)),0.5,[0,1,1])
end
% plot3(X(1,:),X(2,:),X(3,:),'b','linewidth',2)
scatter3(1,1,1,'*')

set(gca,'fontsize',15)
set(gca,'ticklabelinterpreter','latex')
xlabel('$x$ [m]','interpreter','latex')
ylabel('$y$ [m]','interpreter','latex')
zlabel('$z$ [m]','interpreter','latex')
xlim([Xsl(1),Xsu(1)])
ylim([Xsl(2),Xsu(2)])
zlim([Xsl(3),Xsu(3)])
view(-100,30)
box on
grid on



figure % plotting the desired trajectoy within the safet tube
hold on
for k=1:M
 plotcube(transpose(2*R_Array(:,k)),transpose(X_Array(:,k)-R_Array(:,k)),0.1,[0,1,1])

end
plot3(X(1,:),X(2,:),X(3,:),'b','linewidth',2)
hold on


set(gca,'fontsize',15)
set(gca,'ticklabelinterpreter','latex')
xlabel('$x$ [m]','interpreter','latex')
ylabel('$y$ [m]','interpreter','latex')
zlabel('$z$ [m]','interpreter','latex')
xlim([Xsl(1),Xsu(1)])
ylim([Xsl(2),Xsu(2)])
zlim([Xsl(3),Xsu(3)])
box on
grid on
view(-100,30)


% printing values associated with the generated trajectory
fprintf('m=%0.2f kg\n',mass)
fprintf('J=diag(%0.2f,%0.2f,%0.2f)\n',diag(J));
fprintf('kp=%0.2f\n',kp)
fprintf('kv=%0.2f\n',kv)
fprintf('kR=%0.2f\n',kR)
fprintf('kw=%0.2f\n',kw)
fprintf('c1=%0.3f\n',c1)
fprintf('c2=%0.3f\n',c2)
fprintf('psi_bar=%0.3f\n',psi_bar)
fprintf('alpha_psi=%0.3f\n',alpha_psi)
fprintf('V1_bar=%0.3f\n',V1_0)
fprintf('V2_bar=%0.3f\n',V2_0)

fprintf('uniform position error Lp=%0.2f\n',Lp)
fprintf('uniform velocity error Lv=%0.2f\n',Lv)
fprintf('uniform force bound  F_bound=%0.2f\n',F_bound)
fprintf('cpu time to compute uniform bounds is %0.2f seconds \n\n',t_cpu_bounds)


fprintf('value of alpha used in computing safe rectangles is %0.2f \n',alpha)
fprintf('maximum value of vertices is  N_v=%0.2f \n',Nv)
fprintf('value of parameter C_{sample} is %0.2f \n',C_sample)
fprintf('cpu time to compute safe tube is %0.2f seconds \n\n',t_c1)

fprintf('value of epsilon  is  %0.8f \n',epsilon)
fprintf('value of alpha_T  is  %0.2f \n',alpha_T)
fprintf('initial guess for T is %0.2f seconds \n',T0)
fprintf('Value of T after executing Alg2 is %0.2f seconds \n',T)
fprintf('Number of trajectory segments is %0.2f \n',N_seg)
fprintf('Number of control points per segment is %0.2f \n',N_pts)
fprintf('cpu time to compute trajectory using Alg. 2 is %0.2f seconds \n',t_c2)


fprintf('Velocity bound v_max is  (%0.2f,%0.2f,%0.2f) \n',vm)
fprintf('acceleration bound a_max is  (%0.2f,%0.2f,%0.2f) \n',am)


end


