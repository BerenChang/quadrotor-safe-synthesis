clc
clear all
close all

load("GeneratedTrajectoryData.mat");
load("UniformBoundsData.mat");

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

figure


% function DesiredTrajectory outputs p, pdot, pddot, and pdddot as 
% functions of time. Function DesiredTrajectory requires the arrays 
%Points_Array (control points), and tau (duration of each segment) 

for i=1:N
    [X(:,i),V(:,i),A(:,i),Je(:,i),S(:,i)]=DesiredTrajectory(t(i),Points_Array,tau); 
    
end


figure % plotting the map only
hold on
plotcube(transpose(Xtu-Xtl),transpose(Xtl),0.5,[0,0,1])
for i=1:Nu
plotcube(transpose(XuuArray(:,i)-XulArray(:,i)),transpose(XulArray(:,i)),0.5,[1 0 0]) 
end

set(gca,'fontsize',15)
set(gca,'ticklabelinterpreter','latex')
xlabel('$x$ [m]','interpreter','latex')
ylabel('$y$ [m]','interpreter','latex')
zlabel('$z$ [m]','interpreter','latex')
xlim([Xsl(1),Xsu(1)])
ylim([Xsl(2),Xsu(2)])
zlim([Xsl(3),Xsu(3)])
box on
view(-51,76)


figure %plotting the safety tube within the map
hold on

plotcube(transpose(Xtu-Xtl),transpose(Xtl),0.05,[0,0,1])
for i=1:Nu
plotcube(transpose(XuuArray(:,i)-XulArray(:,i)),transpose(XulArray(:,i)),0.05,[1 0 0]) 
end

for k=1:M
 plotcube(transpose(2*R_Array(:,k)),transpose(X_Array(:,k)-R_Array(:,k)),0.5,[0,1,1])
end
set(gca,'fontsize',15)
set(gca,'ticklabelinterpreter','latex')
xlabel('$x$ [m]','interpreter','latex')
ylabel('$y$ [m]','interpreter','latex')
zlabel('$z$ [m]','interpreter','latex')
xlim([Xsl(1),Xsu(1)])
ylim([Xsl(2),Xsu(2)])
zlim([Xsl(3),Xsu(3)])
view(-51,76)
box on



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
view(-51,76)


% printing values associated with the generated trajectory
fprintf('kp=%0.2f\n',kp)
fprintf('kv=%0.2f\n',kv)
fprintf('kR=%0.2f\n',kR)
fprintf('kw=%0.2f\n',kR)
fprintf('psi_bar=%0.2f\n',psi_1)
fprintf('V1_bar=%0.2f\n',V1_0)

fprintf('uniform position error Lp=%0.2f\n',L_p)
fprintf('uniform velocity error Lv=%0.2f\n',L_v)
fprintf('uniform force bound  F_bound=%0.2f\n',F_bound)
fprintf('Value of alpha used in computing safe rectangles is %0.2f \n',alpha)

fprintf('cpu time to compute safe tube is %0.2f seconds \n',t_c1)
fprintf('cpu time to compute trajectory using Alg. 2 is %0.2f seconds \n',t_c2)
fprintf('initial guess for T is %0.2f seconds \n',T0)
fprintf('Value of T after executing Alg2 is %0.2f seconds \n',T)
fprintf('Number of trajectory segments is %0.2f \n',N_seg)
fprintf('Number of control points per segment is %0.2f \n',N_pts)
fprintf('Velocity bound v_max is  (%0.2f,%0.2f,%0.2f) \n',vm)

% you will need to plot the velocity profiles of generated trajectories 
% showing they are bounded by vm.




