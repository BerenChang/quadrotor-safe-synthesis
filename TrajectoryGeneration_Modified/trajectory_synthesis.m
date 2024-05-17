function generated_trajectory = trajectory_synthesis(annealing_output, anneal_options, param)
rng('default');
Lp = annealing_output.opt_bounds.Lp;
Lv = annealing_output.opt_bounds.Lv;
Lf = annealing_output.opt_bounds.Lf;
mass = param.m;
vm = anneal_options.vm;
am = anneal_options.am;
% system dimension
n=3;
% m=2;
alpha=0.9; %scaling factor for safe hyper-rectangles
Nv=400; % maximum number of vertices in RRT plans
C_sample=0.9; % parameter for RRT generation
N_pts=15; % control points per segment
T0=10; % initial time guess
epsilon=1e-6; % parameters for ALG.2
alpha_T=1.1; % parameters for ALG.2
delta_vector=[Lp;Lp;Lp]; % robustness margins
% safe set Xs
Xsl=[0;0;0];
Xsu=[5;5;5];
Cs=0.5*(Xsl+Xsu);
Ds=0.5*(Xsu-Xsl)-delta_vector;
% Gs=diag(Ds);
% Initial point
X0=[0.5;0.5;1];
% sX=size(X0);
% Target set Xt
Xtl=[4;4;4];
Xtu=[5;5;5];
Ct=0.5*(Xtl+Xtu);
Dt=0.5*(Xtu-Xtl)-delta_vector;
Gt=diag(Dt);
% Unsafe set Xu
XulArray=[4;5;0];
XuuArray=[4;5;3];
XulArray=[XulArray, [4.5;3.5;0]];
XuuArray=[XuuArray,[5;4;4.5]];
XulArray=[XulArray, [1.5;0;3]];
XuuArray=[XuuArray,[2.5;1;5]];
XulArray=[XulArray, [0;4;0]];
XuuArray=[XuuArray,[2;5;3]];
XulArray=[XulArray, [2;3;2]];
XuuArray=[XuuArray,[4;4;5]];
XulArray=[XulArray, [3;4;0]];
XuuArray=[XuuArray,[3.5;5;5]];
XulArray=[XulArray, [3.5;0;0.5]];
XuuArray=[XuuArray,[4;2.5;3]];
XulArray=[XulArray, [2.5;1;1.5]];
XuuArray=[XuuArray,[3;1.5;5]];
XulArray=[XulArray, [2.5;2.5;2]];
XuuArray=[XuuArray,[3;3;5]];
XulArray=[XulArray, [1;1;0]];
XuuArray=[XuuArray,[1.5;1.5;5]];
XulArray=[XulArray, [1;2;0]];
XuuArray=[XuuArray,[2;3;2]];
sXu=size(XulArray);
% Nu=sXu(2);
CuArray=0.5*(XulArray+XuuArray);
DuArray=0.5*(XuuArray-XulArray)+delta_vector;
tic;
%initializing the RRT Tree and associated tree structures
Tree=zeros(n,Nv);
Safety_Radius_Tree=zeros(n,Nv);
Nodes=ones(1,Nv);
Images=ones(1,Nv);
ind=1;
% ind_iter=0;
Tree(:,1)=X0; % adding initial point to the tree.
Safety_Radius_Tree(:,1)=Safety_Radius_M(Tree(:,1),Cs,Ds,CuArray,DuArray,alpha);
% dist_T=300;
while  ind<=Nv
    % ind
    % generating a random sample
    sample_safety=0;
    while sample_safety==0
        % generating a random sample
        if ind<=C_sample*Nv
            x_sample=(Xsl+delta_vector)+((Xsu-delta_vector)-(Xsl+delta_vector)).*rand(n,1);
        else
            x_sample=(Xtl+delta_vector)+((Xtu-delta_vector)-(Xtl+delta_vector)).*rand(n,1);
        end
        sample_safety=Safety_Check(x_sample,Cs,Ds,CuArray,DuArray);
    end
    dist=100*norm(Xsu-Xsl,inf);
    for ind_dist=1:ind
        dr=norm(x_sample-ClosestPoint(x_sample,Tree(:,ind_dist),Safety_Radius_Tree(:,ind_dist)),inf);
        if dr<dist
            ind_near=ind_dist;
            dist=dr;
        end
    end
    x_near=Tree(:,ind_near);
    R_x_near=Safety_Radius_Tree(:,ind_near);
    x_new_c=ClosestPoint(x_sample,x_near,R_x_near);
    ind=ind+1;
    Tree(:,ind)=x_new_c;
    Safety_Radius_Tree(:,ind)=Safety_Radius_M(Tree(:,ind),Cs,Ds,CuArray,DuArray,alpha);
    Images(ind-1)=ind;
    Nodes(ind-1)=ind_near;
    dist_T=norm(Gt\(Tree(:,ind)-Ct),inf);
    if dist_T<=1
        break;
    end
end
% if dist_T<=1
warning('Problem solved!')
G = digraph(Nodes,Images);
Path=shortestpath(G,1,ind);
M=length(Path);
X_Array=zeros(n,M);
R_Array=zeros(n,M);
% end
for k=1:M
    X_Array(:,k)=Tree(:,Path(k));
    if k<M
        Rs= Safety_Radius_M(X_Array(:,k),Cs,Ds,CuArray,DuArray,alpha);
    elseif k==M
        Rs=Dt-abs(X_Array(:,k)-Ct);
    end
    R_Array(:,k)=Rs;
end
t_c1=toc;
tic;
[Points_Array,T0,~,tau]=BezierControlPoints_IterativeLP(epsilon,alpha_T,X_Array,R_Array,N_pts,T0,vm,am,Lv,Lf,mass);
t_c2=toc;
generated_trajectory.alpha = alpha;
generated_trajectory.C_sample = C_sample;
generated_trajectory.Nv = Nv;
generated_trajectory.epsilon = epsilon;
generated_trajectory.alpha_T = alpha_T;
generated_trajectory.T0 = T0;
generated_trajectory.N_pts = N_pts;
generated_trajectory.X_Array = X_Array;
generated_trajectory.R_Array = R_Array;
generated_trajectory.Points_Array = Points_Array;
generated_trajectory.tau = tau;
generated_trajectory.XulArray = XulArray;
generated_trajectory.XuuArray = XuuArray;
generated_trajectory.Xtl = Xtl;
generated_trajectory.Xtu = Xtu;
generated_trajectory.Xsl = Xsl;
generated_trajectory.Xsu = Xsu;
generated_trajectory.t_c1 = t_c1;
generated_trajectory.t_c2 = t_c2;
end