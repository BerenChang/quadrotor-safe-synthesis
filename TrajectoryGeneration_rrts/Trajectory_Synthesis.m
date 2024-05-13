clc
clear all
close all

rng default;

load("UniformBoundsData.mat")


% system dimension 
n=3;
m=2;


%scaling factor for safe hyper-rectangles
alpha=0.8;

% maximum number of iterations
N_iter=200;

% control points
N_pts=15;



% initial time guess
T0=10;


% maximum number of vertices in RRT plans
Nv=N_iter;


% robustness margins
delta_vector=[L_p;L_p;L_p];





% safe set Xs

Xsl=[0;0;0];

Xsu=[10;10;10];


Cs=0.5*(Xsl+Xsu);
Ds=0.5*(Xsu-Xsl)-delta_vector;
Gs=diag(Ds);


% Initial point

X0=[1;1;1];

sX=size(X0);

% system dimension


% Target set Xt 

Xtl=[8;8;8];
Xtu=[10;10;10];
Ct=0.5*(Xtl+Xtu);
Dt=0.5*(Xtu-Xtl)-delta_vector;
Gt=diag(Dt);


% Unsafe set Xu
XulArray=[5;5;0];
XuuArray=[6;6;10];

XulArray=[XulArray, [8;8;4]];
XuuArray=[XuuArray,[9;9;7]];

XulArray=[XulArray, [2;2;0]];
XuuArray=[XuuArray,[3;3;5]];

XulArray=[XulArray, [8;4;7]];
XuuArray=[XuuArray,[10;6;10]];

XulArray=[XulArray, [2;8;2]];
XuuArray=[XuuArray,[4;10;7]];

%XulArray=[XulArray, [6;8;4]];
%XuuArray=[XuuArray,[8;10;6]];

XulArray=[XulArray, [7;4;4]];
XuuArray=[XuuArray,[9;6;6]];

XulArray=[XulArray, [4;3;3]];
XuuArray=[XuuArray,[5;4;9]];

XulArray=[XulArray, [2;4;6]];
XuuArray=[XuuArray,[4;6;9]];

XulArray=[XulArray, [1;6;0]];
XuuArray=[XuuArray,[2;8;10]];

%XulArray=[XulArray, [4;1;0]];
%XuuArray=[XuuArray,[6;2;10]];
sXu=size(XulArray);
Nu=sXu(2);

CuArray=0.5*(XulArray+XuuArray);
DuArray=0.5*(XuuArray-XulArray)+delta_vector;

tic

%initializing the RRT Tree and associated tree structures
Tree=zeros(n,Nv);
Safety_Radius_Tree=zeros(n,Nv);
Nodes=ones(1,N_iter);
Images=ones(1,N_iter);

ind=1;
ind_iter=0;
Tree(:,1)=X0; % adding initial point to the tree.
Safety_Radius_Tree(:,1)=Safety_Radius_M(Tree(:,1),Cs,Ds,CuArray,DuArray,alpha);

dist_T=300;



while  ind<=Nv 
   ind 
    
 % generating a random sample 
   sample_safety=0;
while sample_safety==0
 % generating a random sample 
 if ind<=0.9*Nv
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



    
    
    
    




  if dist_T<=1
   warning('Problem solved!')   
   G = digraph(Nodes,Images);
  Path=shortestpath(G,1,ind);
  M=length(Path);
  X_Array=zeros(n,M);
  R_Array=zeros(n,M);

  
  
  
     for i=1:M
     end
  end
  
  
  
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
 [Points_Array,T0,T,tau]=BezierControlPoints_Refined(X_Array,R_Array,N_pts,T0,vm,am,L_v,L_f,mass);
 t_c2=toc;
  
  
 
 
 save("GeneratedTrajectoryData.mat","alpha","T0","N_pts","X_Array","R_Array","Points_Array","tau","XulArray","XuuArray","Xtl","Xtu","Xsl","Xsu","t_c1","t_c2");  

 
 
 

