function [Points_Array,T0,T,tau] = BezierControlPoints_Refined(X_Array,R_Array,N_pts,T0,vm,am,L_v,L_f,m)

%Note: the variable A_Bound correspond to B in our paper



g=9.81;
e3=[0;0;1];


% scaling time 
alpha_T=1.05;


LPopts = optimoptions('linprog','Display','off','ConstraintTolerance',1e-9);

N_seg=size(X_Array,2)-1;


Dist=0;

for j=1:N_seg
    Dist=Dist+norm(X_Array(:,j+1)-X_Array(:,j));    
end




tau=zeros(1,N_seg);
rat=zeros(1,N_seg);
for j=1:N_seg
    rat(j)=norm(X_Array(:,j+1)-X_Array(:,j))/Dist;
    tau(j)=(norm(X_Array(:,j+1)-X_Array(:,j))/Dist)*T0;   
end





% This code conputes the Bezier control points for   based on given way points and associated 
%safety radii
% the vector to optimize is
%X=[b^{1}_{1};b^{1}_{2};...;b^{1}_{N_pts};......;b^{N_seg}_{1};b^{N_seg}_{2};...;b^{N_seg}_{N_pts}]
% size of X is 3*N_pts*N_seg;





% The p0 constraints imposes matching waypoint(Only First Point)
%b^{j}_{1}=X_Array(:,j) for j in [1;1]


Ap0=zeros(3*1,3*N_pts*N_seg);
Bp0=zeros(3*1,1);

for j=1%:N_seg
Ap0((j-1)*3+1:j*3,(j-1)*N_pts*3+1:(j-1)*N_pts*3+3)=eye(3,3);
Bp0((j-1)*3+1:j*3)=X_Array(:,j);
end





%The set of constraints v0 imposes a certain initial velocity (zero in this case)
Av0=zeros(3,3*N_pts*N_seg);
Bv0=[0;0;0];

Av0(1:3,1:3)=-eye(3,3);
Av0(1:3,4:6)=eye(3,3);




%The set of constraints a0 imposes a certain initial acceleration (zero in this case)
Aa0=zeros(3,3*N_pts*N_seg);
Ba0=[0;0;0];

Aa0(1:3,1:3)=eye(3,3);
Aa0(1:3,4:6)=-2*eye(3,3);
Aa0(1:3,7:9)=eye(3,3);


%The set of constraints j0 imposes a certain initial jerk (zero in this case)
Aj0=zeros(3,3*N_pts*N_seg);
Bj0=[0;0;0];

Aj0(1:3,1:3)=-eye(3,3);
Aj0(1:3,4:6)=3*eye(3,3);
Aj0(1:3,7:9)=-3*eye(3,3);
Aj0(1:3,10:12)=eye(3,3);



%The set of constraints s0 imposes a certain initial snap (zero in this case)
As0=zeros(3,3*N_pts*N_seg);
Bs0=[0;0;0];

As0(1:3,1:3)=eye(3,3);
As0(1:3,4:6)=-4*eye(3,3);
As0(1:3,7:9)=6*eye(3,3);
As0(1:3,10:12)=-4*eye(3,3);
As0(1:3,13:15)=eye(3,3);







% The pf constraints imposes bound on final waypoint
%b^{N_seg}_{N_pts} in X_Array(:,end)+[-R_Array(:,end),R_Array(:,end)] 

%upper bound 
Apf1=zeros(3,3*N_pts*N_seg);
Bpf1=X_Array(:,end)+R_Array(:,end);
Apf1(1:3,N_seg*N_pts*3-3+1:N_seg*N_pts*3)=eye(3,3);

%lower bound
Apf2=zeros(3,3*N_pts*N_seg);
Bpf2=-X_Array(:,end)+R_Array(:,end);
Apf2(1:3,N_seg*N_pts*3-3+1:N_seg*N_pts*3)=-eye(3,3);




%The  set of constraints vf imposes a certain final velocity (zero in this case)
Avf=zeros(3,3*N_pts*N_seg);
Bvf=[0;0;0];

Avf(1:3,3*N_pts*N_seg-5:3*N_pts*N_seg-3)=-eye(3,3);
Avf(1:3,3*N_pts*N_seg-2:end)=eye(3,3);



% The pc constraints imposes position continuity
%b^{j}_{N_pts}=b^{j+1}_{1} for j in [1;N_seg-1]
Apc=zeros(3*(N_seg-1),3*N_pts*N_seg);
Bpc=zeros(3*(N_seg-1),1);
for j=1:N_seg-1
Apc((j-1)*3+1:j*3,j*N_pts*3-2:j*N_pts*3)=eye(3,3); %b^{j}_{N_pts} 
Apc((j-1)*3+1:j*3,j*N_pts*3+1:j*N_pts*3+3)=-eye(3,3); % b^{j+1}_{1}
end



% The pb1 constraints imposes position bounds
%b^{j}_{i}<=X_Array(:,j)+R_Array(:,j) for j in [1;N_seg] and i in [1;N_pts]
Apb1=eye(3*N_pts*N_seg,3*N_pts*N_seg);
Bpb1=zeros(3*N_pts*N_seg,1);


for j=1:N_seg   

    V=zeros(3*N_pts,1);

    for k=1:N_pts
        V((k-1)*3+1:k*3)=X_Array(:,j)+R_Array(:,j);
    end
Bpb1((j-1)*N_pts*3+1:j*N_pts*3)=V;
end








% The pb2 constraints imposes position bounds
%-b^{j}_{i}<=-X_Array(:,j)+R_Array(:,j) for j in [1;N_seg] and i in [1;N_pts]
Apb2=-eye(3*N_pts*N_seg,3*N_pts*N_seg);
Bpb2=zeros(3*N_pts*N_seg,1);


for j=1:N_seg   
   V=zeros(3*N_pts,1);    
   for k=1:N_pts
       V((k-1)*3+1:k*3)=-X_Array(:,j)+R_Array(:,j);
   end
       Bpb2((j-1)*N_pts*3+1:j*N_pts*3)=V;
end





% The vb1 set of constraints imposes velocity bounds
%b^{j}_{i+1}-b^{j}_{i}<=(tau(j)/(N_pts-1))*(vm-L_v*ones(3,1)) for j in [1;N_seg] and i in [1;N_pts-1] 
 Avb1=[];
 for j=1:N_seg   
 
 
     for k=1:N_pts-1
         R=zeros(3,3*N_pts*N_seg);
         R(:,3*(j-1)*N_pts+(k-1)*3+1:3*(j-1)*N_pts+(k-1)*3+3)=-eye(3,3);
         R(:,3*(j-1)*N_pts+k*3+1:3*(j-1)*N_pts+k*3+3)=eye(3,3);
         Avb1=[Avb1;R];
     end
 end
 
% 
% 
 % The vb2 set of constraints imposes velocity bounds
 %-b^{j}_{i+1}+b^{j}_{i}<=(tau(j)/(N_pts-1))*(vm-L_v*ones(3,1)) for j in [1;N_seg] and i in [1;N_pts-1]
 Avb2=[];
 for j=1:N_seg   
 
 
     for k=1:N_pts-1
         R=zeros(3,3*N_pts*N_seg);
         R(:,3*(j-1)*N_pts+(k-1)*3+1:3*(j-1)*N_pts+(k-1)*3+3)=eye(3,3);
         R(:,3*(j-1)*N_pts+k*3+1:3*(j-1)*N_pts+k*3+3)=-eye(3,3);
         Avb2=[Avb2;R];
     end
 end



















% The ab1 constraints imposes aceeleration bounds
%b^{j}_{i+2}-2*b^{j}_{i+1}+b^{j}_{i}<=(tau(j)^2/((N_pts-1)*(N_pts-2)))*((A_Bound/m*sqrt(3))*ones(3,1)-g*e3)) for j in [1;N_seg] and i in [1;N_pts-2]
Aab1=[];
for j=1:N_seg   

    for k=1:N_pts-2
        R=zeros(3,3*N_pts*N_seg);
        R(:,3*(j-1)*N_pts+(k-1)*3+1:3*(j-1)*N_pts+(k-1)*3+3)=eye(3,3);
        R(:,3*(j-1)*N_pts+k*3+1:3*(j-1)*N_pts+k*3+3)=-2*eye(3,3);
        R(:,3*(j-1)*N_pts+(k+1)*3+1:3*(j-1)*N_pts+(k+1)*3+3)=eye(3,3);
        Aab1=[Aab1;R];
    end
end


% The ab2 constraints imposes acceleration bounds
%-b^{j}_{i+2}+2*b^{j}_{i+1}-b^{j}_{i}<=(tau(j)^2/((N_pts-1)*(N_pts-2)))*min((A_Bound/(sqrt(3)*m))*ones(3,1)+g*e3),[inf;inf;g-L_f/m-eps]) for j in [1;N_seg] and i in [1;N_pts-2]

Aab2=[];
for j=1:N_seg   
    for k=1:N_pts-2
        R=zeros(3,3*N_pts*N_seg);
        R(:,3*(j-1)*N_pts+(k-1)*3+1:3*(j-1)*N_pts+(k-1)*3+3)=-eye(3,3);
        R(:,3*(j-1)*N_pts+k*3+1:3*(j-1)*N_pts+k*3+3)=2*eye(3,3);
        R(:,3*(j-1)*N_pts+(k+1)*3+1:3*(j-1)*N_pts+(k+1)*3+3)=-eye(3,3);
        Aab2=[Aab2;R];
    end
end








ind_opt=0;
X_opt=[];

while isempty(X_opt)==1

if ind_opt>0    
tau=alpha_T*tau;
end








% The vc constraints imposes velocity continuity
%(1/rat(j))*(b^{j}_{N_pts}-b^{j}_{N_pts-1})=(1/rat(j+1))*b^{j+1}_{2}-b^{j+1}_{1} for j in [1;N_seg-1]
Avc=zeros(3*(N_seg-1),3*N_pts*N_seg);
Bvc=zeros(3*(N_seg-1),1);
for j=1:N_seg-1
Avc((j-1)*3+1:j*3,j*N_pts*3-5:j*N_pts*3-3)=-(1/rat(j))*eye(3,3); %b^{j}_{N_pts-1}
Avc((j-1)*3+1:j*3,j*N_pts*3-2:j*N_pts*3)=(1/rat(j))*eye(3,3); %b^{j}_{N_pts}
Avc((j-1)*3+1:j*3,j*N_pts*3+1:j*N_pts*3+3)=(1/rat(j+1))*eye(3,3); % b^{j+1}_{1}
Avc((j-1)*3+1:j*3,j*N_pts*3+4:j*N_pts*3+6)=-(1/rat(j+1))*eye(3,3); % b^{j+1}_{2}
end


% The ac constraints imposes acceleration continuity
%(1/rat(j)^2)*(b^{j}_{N_pts}-2*b^{j}_{N_pts-1}+b^{j}_{N_pts-2})=(1/rat(j+1)^2)*(b^{j+1}_{3}-2*b^{j+1}_{2}+b^{j+1}_{1}) for j in [1;N_seg-1]
Aac=zeros(3*(N_seg-1),3*N_pts*N_seg);
Bac=zeros(3*(N_seg-1),1);
for j=1:N_seg-1
Aac((j-1)*3+1:j*3,j*N_pts*3-2:j*N_pts*3)=(1/rat(j)^2)*eye(3,3); %b^{j}_{N_pts}
Aac((j-1)*3+1:j*3,j*N_pts*3-5:j*N_pts*3-3)=-2*(1/rat(j)^2)*eye(3,3); %b^{j}_{N_pts-1}
Aac((j-1)*3+1:j*3,j*N_pts*3-8:j*N_pts*3-6)=(1/rat(j)^2)*eye(3,3); %b^{j}_{N_pts-2}
Aac((j-1)*3+1:j*3,j*N_pts*3+1:j*N_pts*3+3)=-(1/rat(j+1)^2)*eye(3,3); % b^{j+1}_{1}
Aac((j-1)*3+1:j*3,j*N_pts*3+4:j*N_pts*3+6)=2*(1/rat(j+1)^2)*eye(3,3); % b^{j+1}_{2}
Aac((j-1)*3+1:j*3,j*N_pts*3+7:j*N_pts*3+9)=-(1/rat(j+1)^2)*eye(3,3); % b^{j+1}_{3}
end



% The jc set of constraints imposes jerk continuity
%(1/rat(j)^3)*(b^{j}_{N_pts}-3*b^{j}_{N_pts-1}+3*b^{j}_{N_pts-2}-b^{j}_{N_pts-3})=(1/rat(j+1)^3)*(b^{j+1}_{4}-3*b^{j+1}_{3}+3*b^{j+1}_{2}-b^{j+1}_{1}) for j in [1;N_seg-1]
Ajc=zeros(3*(N_seg-1),3*N_pts*N_seg);
Bjc=zeros(3*(N_seg-1),1);
for j=1:N_seg-1
Ajc((j-1)*3+1:j*3,j*N_pts*3-2:j*N_pts*3)=(1/rat(j)^3)*eye(3,3); %b^{j}_{N_pts}
Ajc((j-1)*3+1:j*3,j*N_pts*3-5:j*N_pts*3-3)=-3*(1/rat(j)^3)*eye(3,3); %b^{j}_{N_pts-1}
Ajc((j-1)*3+1:j*3,j*N_pts*3-8:j*N_pts*3-6)=3*(1/rat(j)^3)*eye(3,3); %b^{j}_{N_pts-2}
Ajc((j-1)*3+1:j*3,j*N_pts*3-11:j*N_pts*3-9)=-1*(1/rat(j)^3)*eye(3,3); %b^{j}_{N_pts-3}
Ajc((j-1)*3+1:j*3,j*N_pts*3+1:j*N_pts*3+3)=(1/rat(j+1)^3)*eye(3,3); % b^{j+1}_{1}
Ajc((j-1)*3+1:j*3,j*N_pts*3+4:j*N_pts*3+6)=-3*(1/rat(j+1)^3)*eye(3,3); % b^{j+1}_{2}
Ajc((j-1)*3+1:j*3,j*N_pts*3+7:j*N_pts*3+9)=3*(1/rat(j+1)^3)*eye(3,3); % b^{j+1}_{3}
Ajc((j-1)*3+1:j*3,j*N_pts*3+10:j*N_pts*3+12)=-(1/rat(j+1)^3)*eye(3,3); % b^{j+1}_{4}
end




% The sc set of constraints imposes snap continuity
%(1/rat(j)^4)*(b^{j}_{N_pts}-4*b^{j}_{N_pts-1}+6*b^{j}_{N_pts-2}-4*b^{j}_{N_pts-3}+b^{j}_{N_pts-4})=(1/rat(j+1)^4)*(b^{j+1}_{5}-4*b^{j+1}_{4}+6*b^{j+1}_{3}-4*b^{j+1}_{2}+4*b^{j+1}_{1}) for j in [1;N_seg-1]
Asc=zeros(3*(N_seg-1),3*N_pts*N_seg);
Bsc=zeros(3*(N_seg-1),1);
for j=1:N_seg-1
Asc((j-1)*3+1:j*3,j*N_pts*3-2:j*N_pts*3)=(1/rat(j)^4)*eye(3,3); %b^{j}_{N_pts}
Asc((j-1)*3+1:j*3,j*N_pts*3-5:j*N_pts*3-3)=-4*(1/rat(j)^4)*eye(3,3); %b^{j}_{N_pts-1}
Asc((j-1)*3+1:j*3,j*N_pts*3-8:j*N_pts*3-6)=6*(1/rat(j)^4)*eye(3,3); %b^{j}_{N_pts-2}
Asc((j-1)*3+1:j*3,j*N_pts*3-11:j*N_pts*3-9)=-4*(1/rat(j)^4)*eye(3,3); %b^{j}_{N_pts-3}
Asc((j-1)*3+1:j*3,j*N_pts*3-14:j*N_pts*3-12)=1*(1/rat(j)^4)*eye(3,3); %b^{j}_{N_pts-4}

Asc((j-1)*3+1:j*3,j*N_pts*3+1:j*N_pts*3+3)=-(1/rat(j+1)^4)*eye(3,3); % b^{j+1}_{1}
Asc((j-1)*3+1:j*3,j*N_pts*3+4:j*N_pts*3+6)=4*(1/rat(j+1)^4)*eye(3,3); % b^{j+1}_{2}
Asc((j-1)*3+1:j*3,j*N_pts*3+7:j*N_pts*3+9)=-6*(1/rat(j+1)^4)*eye(3,3); % b^{j+1}_{3}
Asc((j-1)*3+1:j*3,j*N_pts*3+10:j*N_pts*3+12)=4*(1/rat(j+1)^4)*eye(3,3); % b^{j+1}_{4}
Asc((j-1)*3+1:j*3,j*N_pts*3+13:j*N_pts*3+15)=-(1/rat(j+1)^4)*eye(3,3); % b^{j+1}_{5}

end







% The vb1 set of constraints imposes velocity bounds
%b^{j}_{i+1}-b^{j}_{i}<=(tau(j)/(N_pts-1))*(vm-L_v*ones(3,1)) for j in [1;N_seg] and i in [1;N_pts-1]
Bvb1=zeros(3*(N_pts-1)*N_seg,1);
 
 
 for j=1:N_seg   
     V=zeros(3*(N_pts-1),1);
 
     for k=1:N_pts-1
         V((k-1)*3+1:k*3)=(tau(j)/(N_pts-1))*(vm-L_v*ones(3,1));    
     end
 Bvb1((j-1)*(N_pts-1)*3+1:j*(N_pts-1)*3)=V;
 end



 % The vb2 set of constraints imposes velocity bounds
 %-b^{j}_{i+1}+b^{j}_{i}<=(tau(j)/(N_pts-1))*(vm-L_v*ones(3,1)) for j in [1;N_seg] and i in [1;N_pts-1]
 Bvb2=zeros(3*(N_pts-1)*N_seg,1);
 for j=1:N_seg   
     V=zeros(3*(N_pts-1),1);
% 
     for k=1:N_pts-1
         V((k-1)*3+1:k*3)=(tau(j)/(N_pts-1))*(vm-L_v*ones(3,1));    
     end
 Bvb2((j-1)*(N_pts-1)*3+1:j*(N_pts-1)*3)=V;
 end



% The ab1 constraints imposes aceeleration bounds
%b^{j}_{i+2}-2*b^{j}_{i+1}+b^{j}_{i}<=(tau(j)^2/((N_pts-1)*(N_pts-2)))*(((A_Bound/m*sqrt(3))*ones(3,1)-g*e3)) for j in [1;N_seg] and i in [1;N_pts-2]
Bab1=zeros(3*(N_pts-2)*N_seg,1);

for j=1:N_seg   
    V=zeros(3*(N_pts-2),1);

    for k=1:N_pts-2
        V((k-1)*3+1:k*3)=((tau(j)^2)/((N_pts-1)*(N_pts-2)))*(am-g*e3);    
    end
Bab1((j-1)*(N_pts-2)*3+1:j*(N_pts-2)*3)=V;
end





% The ab2 constraints imposes acceleration bounds
%-b^{j}_{i+2}+2*b^{j}_{i+1}-b^{j}_{i}<=(tau(j)^2/((N_pts-1)*(N_pts-2)))*(min((A_Bound/m*sqrt(3))*ones(3,1)+g*e3),[inf;inf;g-L_f/m-eps]) for j in [1;N_seg] and i in [1;N_pts-2]
Bab2=zeros(3*(N_pts-2)*N_seg,1);


for j=1:N_seg   
    V=zeros(3*(N_pts-2),1);

    for k=1:N_pts-2
        V((k-1)*3+1:k*3)=((tau(j)^2)/((N_pts-1)*(N_pts-2)))*(min([am+g*e3,[inf;inf;g-(L_f/m)-eps]],[],2));   
    end
Bab2((j-1)*(N_pts-2)*3+1:j*(N_pts-2)*3)=V;
end















f=zeros(1,3*N_pts*N_seg);
f(1)=1;

Aeq=[Ap0;Av0;Aa0;Aj0;Apc;Avc;Aac;Ajc;Asc;Avf];
Beq=[Bp0;Bv0;Ba0;Bj0;Bpc;Bvc;Bac;Bjc;Bsc;Bvf];

%size(Aeq)
%size(Beq)

%Avb1=[];
%Avb2=[];
%Bvb1=[];
%Bvb2=[];

A=[Apf1;Apf2;Apb1;Apb2;Avb1;Avb2;Aab1;Aab2];
B=[Bpf1;Bpf2;Bpb1;Bpb2;Bvb1;Bvb2;Bab1;Bab2];


X_opt=linprog(f,A,B,Aeq,Beq,[],[],LPopts);

ind_opt=ind_opt+1

end



if isempty(X_opt)==0
Points_Array=zeros(3,N_pts,N_seg);
 
for j=1:N_seg
    Points_Array(:,:,j)=reshape(X_opt((j-1)*N_pts*3+1:j*N_pts*3),3,N_pts);
 end
else
Points_Array=[];
warning('No Solution!!!')
end

T=sum(tau);
