function [Points_Array,T0,T,tau] = BezierControlPoints(X_Array,R_Array,N_pts,T0,a_max)

% the variable a_max correspond to B in our paper
% CrazyFly Quadrotor parameter
m=0.033;

g=9.81;
e3=[0;0;1];


LPopts = optimoptions('linprog','Display','off','ConstraintTolerance',1e-9);

N_seg=size(X_Array,2)-1;


Dist=0;

for j=1:N_seg
Dist=Dist+norm(X_Array(:,j+1)-X_Array(:,j));    
end




tau=zeros(1,N_seg);
rat=zeros(1,N_seg);
for j=1:N_seg
rat(j)=(norm(X_Array(:,j+1)-X_Array(:,j))/Dist);
    tau(j)=(norm(X_Array(:,j+1)-X_Array(:,j))/Dist)*T0;   
end





% This code conputes the Bezier control points for   based on given way points and associated 
%safety radii
% the vector to optimize is
%X=[b^{1}_{1};b^{1}_{2};...;b^{1}_{N_pts};......;b^{N_seg}_{1};b^{N_seg}_{2};...;b^{N_seg}_{N_pts}]
% size of X is 3*N_pts*N_seg;





% The first set of constraints imposes matching waypoint
%b^{j}_{1}=X_Array(:,j) for j in [1;N_seg]


A1=zeros(3*N_seg,3*N_pts*N_seg);
B1=zeros(3*N_seg,1);

for j=1%:N_seg
A1((j-1)*3+1:j*3,(j-1)*N_pts*3+1:(j-1)*N_pts*3+3)=eye(3,3);
B1((j-1)*3+1:j*3)=X_Array(:,j);
end

% The second set of constraints imposes matching final waypoint
%b^{N_seg}_{N_pts}=X_Array(:,end) 
A2=zeros(3,3*N_pts*N_seg);
B2=X_Array(:,end);

A2(1:3,N_seg*N_pts*3-3+1:N_seg*N_pts*3)=eye(3,3);

%A1=[];
%A2=[];
%B1=[];
%B2=[];


% The third set of constraints imposes position continuity
%b^{j}_{N_pts}=b^{j+1}_{1} for j in [1;N_seg-1]
A3=zeros(3*(N_seg-1),3*N_pts*N_seg);
B3=zeros(3*(N_seg-1),1);
for j=1:N_seg-1
A3((j-1)*3+1:j*3,j*N_pts*3-2:j*N_pts*3)=eye(3,3); %b^{j}_{N_pts} 
A3((j-1)*3+1:j*3,j*N_pts*3+1:j*N_pts*3+3)=-eye(3,3); % b^{j+1}_{1}
end



% The sixth set of constraints imposes position bounds
%b^{j}_{i}<=X_Array(:,j)+R_Array(:,j) for j in [1;N_seg] and i in [1;N_pts]
A6=eye(3*N_pts*N_seg,3*N_pts*N_seg);
B6=zeros(3*N_pts*N_seg,1);


for j=1:N_seg   

    V=zeros(3*N_pts,1);

    for k=1:N_pts
        V((k-1)*3+1:k*3)=X_Array(:,j)+R_Array(:,j);
    end
B6((j-1)*N_pts*3+1:j*N_pts*3)=V;
end








% The 7th set of constraints imposes position bounds
%-b^{j}_{i}<=-X_Array(:,j)+R_Array(:,j) for j in [1;N_seg] and i in [1;N_pts]
A7=-eye(3*N_pts*N_seg,3*N_pts*N_seg);
B7=zeros(3*N_pts*N_seg,1);


for j=1:N_seg   
   V=zeros(3*N_pts,1);    
   for k=1:N_pts
       V((k-1)*3+1:k*3)=-X_Array(:,j)+R_Array(:,j);
   end
       B7((j-1)*N_pts*3+1:j*N_pts*3)=V;
end



ind_opt=0;
X_opt=[];

while isempty(X_opt)==1

if ind_opt>0    
tau=2*tau;
end








% The fourth set of constraints imposes velocity continuity
%(1/rat(j))*(b^{j}_{N_pts}-b^{j}_{N_pts-1})=(1/rat(j+1))*b^{j+1}_{2}-b^{j+1}_{1} for j in [1;N_seg-1]
A4=zeros(3*(N_seg-1),3*N_pts*N_seg);
B4=zeros(3*(N_seg-1),1);
for j=1:N_seg-1
A4((j-1)*3+1:j*3,j*N_pts*3-5:j*N_pts*3-3)=-(1/rat(j))*eye(3,3); %b^{j}_{N_pts-1}
A4((j-1)*3+1:j*3,j*N_pts*3-2:j*N_pts*3)=(1/rat(j))*eye(3,3); %b^{j}_{N_pts}
A4((j-1)*3+1:j*3,j*N_pts*3+1:j*N_pts*3+3)=(1/rat(j+1))*eye(3,3); % b^{j+1}_{1}
A4((j-1)*3+1:j*3,j*N_pts*3+4:j*N_pts*3+6)=-(1/rat(j+1))*eye(3,3); % b^{j+1}_{2}
end


% The fifth set of constraints imposes acceleration continuity
%(1/rat(j)^2)*(b^{j}_{N_pts}-2*b^{j}_{N_pts-1}+b^{j}_{N_pts-2})=(1/rat(j+1)^2)*(b^{j+1}_{3}-2*b^{j+1}_{2}+b^{j+1}_{1}) for j in [1;N_seg-1]
A5=zeros(3*(N_seg-1),3*N_pts*N_seg);
B5=zeros(3*(N_seg-1),1);
for j=1:N_seg-1
A5((j-1)*3+1:j*3,j*N_pts*3-2:j*N_pts*3)=(1/rat(j)^2)*eye(3,3); %b^{j}_{N_pts}
A5((j-1)*3+1:j*3,j*N_pts*3-5:j*N_pts*3-3)=-2*(1/rat(j)^2)*eye(3,3); %b^{j}_{N_pts-1}
A5((j-1)*3+1:j*3,j*N_pts*3-8:j*N_pts*3-6)=(1/rat(j)^2)*eye(3,3); %b^{j}_{N_pts-2}
A5((j-1)*3+1:j*3,j*N_pts*3+1:j*N_pts*3+3)=-(1/rat(j+1)^2)*eye(3,3); % b^{j+1}_{1}
A5((j-1)*3+1:j*3,j*N_pts*3+4:j*N_pts*3+6)=2*(1/rat(j+1)^2)*eye(3,3); % b^{j+1}_{2}
A5((j-1)*3+1:j*3,j*N_pts*3+7:j*N_pts*3+9)=-(1/rat(j+1)^2)*eye(3,3); % b^{j+1}_{3}
end



% The jB set of constraints imposes jerk continuity
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






















% The 8th set of constraints imposes velocity bounds
%b^{j}_{i+1}-b^{j}_{i}<=(tau(j)/(N_pts-1))*v_max*ones(3,1) for j in [1;N_seg] and i in [1;N_pts-1]
% B8=zeros(3*(N_pts-1)*N_seg,1);
% 
% 
% for j=1:N_seg   
%     V=zeros(3*(N_pts-1),1);
% 
%     for k=1:N_pts-1
%         V((k-1)*3+1:k*3)=(tau(j)/(N_pts-1))*v_max*ones(3,1);    
%     end
% B8((j-1)*(N_pts-1)*3+1:j*(N_pts-1)*3)=V;
% end
% 
% %A8=zeros(3,3*N_pts*N_seg);
% A8=[];
% for j=1:N_seg   
% 
% 
%     for k=1:N_pts-1
%         R=zeros(3,3*N_pts*N_seg);
%         R(:,3*(j-1)*N_pts+(k-1)*3+1:3*(j-1)*N_pts+(k-1)*3+3)=-eye(3,3);
%         R(:,3*(j-1)*N_pts+k*3+1:3*(j-1)*N_pts+k*3+3)=eye(3,3);
%         A8=[A8;R];
%     end
% end
% 
% 
% 
% % The 9th set of constraints imposes velocity bounds
% %-b^{j}_{i+1}+b^{j}_{i}<=(tau(j)/(N_pts-1))*v_max*ones(3,1) for j in [1;N_seg] and i in [1;N_pts-1]
% B9=zeros(3*(N_pts-1)*N_seg,1);
% for j=1:N_seg   
%     V=zeros(3*(N_pts-1),1);
% 
%     for k=1:N_pts-1
%         V((k-1)*3+1:k*3)=(tau(j)/(N_pts-1))*v_max*ones(3,1);    
%     end
% B9((j-1)*(N_pts-1)*3+1:j*(N_pts-1)*3)=V;
% end
% 
% 
% A9=[];
% for j=1:N_seg   
% 
% 
%     for k=1:N_pts-1
%         R=zeros(3,3*N_pts*N_seg);
%         R(:,3*(j-1)*N_pts+(k-1)*3+1:3*(j-1)*N_pts+(k-1)*3+3)=eye(3,3);
%         R(:,3*(j-1)*N_pts+k*3+1:3*(j-1)*N_pts+k*3+3)=-eye(3,3);
%         A9=[A9;R];
%     end
% end

A8=[];
B8=[];
A9=[];
B9=[];


% The 10th set of constraints imposes aceeleration bounds
%b^{j}_{i+2}-2*b^{j}_{i+1}+b^{j}_{i}<=(tau(j)^2/((N_pts-1)*(N_pts-2)))*((a_max/m*sqrt(3))*ones(3,1)-g*e3) for j in [1;N_seg] and i in [1;N_pts-2]
B10=zeros(3*(N_pts-2)*N_seg,1);


for j=1:N_seg   
    V=zeros(3*(N_pts-2),1);

    for k=1:N_pts-2
        V((k-1)*3+1:k*3)=((tau(j)^2)/((N_pts-1)*(N_pts-2)))*((a_max/(sqrt(3)*m))*ones(3,1)-g*e3);    
    end
B10((j-1)*(N_pts-2)*3+1:j*(N_pts-2)*3)=V;
end

%A10=zeros(3,3*N_pts*N_seg);
A10=[];
for j=1:N_seg   


    for k=1:N_pts-2
        R=zeros(3,3*N_pts*N_seg);
        R(:,3*(j-1)*N_pts+(k-1)*3+1:3*(j-1)*N_pts+(k-1)*3+3)=eye(3,3);
        R(:,3*(j-1)*N_pts+k*3+1:3*(j-1)*N_pts+k*3+3)=-2*eye(3,3);
        R(:,3*(j-1)*N_pts+(k+1)*3+1:3*(j-1)*N_pts+(k+1)*3+3)=eye(3,3);
        A10=[A10;R];
    end
end



% The 11th set of constraints imposes acceleration bounds
%-b^{j}_{i+2}+2*b^{j}_{i+1}-b^{j}_{i}<=(tau(j)^2/((N_pts-1)*(N_pts-2)))*((a_max/(sqrt(3)*m))*ones(3,1)+g*e3) for j in [1;N_seg] and i in [1;N_pts-2]
B11=zeros(3*(N_pts-2)*N_seg,1);


for j=1:N_seg   
    V=zeros(3*(N_pts-2),1);

    for k=1:N_pts-2
        V((k-1)*3+1:k*3)=((tau(j)^2)/((N_pts-1)*(N_pts-2)))*((a_max/(sqrt(3)*m))*ones(3,1)+g*e3);    
    end
B11((j-1)*(N_pts-2)*3+1:j*(N_pts-2)*3)=V;
end

%A11=zeros(3,3*N_pts*N_seg);
A11=[];
for j=1:N_seg   
    for k=1:N_pts-2
        R=zeros(3,3*N_pts*N_seg);
        R(:,3*(j-1)*N_pts+(k-1)*3+1:3*(j-1)*N_pts+(k-1)*3+3)=-eye(3,3);
        R(:,3*(j-1)*N_pts+k*3+1:3*(j-1)*N_pts+k*3+3)=2*eye(3,3);
        R(:,3*(j-1)*N_pts+(k+1)*3+1:3*(j-1)*N_pts+(k+1)*3+3)=-eye(3,3);
        A11=[A11;R];
    end
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


%The set of constraints j0 imposes a certain initial acceleration (zero in this case)
Aj0=zeros(3,3*N_pts*N_seg);
Bj0=[0;0;0];

Aj0(1:3,1:3)=-eye(3,3);
Aj0(1:3,4:6)=3*eye(3,3);
Aj0(1:3,7:9)=-3*eye(3,3);
Aj0(1:3,10:12)=eye(3,3);





%The  set of constraints vf imposes a certain final velocity (zero in this case)
Avf=zeros(3,3*N_pts*N_seg);
Bvf=[0;0;0];

Avf(1:3,3*N_pts*N_seg-5:3*N_pts*N_seg-3)=-eye(3,3);
Avf(1:3,3*N_pts*N_seg-2:end)=eye(3,3);







f=zeros(1,3*N_pts*N_seg);
f(1)=1;

Aeq=[Av0;Aa0;Aj0;A1;A2;A3;A4;A5;Avf;Ajc];
Beq=[Bv0;Ba0;Bj0;B1;B2;B3;B4;B5;Bvf;Bjc];

%size(Aeq)
%size(Beq)


A=[A6;A7;A8;A9;A10;A11];
B=[B6;B7;B8;B9;B10;B11];


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
