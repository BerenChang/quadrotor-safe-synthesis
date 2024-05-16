function [p,p_dot,p_ddot,p_dddot,p_ddddot] = DesiredTrajectoryAndDerivatives(t,Points_Array,tau)


T=sum(tau);

if t>T
   p=[];
   p_dot=[];
   p_ddot=[];
   p_dddot=[];
   p_ddddot=[];
    
 
else


N_seg=length(tau);
T=zeros(1,N_seg);


for j=1:N_seg 
 
 if j==1
    T(j)=tau(j); 
 else
    T(j)=T(j-1)+tau(j);   
 end
 
 if t<=T(j) && t>=T(j)-tau(j)
 
     ind=j;
     
     break;
 end
 
end

 
    
Control_Points=Points_Array(:,:,ind);
n=size(Control_Points,2)-1;
Tc=T(ind);
T_p=Tc-tau(ind);
        p= BezierPosition(n,t,Control_Points,T_p,Tc);  
        p_dot= BezierVelocity(n,t,Control_Points,T_p,Tc);  
        p_ddot= BezierAcceleration(n,t,Control_Points,T_p,Tc);
        p_dddot= BezierJerk(n,t,Control_Points,T_p,Tc);
        p_ddddot=BezierSnap(n,t,Control_Points,T_p,Tc);
end   

end




function y= BezierPosition(n,t,Control_Points,Tp,Tc)
  
  y = zeros(3,1);
  k=0;
  tt=(t-Tp)/(Tc-Tp);
  while k<=n
   y =y+(factorial(n)/(factorial(k)*factorial(n-k)) * (1-tt)^(n-k) * tt^(k))*Control_Points(:,k+1);
  k=k+1;
  end
end






function y= BezierVelocity(n,t,Control_Points,Tp,Tc)
  y = zeros(3,1);
  k=0;
  tt=(t-Tp)/(Tc-Tp);
  delta=Tc-Tp;
  coef=n*(1/delta);
  while k<=n-1
   y =y+(factorial(n-1)/(factorial(k)*factorial((n-1)-k)) * (1-tt)^((n-1)-k) * tt^(k))*(Control_Points(:,k+2)-Control_Points(:,k+1));
  k=k+1;
  end
y=coef*y;  
end


function y= BezierAcceleration(n,t,Control_Points,Tp,Tc)
  y = zeros(3,1);
  k=0;
  tt=(t-Tp)/(Tc-Tp);
  delta=Tc-Tp;
  coef=n*(n-1)*((1/delta)^2);
  while k<=n-2
   y =y+(factorial(n-2)/(factorial(k)*factorial((n-2)-k)) * (1-tt)^((n-2)-k) * tt^(k))*(Control_Points(:,k+3)-2*Control_Points(:,k+2)+Control_Points(:,k+1));
  k=k+1;
  end
  y=coef*y;
end



function y= BezierJerk(n,t,Control_Points,Tp,Tc)
  y = zeros(3,1);
  k=0;
  tt=(t-Tp)/(Tc-Tp);
  delta=Tc-Tp;
  coef=n*(n-1)*(n-2)*((1/delta)^3);
  while k<=n-3
   y =y+(factorial(n-3)/(factorial(k)*factorial((n-3)-k)) * (1-tt)^((n-3)-k) * tt^(k))*(Control_Points(:,k+4)-3*Control_Points(:,k+3)+3*Control_Points(:,k+2)-Control_Points(:,k+1));
  k=k+1;
  end
  y=coef*y;
end


function y= BezierSnap(n,t,Control_Points,Tp,Tc)
  y = zeros(3,1);
  k=0;
  tt=(t-Tp)/(Tc-Tp);
  delta=Tc-Tp;
  coef=n*(n-1)*(n-2)*(n-3)*((1/delta)^4);
  while k<=n-4
   y =y+(factorial(n-4)/(factorial(k)*factorial((n-4)-k)) * (1-tt)^((n-4)-k) * tt^(k))*(Control_Points(:,k+5)-4*Control_Points(:,k+4)+6*Control_Points(:,k+3)-4*Control_Points(:,k+2)+Control_Points(:,k+1));
  k=k+1;
  end
  y=coef*y;
end




