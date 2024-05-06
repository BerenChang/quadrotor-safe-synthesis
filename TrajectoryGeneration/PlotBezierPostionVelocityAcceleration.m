function [] = PlotBezierPostionVelocityAcceleration(Points_Array,tau,vm,am,L_v,L_f,mass)
figure
N_seg=length(tau);
n_plot_points=200;
T=zeros(1,N_seg);
T(1)=tau(1);

for j=2:N_seg 
 T(j)=T(j-1)+tau(j);   
end

for j=1:N_seg
    
    if j==1
       T_p=0;
    else
       T_p=T(j-1); 
    end
    
    
t=linspace(T_p,T(j),n_plot_points);    
Control_Points=Points_Array(:,:,j);
X=zeros(3,n_plot_points);
V=zeros(3,n_plot_points);
A=zeros(3,n_plot_points);
J=zeros(3,n_plot_points);
S=zeros(3,n_plot_points);
n=size(Control_Points,2)-1;
    for i=1:n_plot_points
        X(:,i)= BezierPosition(n,t(i),Control_Points,T_p,T(j));  
        V(:,i)= BezierVelocity(n,t(i),Control_Points,T_p,T(j));  
        A(:,i)= BezierAcceleration(n,t(i),Control_Points,T_p,T(j));
        J(:,i)= BezierJerk(n,t(i),Control_Points,T_p,T(j));
        S(:,i)= BezierSnap(n,t(i),Control_Points,T_p,T(j));

    end

subplot(5,1,1)    
plot(t,X(1,:),'b','linewidth',2)
hold on
plot(t,X(2,:),'r','linewidth',2)
plot(t,X(3,:),'g','linewidth',2)
grid on
subplot(5,1,2)    
plot(t,V(1,:),'b','linewidth',2)
hold on
plot(t,V(2,:),'r','linewidth',2)
plot(t,V(3,:),'g','linewidth',2)
grid on

subplot(5,1,3)    
plot(t,A(1,:),'b','linewidth',2)
hold on
plot(t,A(2,:),'r','linewidth',2)
plot(t,A(3,:),'g','linewidth',2)
grid on


subplot(5,1,4)    
plot(t,J(1,:),'b','linewidth',2)
hold on
plot(t,J(2,:),'r','linewidth',2)
plot(t,J(3,:),'g','linewidth',2)
grid on


subplot(5,1,5)    
plot(t,S(1,:),'b','linewidth',2)
hold on
plot(t,S(2,:),'r','linewidth',2)
plot(t,S(3,:),'g','linewidth',2)
grid on



end
g=9.81;
subplot(5,1,2) 
yline(vm(1)-L_v,':k','linewidth',3)
yline(-(vm(1)-L_v),':k','linewidth',3)



subplot(5,1,3) 
yline(am(1),':r','linewidth',3)
yline(am(3)-g,':g','linewidth',3)
yline(-am(1),':r','linewidth',3)
yline(-min([am(3)+g,g-L_f/mass]),':g','linewidth',3)




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



