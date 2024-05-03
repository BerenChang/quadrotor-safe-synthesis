function desired_bezier = DesiredTrajectory(t,Points_Array,tau)

T=sum(tau);
N = 0;
CPoints = 0;
T_P = 0;
TC = 0;

if t<=T

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
    p_2dot= BezierAcceleration(n,t,Control_Points,T_p,Tc);
    p_3dot= BezierJerk(n,t,Control_Points,T_p,Tc);
    p_4dot= BezierSnap(n,t,Control_Points,T_p,Tc);

    N = n;
    CPoints = Control_Points;
    T_P = T_p;
    TC = Tc;
else
    p= Points_Array(:,7,12);
    p_dot= zeros(3,1);
    p_2dot= zeros(3,1);
    p_3dot= zeros(3,1);
    p_4dot= zeros(3,1);
end

desired_bezier.x = p;
desired_bezier.v = p_dot;
desired_bezier.x_2dot = p_2dot;
desired_bezier.x_3dot = p_3dot;
desired_bezier.x_4dot = p_4dot;

w=0; % pi/2*t;
desired_bezier.w = w;
desired_bezier.b1 = [cos(w * t), sin(w * t), 0]';
desired_bezier.b1_dot = w * [-sin(w * t), cos(w * t), 0]';
desired_bezier.b1_2dot = w^2 * [-cos(w * t), -sin(w * t), 0]';

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

