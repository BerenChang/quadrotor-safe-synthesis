function [] = PlotBezier(Control_Points)
n_plot_points=100;
t=linspace(0,1,n_plot_points);
X=zeros(3,n_plot_points);
n=size(Control_Points,2)-1;
for j=1:n_plot_points
 X(:,j)= Bezier(n,t(j),Control_Points);  
end

plot3(X(1,:),X(2,:),X(3,:),'b','linewidth',2)
end


function y= Bezier(n,t,Control_Points)
  y = zeros(3,1);
  k=0;
  while k<=n
   y =y+(factorial(n)/(factorial(k)*factorial(n-k)) * (1-t)^(n-k) * t^(k))*Control_Points(:,k+1);
  k=k+1;
  end
end