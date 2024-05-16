function [y] = ClosestPoint(x,c,D)
% this function computes the  closest point to a point x within a box
% with center c and radius D.
%   Detailed explanation goes here
n=length(x);
y=x;

for i=1:n

    if abs(x(i)-c(i))<=D(i)
        
        y(i)=x(i);
        
    else 
        y(i)=c(i)+D(i)*sign(x(i)-c(i));
    end
    
end



end

