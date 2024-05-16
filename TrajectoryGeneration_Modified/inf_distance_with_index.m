function [dist,ind] = inf_distance_with_index(x,c,D)
% this function estimates the  inf-distance between a point x and a box
% with center c and radius D.
%   Detailed explanation goes here
n=length(x);
y=x;

for i=1:n

    if abs(x(i)-c(i))<=D(i)
        
        y(i)=0;
        
    else 
        y(i)=abs(c(i)+D(i)*sign(x(i)-c(i))-x(i));
    end
    
end

[dist,ind]=max(y);

end

