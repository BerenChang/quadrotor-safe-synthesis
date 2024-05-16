function [safetycheck] = Safety_Check(Xc,Cs,Ds,CuArray,DuArray)
Nu=size(CuArray,2);
safetycheck=1;
if min(Ds-abs(Xc-Cs))<0   
               safetycheck=0;
else
   for ind_unsafe=1:Nu
        if  min(DuArray(:,ind_unsafe)-abs(Xc-CuArray(:,ind_unsafe)))>=0   
            safetycheck=0;
            break;
        end
   end
end

