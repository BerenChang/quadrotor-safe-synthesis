function [Radius] = Safety_Radius(Xc,Cs,Ds,CuArray,DuArray)
n=size(Xc,1);
Nu=size(CuArray,2);
Radius=-ones(n,1);


if min(Ds-abs(Xc-Cs))<0
   Radius=zeros(n,1);
 
else
    
     for ind_unsafe=1:Nu
         if  min(DuArray(:,ind_unsafe)-abs(Xc-CuArray(:,ind_unsafe)))>0  
             Radius=zeros(n,1);
             break; 
         end
     end
     
     
  if   sum(Radius)==-n   
  
   Distances=Inf(n,Nu+1);
   Distances(:,Nu+1)=(Ds-abs(Xc-Cs));
   for j=1:Nu
   [dist,ind_M]=inf_distance_with_index(Xc,CuArray(:,j),DuArray(:,j));
   Distances(ind_M,j)=dist;   
   end
   
   for j=1:n
   Radius(j)=min(Distances(j,:));
   end

 end
end


end
