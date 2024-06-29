function plot_tube(plan_traj, real_traj, radius)
% plan_traj: 3*N matrix
% real_traj: n_traj*3*N matrix
% radius: the radius of safe tube

size_traj = size(real_traj);
n_traj = size_traj(1);
N = size_traj(3);
color = {'k','b',[0 0.5 0],'r',[0.8 0.9 0.9741],[0.8 0.98 0.9],[1 0.8 0.8],[0.7, 0.7 1]};

% Plot the line
figure;
plot3(plan_traj(1,:), plan_traj(2,:), plan_traj(3,:), 'blue', 'LineWidth', 1,'LineStyle','--');
hold on;

for i=1:n_traj
    plot3(squeeze(real_traj(i,1,:)), squeeze(real_traj(i,2,:)), squeeze(real_traj(i,3,:)), 'red', 'LineWidth', 1);
end

for i=1:N
   center = [plan_traj(1,i),plan_traj(2,i),plan_traj(3,i)];
   if i == 1 || norm(center-center_prev) > 0.003
       [X,Y,Z] = ellipsoid(center(1),center(2),center(3),radius,radius,radius);
       surf(X,Y,Z,'FaceColor',color{8},'FaceAlpha',0.05,'EdgeColor','none'); %'FaceLighting','flat'
   else
       aa = 1;
   end
   center_prev = center;
end

% Set labels and title
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Tube-shaped Region around 3D Line');

% Set grid and aspect ratio
grid on;
axis equal;

hold off;

% Set view
view(3);