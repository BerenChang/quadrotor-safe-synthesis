function plot_traj(plan_traj, real_traj, Lp, X, body_axis, zoom_in)
% plan_traj: 3*N matrix
% real_traj: n_traj*3*N matrix
% Lp: the radius of safe tube
% body_axis: Boolean. show attitude
% X: full desired states
% zoom_in: number treated as Boolean. plot the zoom-in traj to show convergence
%          0 for no zoom_in image. positive integer for first 1/zoom_in
%          section of trajectories

figure;
ax1 = axes('position', [0.13 0.13 0.85 0.85]);

size_traj = size(real_traj);
n_traj = size_traj(1);
N = size_traj(3);
color = {'k','b',[0 0.5 0],'r',[0.8 0.9 0.9741],[0.8 0.98 0.9],[1 0.8 0.8],[0.7, 0.7 1]};

view(-100,30)
plot3(plan_traj(1,:), plan_traj(2,:), plan_traj(3,:), 'black', 'LineWidth', 0.5,'LineStyle','--');

hold on;

if body_axis
    sample_freq = 1000;
    for i = 1:floor(N/sample_freq)
        plotR = reshape(X(sample_freq*i,10:18), 3, 3);
        plotTransforms(plan_traj(:,sample_freq*i)',rotm2quat(plotR));
    end
end

for i=1:n_traj
    plot3(squeeze(real_traj(i,1,:)), squeeze(real_traj(i,2,:)), squeeze(real_traj(i,3,:)), 'blue', 'LineWidth', 0.5);
end

for i=1:N
   center = [plan_traj(1,i),plan_traj(2,i),plan_traj(3,i)];
   % if i == 1 || norm(center-center_prev) > 0.00000003
       [X,Y,Z] = ellipsoid(center(1),center(2),center(3),Lp,Lp,Lp);
       surf(X,Y,Z,'FaceColor',color{8},'FaceAlpha',0.01,'EdgeColor','none'); %'FaceLighting','flat'
   % else
   %     aa = 1;
   % end
   % center_prev = center;
end

% plotting the obstacles

% Target set Xt 
Xtl=[8;8;8]; 
Xtu=[10;10;10];        

% Unsafe set Xu
XulArray=[5;5;0];
XuuArray=[6;6;10];

XulArray=[XulArray, [8;8;4]];
XuuArray=[XuuArray,[9;9;7]];

XulArray=[XulArray, [2;2;0]];
XuuArray=[XuuArray,[3;3;5]];

XulArray=[XulArray, [8;4;7]];
XuuArray=[XuuArray,[10;6;10]];

XulArray=[XulArray, [2;8;2]];
XuuArray=[XuuArray,[4;10;7]];

%XulArray=[XulArray, [6;8;4]];
%XuuArray=[XuuArray,[8;10;6]];

XulArray=[XulArray, [7;4;4]];
XuuArray=[XuuArray,[9;6;6]];

XulArray=[XulArray, [4;3;3]];
XuuArray=[XuuArray,[5;4;9]];

XulArray=[XulArray, [2;4;6]];
XuuArray=[XuuArray,[4;6;9]];

XulArray=[XulArray, [1;6;0]];
XuuArray=[XuuArray,[2;8;10]];

%XulArray=[XulArray, [4;1;0]];
%XuuArray=[XuuArray,[6;2;10]];
sXu=size(XulArray);
Nu=sXu(2);

plotcube(transpose(Xtu-Xtl),transpose(Xtl),0.3,[0,0,1]);

for i=1:Nu
    plotcube(transpose(XuuArray(:,i)-XulArray(:,i)),transpose(XulArray(:,i)),0.5,[1 0 0]);
end

% Set labels and title
set(gca,'fontsize',15);
set(gca,'ticklabelinterpreter','latex');
xlabel('$x$ [m]','interpreter','latex');
ylabel('$y$ [m]','interpreter','latex');
zlabel('$z$ [m]','interpreter','latex');
box on;
grid on;

view(-100,30)

if zoom_in
    ax2 = axes('position', [0.65 0.6 0.3 0.3]);
    crop_index = floor(size_traj(3)/zoom_in);
    plot3(ax2, plan_traj(1,1:crop_index), ...
        plan_traj(2,1:crop_index), ...
        plan_traj(3,1:crop_index), ...
        'black', 'LineWidth', 1,'LineStyle','--');
    hold on;

    for i=1:n_traj
        plot3(ax2, squeeze(real_traj(i,1,1:crop_index)), ...
            squeeze(real_traj(i,2,1:crop_index)), ...
            squeeze(real_traj(i,3,1:crop_index)), 'blue', 'LineWidth', 0.5);
    end

    for i=1:crop_index
        center = [plan_traj(1,i),plan_traj(2,i),plan_traj(3,i)];
        [X,Y,Z] = ellipsoid(center(1),center(2),center(3),Lp,Lp,Lp);
        surf(ax2, X,Y,Z,'FaceColor',color{8},'FaceAlpha',0.005,'EdgeColor','none'); %'FaceLighting','flat'
    end

    box on;
    grid on;
    hold off;
    view(-100,30)
end

annotation('rectangle', [0.85 0.2 0.1 0.1], 'Color', 'black')
annotation('arrow', [0.88 0.85], [0.3 0.55])

end