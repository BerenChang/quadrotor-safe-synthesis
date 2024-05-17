function plot_initial_position_attitude(initial)

figure;
scatter3(initial.initial_points(1,:), ...
    initial.initial_points(2,:), ...
    initial.initial_points(3,:), ...
    'MarkerEdgeColor',[0.8500 0.3250 0.0980],...
    'MarkerFaceColor',[0.8500 0.3250 0.0980]);
hold on;

for i = 1:size(initial.initial_points,2)
    plotR = reshape(initial.initial_points(10:18,i), 3, 3);

    % axis-angle representation
    axang = rotm2axang(plotR);
    mArrow3(initial.initial_points(1:3,i)', initial.initial_points(1:3,i)' + axang(1:3)*axang(4));

    % rotation matrix representation
    % plotTransforms(initial.initial_points(1:3,i)',rotm2quat(plotR)*0.01);

end

legend('Initial points','interpreter','latex');
xlabel('$x$ [m]','interpreter','latex');
ylabel('$y$ [m]','interpreter','latex');
zlabel('$z$ [m]','interpreter','latex');

% xlim([0.85 1.15]);
% ylim([0.85 1.15]);
% zlim([0.85 1.15]);

% Adjust plot appearance
grid on;
box on;
set(gca,'ticklabelinterpreter','latex');
set(gca, 'FontSize', 12);
set(gca, 'LineWidth', 1.2);
set(gca, 'TickDir', 'out');
set(gca, 'TickLength', [0.02, 0.02]);
set(gca, 'XMinorTick', 'on');
set(gca, 'YMinorTick', 'on');
set(gca, 'ZMinorTick', 'on');
hold off;

% Set view
view(-100,30)

end