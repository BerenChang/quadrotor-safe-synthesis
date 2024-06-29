function plot_initial_position_attitude(initial, view_angle)

figure;
scatter3(initial.initial_points(1,1:50), ...
    initial.initial_points(2,1:50), ...
    initial.initial_points(3,1:50), ...
    'MarkerEdgeColor',[0.8500 0.3250 0.0980],...
    'MarkerFaceColor',[0.8500 0.3250 0.0980]);
hold on;

for i = 1:50
    plotR = reshape(initial.initial_points(10:18,i), 3, 3);

    % axis-angle representation0
    % axang = rotm2axang(plotR);
    
    % plot b3
    mArrow3(initial.initial_points(1:3,i)', ...
        initial.initial_points(1:3,i)' + plotR(:,3)'*0.1, ...
        'color','black', ...
        'stemWidth', 0.002, ...
        'tipWidth', 0.005);

    % rotation matrix representation
    % plotTransforms(initial.initial_points(1:3,i)',rotm2quat(plotR)*0.01);

end

% legend('Initial points','interpreter','latex');
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
view(view_angle)

end