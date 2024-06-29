function plot_initial_position(initial)

figure;
scatter3(initial.initial_points(1,:), ...
    initial.initial_points(2,:), ...
    initial.initial_points(3,:), ...
    'MarkerEdgeColor',[0.8500 0.3250 0.0980],...
    'MarkerFaceColor',[0.8500 0.3250 0.0980]);
hold on;
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