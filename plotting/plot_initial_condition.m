function plot_initial_condition(t, param, ep_list, initial)

figure;
subplot(2,2,1);
% Plot each curve with different colors and line styles
% Generate distinguishable colors
colors = [0, 0.4470, 0.7410;     % blue
          0.8500, 0.3250, 0.0980; % orange
          0.9290, 0.6940, 0.1250]; % dark green 
trajectoryIndex = 1:initial.init_n;

hold on;
yline(param.psi_bar, 'Color', [0.8, 0.2, 0.4], 'LineWidth', 3);
scatter(trajectoryIndex, initial.psi_0_list, 'Color', colors(1, :), 'LineWidth', 1.5);
legend('$\overline{\Psi}$','interpreter','latex');
xlabel('Trajectory Number','interpreter','latex');
ylabel('$\psi(0)$','interpreter','latex');
ylim([0.0001 0.0105]);
% Adjust plot appearance
grid on;
box on;
set(gca, 'FontSize', 12);
set(gca, 'LineWidth', 1.2);
set(gca, 'TickDir', 'out');
set(gca, 'TickLength', [0.02, 0.02]);
set(gca, 'XMinorTick', 'on');
set(gca, 'YMinorTick', 'on');
hold off;

% figure;
subplot(2,2,2);
hold on;
yline(param.norm_eW_bound, 'Color', [0.8, 0.2, 0.4], 'LineWidth', 3);
scatter(trajectoryIndex, initial.eW0_norm_list, 'Color', colors(2, :), 'LineWidth', 1.5);
legend('$\frac{2 k_R}{\lambda_{\max}(J)}(1-\overline{\Psi})$','interpreter','latex');
xlabel('Trajectory Number','interpreter','latex');
ylabel('$\|{e_\omega}(0)\|^2$','interpreter','latex');
yscale log
% Adjust plot appearance
grid on;
box on;
set(gca, 'FontSize', 12);
set(gca, 'LineWidth', 1.2);
set(gca, 'TickDir', 'out');
set(gca, 'TickLength', [0.02, 0.02]);
set(gca, 'XMinorTick', 'on');
set(gca, 'YMinorTick', 'on');
hold off;

% figure;
subplot(2,2,3);
hold on;
yline(param.V1_bar, 'Color', [0.8, 0.2, 0.4], 'LineWidth', 3);
scatter(trajectoryIndex, initial.V1_0_list, 'Color', colors(3, :), 'LineWidth', 1.5);
legend('$\overline{\mathcal{V}}_{1}$','interpreter','latex');
xlabel('Trajectory Number','interpreter','latex');
ylabel('$V_1(0)$','interpreter','latex');
ylim([0.01 0.105]);
% Adjust plot appearance
grid on;
box on;
set(gca, 'FontSize', 12);
set(gca, 'LineWidth', 1.2);
set(gca, 'TickDir', 'out');
set(gca, 'TickLength', [0.02, 0.02]);
set(gca, 'XMinorTick', 'on');
set(gca, 'YMinorTick', 'on');
hold off;

subplot(2,2,4);
hold on;

% Plot each curve with different colors and line styles
% Generate distinguishable colors
colors = [0, 0.4470, 0.7410;     % blue
          0.8500, 0.3250, 0.0980; % orange
          0.9290, 0.6940, 0.1250; % yellow
          0.4940, 0.1840, 0.5560; % purple
          0.4660, 0.6740, 0.1880; % green
          0.3010, 0.7450, 0.9330; % light blue
          0.6350, 0.0780, 0.1840; % dark red
          0.8500, 0.3250, 0.0980; % orange (again)
          0.75, 0.75, 0;           % gold
          0, 0.5, 0];              % dark green 

yline(initial.Lp, 'Color', [0.8, 0.2, 0.4], 'LineWidth', 1.5);
for i = 1:initial.init_n
    error_p = squeeze(ep_list(i,:));
    plot(t(1:200), error_p(1:200), 'Color', colors(i, :), 'LineWidth', 1.5);
end

% Add labels and title
xlabel('$t$ (second)','interpreter','latex');
ylabel('$\|e_p\|$ (meter)','interpreter','latex');
% title('Norm of position error vs time','interpreter','latex');

% Add legend
legend('Theoretical bound');

% Adjust plot appearance
grid on;
box on;
% set(gca, 'FontName', 'Arial');
set(gca, 'FontSize', 12);
set(gca, 'LineWidth', 1.2);
set(gca, 'TickDir', 'out');
set(gca, 'TickLength', [0.02, 0.02]);
set(gca, 'XMinorTick', 'on');
set(gca, 'YMinorTick', 'on');
set(gca,'TickLabelInterpreter','latex');

hold off;

end