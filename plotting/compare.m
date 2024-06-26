% Plot each curve with different colors and line styles
% Generate distinguishable colors
colors = [...
    0 0.4470 0.7410;...  % Blue
    0.8500 0.3250 0.0980;...  % Orange
    0.9290 0.6940 0.1250;...  % Yellow
    0.4940 0.1840 0.5560;...  % Purple
    0.4660 0.6740 0.1880;...  % Green
    0.3010 0.7450 0.9330;...  % Light Blue
    0.6350 0.0780 0.1840;...  % Red
    0 0 0;...                % Black
    0.75 0.75 0.75;...       % Gray
    1 0 1;...                % Magenta
    1 1 0;...                % Cyan
    0 1 0;...                % Lime
    1 0 0;...                % Red
    0 0 1;...                % Blue
    0 1 1;...                % Aqua
    1 0.5 0;...              % Orange
    0.5 0 0;...              % Maroon
    0.5 0 0.5;...            % Purple
    0 0.5 0;...              % Olive
    0.5 0.5 0.5];            % Gray

figure;
hold on;

yline(V(1)/min(eig(M11)), 'Color', [0.4, 0.2, 0.4], 'LineWidth', 1.5);
yline(initial.Lp, 'Color', [0.8, 0.2, 0.4], 'LineWidth', 1.5);

for i = 1:initial.init_n
    error_p = squeeze(ep_list(i,:));
    plot(t(1:200), error_p(1:200), 'Color', colors(i, :), 'LineWidth', 1.5);
end

% Add labels and title
xlabel('$t$ (second)','interpreter','latex');
ylabel('$\|e_p\|$ (meter)','interpreter','latex');
title('Norm of position error vs time','interpreter','latex');

% Add legend
legend('Bound1','Bound2');

% Adjust plot appearance
yscale log;
grid on;
box on;
set(gca, 'FontName', 'Arial');
set(gca, 'FontSize', 12);
set(gca, 'LineWidth', 1.2);
set(gca, 'TickDir', 'out');
set(gca, 'TickLength', [0.02, 0.02]);
set(gca, 'XMinorTick', 'on');
set(gca, 'YMinorTick', 'on');
set(gca,'TickLabelInterpreter','latex');

hold off;


