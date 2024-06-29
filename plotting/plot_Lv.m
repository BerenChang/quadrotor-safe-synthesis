function plot_Lv(t, ev_list, initial)

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

figure;
hold on;
yline(initial.Lv, 'Color', [0.8, 0.2, 0.4], 'LineWidth', 1.5);
for i = 1:initial.init_n
    vi = squeeze(ev_list(i,:));
    % plot(t(1:200), error_p(1:200), 'Color', colors(i, :), 'LineWidth', 1.5);
    plot(t, vi, 'Color', colors(i, :), 'LineWidth', 1.5);
end


% Add labels and title
xlabel('$t$ (second)','interpreter','latex');
ylabel('$\|e_v\|$ (meter/second)','interpreter','latex');
% title('Norm of position error vs time','interpreter','latex');

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
% set(gca, 'YScale', 'log');

hold off;

end