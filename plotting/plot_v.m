function plot_v(t, v_list, initial, vm, sample)

sample_order = 1:(sample-1):(length(t));

figure;

%% subplot 1
tiledlayout(3,1,'TileSpacing','tight','Padding','tight')
% subplot(3,1,1);
nexttile
hold on;
yline(vm(1), 'Color', [0.8, 0.2, 0.4], 'LineWidth', 1.5);
for i = 1:initial.init_n
    vi = abs(squeeze(v_list(i,1,:)));
    % plot(t(1:200), error_p(1:200), 'Color', [rand rand rand], 'LineWidth', 1.5);
    plot(t(sample_order), vi(sample_order), 'Color', [rand rand rand], 'LineWidth', 1.5);
end

ylim([0 1.1*vm(1)]);

% Add labels and title
xlim([0 t(length(t))]);
% xlabel('$t$ [s]','interpreter','latex');
ylabel('$|v_1(t)|$ [m/s]','interpreter','latex');
% title('Norm of position error vs time','interpreter','latex');

legend('${|v_{\max,1}|}$', 'interpreter','latex');

% Adjust plot appearance
box on;
% set(gca, 'FontName', 'Arial');
set(gca, 'FontSize', 12);
set(gca, 'LineWidth', 1.2);
set(gca, 'TickDir', 'out');
set(gca, 'TickLength', [0.02, 0.02]);
set(gca, 'XMinorTick', 'on');
set(gca, 'YMinorTick', 'on');
set(gca,'xtick',[])
set(gca,'TickLabelInterpreter','latex');

hold off;

%% subplot 2
% subplot(3,1,2);
nexttile
hold on;
yline(vm(2), 'Color', [0.8, 0.2, 0.4], 'LineWidth', 1.5);
for i = 1:initial.init_n
    vi = abs(squeeze(v_list(i,2,:)));
    % plot(t(1:200), error_p(1:200), 'Color', [rand rand rand], 'LineWidth', 1.5);
    plot(t(sample_order), vi(sample_order), 'Color', [rand rand rand], 'LineWidth', 1.5);
end

ylim([0 1.1*vm(2)]);

% Add labels and title
xlim([0 t(length(t))]);
% xlabel('$t$ [s]','interpreter','latex');
ylabel('$|v_2(t)|$ [m/s]','interpreter','latex');
% title('Norm of position error vs time','interpreter','latex');

legend('${|v_{\max,2}|}$', 'interpreter','latex');

% Adjust plot appearance
box on;
% set(gca, 'FontName', 'Arial');
set(gca, 'FontSize', 12);
set(gca, 'LineWidth', 1.2);
set(gca, 'TickDir', 'out');
set(gca, 'TickLength', [0.02, 0.02]);
set(gca, 'XMinorTick', 'on');
set(gca, 'YMinorTick', 'on');
set(gca,'xtick',[]);
set(gca,'TickLabelInterpreter','latex');

hold off;

%% subplot 3
% subplot(3,1,3);
nexttile
hold on;
yline(vm(3), 'Color', [0.8, 0.2, 0.4], 'LineWidth', 1.5);
for i = 1:initial.init_n
    vi = abs(squeeze(v_list(i,2,:)));
    % plot(t(1:200), error_p(1:200), 'Color', [rand rand rand], 'LineWidth', 1.5);
    plot(t(sample_order), vi(sample_order), 'Color', [rand rand rand], 'LineWidth', 1.5);
end

xlim([0 t(length(t))]);
ylim([0 1.1*vm(3)]);

% Add labels and title
xlabel('$t$ [s]','interpreter','latex');
ylabel('$|v_3(t)|$ [m/s]','interpreter','latex');
% title('Norm of position error vs time','interpreter','latex');

legend('${|v_{\max,3}|}$', 'interpreter','latex');

% Adjust plot appearance
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