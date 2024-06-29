function plot_lyapunov(t, lyap_V, Lu, crop)

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

crop_index = floor(size(t,1) / crop);

figure;
hold on;
yline(Lu, 'Color', [0.8, 0.2, 0.4], 'LineWidth', 1.5);

for i = 1:size(lyap_V, 1)
    plot(t(1:crop_index), lyap_V(i,1:crop_index), 'Color', colors(i, :), 'LineWidth', 1.5);
end

% title("Lyapunov");
% plot(V1);
% plot(V2);
% plot(V_bound);
% yline(uniform_V_bound);
legend("$\mathcal{L}_{u}^2(\overline{\mathcal{V}}_{1},\overline{\mathcal{V}}_{2})$",'interpreter','latex');

xlabel('$t$ [s]','interpreter','latex');
ylabel('$V$','interpreter','latex');

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

end