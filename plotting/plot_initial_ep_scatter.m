function plot_initial_ep_scatter(initial)
% Number of points
n = initial.init_n;

% Generate random points
x = initial.ep0_list(1,:);
y = initial.ep0_list(2,:);
z = initial.ep0_list(3,:);
% Create a new figure
figure;

% Scatter plot for points where a(i) is 1 (red)
scatter3(x(initial.safe_list == 1), y(initial.safe_list == 1), z(initial.safe_list == 1), 5, 'r', 'filled', 'MarkerFaceAlpha', 0.1);

hold on;

% Scatter plot for points where a(i) is not 1 (blue)
scatter3(x(initial.safe_list ~= 1), y(initial.safe_list ~= 1), z(initial.safe_list ~= 1), 0.5, 'b', 'filled', 'MarkerFaceAlpha', 0.1);

% Set axis labels
xlabel('${e_p}_1(0)$ [m]','interpreter','latex', 'FontSize', 20);
ylabel('${e_p}_2(0)$ [m]','interpreter','latex', 'FontSize', 20);
zlabel('${e_p}_3(0)$ [m]','interpreter','latex', 'FontSize', 20);

xlim([-0.21 0.21]);
ylim([-0.21 0.21]);
zlim([-0.21 0.21]);

% Set title
% title('2D Scatter Plot with Conditional Coloring');

% Add legend
% legend('Safe', 'Unsafe');

% Set grid
grid on;

% Hold off to finish the plot
hold off;



%% cross section
cross_n = 2e4;

cross_safe_idx=find(initial.safe_list == 1 & y > -0.01 & y < 0.01);
cross_unsafe_idx=find(initial.safe_list ~= 1 & y > -0.01 & y < 0.01);

safe_frac = length(cross_safe_idx) / (length(cross_safe_idx) + length(cross_unsafe_idx))
unsafe_frac = length(cross_unsafe_idx) / (length(cross_safe_idx) + length(cross_unsafe_idx))

x_cross_safe = x(cross_safe_idx);
z_cross_safe = z(cross_safe_idx);
x_cross_unsafe = x(cross_unsafe_idx);
z_cross_unsafe = z(cross_unsafe_idx);

% Create a new figure
figure;

% Scatter plot for points where a(i) is 1 (red)
scatter(x_cross_safe(1:floor(cross_n*safe_frac)), z_cross_safe(1:floor(cross_n*safe_frac)), 1.5, 'r', 'filled');

hold on;

% Scatter plot for points where a(i) is not 1 (blue)
scatter(x_cross_unsafe(1:floor(cross_n*unsafe_frac)), z_cross_unsafe(1:floor(cross_n*unsafe_frac)), 1.5, 'b', 'filled');

% Set axis labels
xlabel('${e_p}_1(0)$ [m]','interpreter','latex', 'FontSize', 20);
% ylabel('${e_p}_2$ [m]','interpreter','latex');
ylabel('${e_p}_3(0)$ [m]','interpreter','latex', 'FontSize', 20);

xlim([-0.21 0.21]);
ylim([-0.21 0.21]);

% Set title
% title('2D Scatter Plot with Conditional Coloring');

% Add legend
% legend('Safe', 'Unsafe');

% Set grid
% grid on;

% Hold off to finish the plot
hold off;

end