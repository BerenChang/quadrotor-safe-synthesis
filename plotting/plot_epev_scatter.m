function plot_epev_scatter(initial)
% Number of points
n = initial.init_n;

% Generate random points
x = initial.ep0_list;
y = initial.ev0_list;

% Create a new figure
figure;

% Hold the plot for multiple scatter calls
hold on;

% Scatter plot for points where a(i) is 1 (red)
scatter(x(initial.safe_list == 1), y(initial.safe_list == 1), 5, 'r', 'filled');

% Scatter plot for points where a(i) is not 1 (blue)
scatter(x(initial.safe_list ~= 1), y(initial.safe_list ~= 1), 5, 'b', 'filled');

% Set axis labels
xlabel('$\|e_p\|$ [m]','interpreter','latex');
ylabel('$\|e_v\|$ [m]','interpreter','latex');

% Set title
% title('2D Scatter Plot with Conditional Coloring');

% Add legend
legend('Safe', 'Unsafe');

% Set grid
grid on;

% Hold off to finish the plot
hold off;