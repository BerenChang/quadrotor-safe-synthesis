%%
load("m434_03rand_traj1.mat");
figure;
plot3(d.x(1,:), d.x(2,:), d.x(3,:), 'r');
hold on;
plot3(x(1,:), x(2,:), x(3,:), 'k');
axis equal;
xlabel('$x_1$', 'interpreter', 'latex');
ylabel('$x_2$', 'interpreter', 'latex');
zlabel('$x_3$', 'interpreter', 'latex');
set(gca, 'Box', 'on');
grid on;
set(gca, 'FontName', 'Times New Roman');
for i = 2:10
    traj_url = "m434_03rand_traj" + i + ".mat";
    load(traj_url);
    plot3(x(1,:), x(2,:), x(3,:), 'r');
end