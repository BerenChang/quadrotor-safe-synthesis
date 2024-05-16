function plot_rotm(initial)

figure;
hold on
for i = 1:size(initial.initial_points,2)
    plotR = reshape(initial.initial_points(10:18,i), 3, 3);
    % axang = rotm2axang(plotR);
    % mArrow3([0 0 0], axang(1:3)*axang(4));
    plotTransforms([0 0 0],rotm2quat(plotR));
end

set(gca,'fontsize',15);
set(gca,'ticklabelinterpreter','latex');
xlabel('$x$','interpreter','latex');
ylabel('$y$','interpreter','latex');
zlabel('$z$','interpreter','latex');

grid on
box on
view(170, 30)

end
