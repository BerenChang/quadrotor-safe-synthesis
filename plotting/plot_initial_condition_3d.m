function plot_initial_condition_3d(param, initial)

psi_y = 0 : param.norm_eW_bound/100 : param.norm_eW_bound;
psi_z = 0 : param.V1_bar/100 : param.V1_bar;
[psi_Y, psi_Z] = meshgrid(psi_y, psi_z);
psi_X = param.psi_bar*ones(size(psi_Y));

eW_z = 0 : param.V1_bar/100 : param.V1_bar;
eW_x = 0 : param.psi_bar/100 : param.psi_bar;
[eW_X, eW_Z] = meshgrid(eW_x, eW_z);
eW_Y = param.norm_eW_bound*ones(size(eW_X));


V1_y = 0 : param.norm_eW_bound/100 : param.norm_eW_bound;
%V1_y = 0 : min(initial.eW0_norm_list) : param.norm_eW_bound;
V1_x = 0 : param.psi_bar/100 : param.psi_bar;
[V1_X, V1_Y] = meshgrid(V1_x, V1_y);
V1_Z = param.V1_bar*ones(size(V1_X));

figure;
scatter3(initial.psi_0_list, ...
        initial.eW0_norm_list, ...
        initial.V1_0_list, ...
        'MarkerEdgeColor',[0.8500 0.3250 0.0980],...
        'MarkerFaceColor',[0.8500 0.3250 0.0980]);
hold on;

surf(psi_X, psi_Y, psi_Z, ...
    'EdgeColor','none', ...
    'FaceColor',[0 0.4470 0.7410], ...
    'FaceAlpha',0.3);
surf(eW_X, eW_Y, eW_Z, ...
    'EdgeColor','none', ...
    'FaceColor',[0 0.4470 0.7410], ...
    'FaceAlpha',0.3);

surf(V1_X, V1_Y, V1_Z, ...
    'EdgeColor','none', ...
    'FaceColor',[0 0.4470 0.7410], ...
    'FaceAlpha',0.3);
yscale log;

xlabel("$\psi$", 'interpreter', 'latex');
ylabel("$e_\omega$", 'interpreter', 'latex');
zlabel("$V_1$", 'interpreter', 'latex');

grid on;
box on;
set(gca, 'FontSize', 12);
set(gca, 'LineWidth', 1.2);
set(gca, 'TickDir', 'out');
set(gca, 'TickLength', [0.02, 0.02]);
set(gca, 'XMinorTick', 'on');
set(gca, 'YMinorTick', 'on');
hold off;

end