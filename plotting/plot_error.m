function plot_error(t, e)

linetype = 'k';
linewidth = 1;
xlabel_ = 'time (s)';

figure;
plot_3x1(t, e.R, '', xlabel_, 'e_R', linetype, linewidth)
set(gca, 'FontName', 'Times New Roman');

figure;
plot_3x1(t, e.x, '', xlabel_, 'e_x', linetype, linewidth)
set(gca, 'FontName', 'Times New Roman');

figure;
plot_3x1(t, e.v, '', xlabel_, 'e_v', linetype, linewidth)
set(gca, 'FontName', 'Times New Roman');

end