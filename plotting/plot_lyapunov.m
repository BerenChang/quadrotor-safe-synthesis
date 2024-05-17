function plot_lyapunov(t, lyap_V, Lu)

crop_index = floor(size(t,1) / 100);

figure;
hold on;
yline(Lu);

for i = 1:size(lyap_V, 1)
    plot(t(1:crop_index), lyap_V(i,1:crop_index));
end
% title("Lyapunov");
% plot(V1);
% plot(V2);
% plot(V_bound);
% yline(uniform_V_bound);
% legend("V", "V1", "V2", "V_{bound}", "V_{max}");

hold off;

end