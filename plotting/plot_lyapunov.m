function plot_lyapunov(V1, V2, V, V_bound, uniform_V_bound)

figure;
hold on;
plot(V);
title("Lyapunov");
plot(V1);
plot(V2);
plot(V_bound);
yline(uniform_V_bound);
legend("V", "V1", "V2", "V_{bound}", "V_{max}");
hold off;

end