function plot_error_norm(t, e)

figure;
plot(t, vecnorm(e.x));
title("ep");
xlabel('time (s)','interpreter','latex');
ylabel('$\|e_p\|$','interpreter','latex');

figure;
plot(t, vecnorm(e.v));
title("ev");
xlabel('time (s)','interpreter','latex');
ylabel('$\|e_v\|$','interpreter','latex');

figure;
plot(t, vecnorm(e.R));
title("eR");
xlabel('time (s)','interpreter','latex');
ylabel('$\|e_R\|$','interpreter','latex');

figure;
plot(t, vecnorm(e.W));
title("eW");
xlabel('time (s)','interpreter','latex');
ylabel('$\|e_\omega\|$','interpreter','latex');
end