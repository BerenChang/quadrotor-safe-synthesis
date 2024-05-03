%%
load("traj1.mat");
figure;
plot(log(vecnorm(e.x)));
hold on;
yline(Lp);
title("Position error");
xlabel("t (0.1ms)");
ylabel("log(||e_p||)");

for i = 2:10
    traj_url = "traj" + i + ".mat";
    load(traj_url);
    plot(log(vecnorm(e.x)));
end