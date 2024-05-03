%%
load("traj1.mat");
figure;
plot(vecnorm(e.v));
hold on;
title("Velocity error");
xlabel("t (0.1ms)");
ylabel("||e_v||");

for i = 2:10
    traj_url = "traj" + i + ".mat";
    load(traj_url);
    plot(vecnorm(e.v));
end