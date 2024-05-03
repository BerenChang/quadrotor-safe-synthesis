%%
load("traj1.mat");
figure;
plot(M(2,:));
hold on;
title("control M(1)");
xlabel("t (0.1ms)");
ylabel("Mx");

for i = 2:10
    traj_url = "traj" + i + ".mat";
    load(traj_url);
    plot(M(2,:));
end