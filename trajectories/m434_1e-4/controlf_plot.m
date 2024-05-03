%%
load("traj1.mat");
figure;
plot(f);
hold on;
title("control f");
xlabel("t (0.1ms)");
ylabel("f");

for i = 2:10
    traj_url = "traj" + i + ".mat";
    load(traj_url);
    plot(f);
end