%%
load("traj1.mat");
figure;
plot(V2(1:1000));
hold on;
yline(V2_bar);
title("V2");
xlabel("t (0.1ms)");
ylabel("V2");

for i = 2:10
    traj_url = "traj" + i + ".mat";
    load(traj_url);
    plot(V2(1:1000));
end