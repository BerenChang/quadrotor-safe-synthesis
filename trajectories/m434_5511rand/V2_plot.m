%%
load("traj1.mat");
figure;
plot(V2(1:20));
hold on;
yline(V2_bar);
title("V2");
xlabel("t (0.01s)");
ylabel("V2");

for i = 2:10
    traj_url = "traj" + i + ".mat";
    load(traj_url);
    plot(V2(1:20));
end