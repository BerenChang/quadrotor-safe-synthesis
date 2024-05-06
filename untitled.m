rand_pos = zeros(3,initial.init_n);
lyap1 = zeros(1,initial.init_n);
lyap2 = zeros(1,initial.init_n);
lyap3 = zeros(1,initial.init_n);

for i = 1:initial.init_n
    ex = (delta.x+0.02)*(2*rand(3,1)-1);
    ev = delta.v*(2*rand(3,1)-1);
    lyap1(i) = 0.5*k.x*dot(ex, ex);
    lyap2(i) = 0.5*0.033*dot(ev, ev);
    lyap3(i) = 0.04*dot(ex, ev);
    rand_pos(:,i) = ex+[1;1;1];
end
lyap = lyap1 + lyap2 + lyap3;

pos_in = rand_pos(:, lyap < 0.1);
pos_out = rand_pos(:, lyap >= 0.1);

lyap1_in = lyap1(lyap < 0.1);
lyap1_out = lyap1(lyap >= 0.1);

lyap2_in = lyap2(lyap < 0.1);
lyap2_out = lyap2(lyap >= 0.1);

lyap3_in = lyap3(lyap < 0.1);
lyap3_out = lyap3(lyap >= 0.1);

figure;
scatter3(pos_out(1,:), pos_out(2,:), pos_out(3,:), 'MarkerFaceColor', [0.8500 0.3250 0.0980]);
hold on;
scatter3(pos_in(1,:), pos_in(2,:), pos_in(3,:), 'MarkerFaceColor', [0.3010 0.7450 0.9330]);
legend("pos_out","pos_in")
hold off;

figure;
scatter(1:size(lyap1_out,2),lyap1_out, 'MarkerFaceColor', [0.8500 0.3250 0.0980]);
hold on;
scatter(1:size(lyap1_in,2),lyap1_in, 'MarkerFaceColor', [0.3010 0.7450 0.9330]);
legend("out","in");
hold off;

figure;
scatter(1:size(lyap2_out,2),lyap2_out, 'MarkerFaceColor', [0.8500 0.3250 0.0980]);
hold on;
scatter(1:size(lyap2_in,2),lyap2_in,'MarkerFaceColor', [0.3010 0.7450 0.9330]);
legend("out","in");
hold off;

figure;
scatter(1:size(lyap3_out,2),lyap3_out,'MarkerFaceColor', [0.8500 0.3250 0.0980]);
hold on;
scatter(1:size(lyap3_in,2),lyap3_in, 'MarkerFaceColor', [0.3010 0.7450 0.9330]);
legend("out","in");
hold off;


