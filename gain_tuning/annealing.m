function annealing_output = annealing(anneal_options, param)
rng("default");

initial_solution = anneal_options.initial_solution; % Initial solution guess
lower_bound = anneal_options.lower_bound; % Lower bounds for variables
upper_bound = anneal_options.upper_bound; % Upper bounds for variables

total_runtime = 0;
runtime_n = anneal_options.runtime_n;

optimal.x = zeros(runtime_n, size(initial_solution,2));
optimal.fval = zeros(runtime_n, 1);

options = optimoptions(@simulannealbnd, 'MaxFunctionEvaluations', 50000);

f = @(x)position_bound(x, anneal_options, param);

% Call the Simulated Annealing function
for i = 1:runtime_n
    tic;
    [x, fval, ~, ~] = simulannealbnd(f, initial_solution, lower_bound, upper_bound, options);
    runtime = toc;
    total_runtime = total_runtime + runtime;

    optimal.x(i,:) = x;
    optimal.fval(i) = fval;
end

ave_runtime = total_runtime / runtime_n;
opt_fval = min(optimal.fval);
opt_k = optimal.x(find(optimal.fval == min(optimal.fval)),:);
opt_bounds = evaluate(opt_k, anneal_options, param);

annealing_output.opt_k = opt_k;
annealing_output.opt_bounds = opt_bounds;

% Display results
disp('Average runtime of optimization:');
disp(ave_runtime);
disp('Minimum value among those n trials:');
disp(opt_fval);
disp('Optimal gains:');
disp(opt_k);
disp('Optimal bounds and parameters: Lp, Lv, Lf, F_bound, c1, c2');
disp([opt_bounds.Lp, opt_bounds.Lv, opt_bounds.Lf, opt_bounds.F_bound, opt_bounds.c1, opt_bounds.c2]);

end
