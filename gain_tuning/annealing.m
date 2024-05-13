% Define the Simulated Annealing parameters
% initial_solution = [100, 1, 100, 1, .5, 0.35]; % Initial solution guess
rng("default");
initial_solution = [10, 10, 10, 10, 0.5, 0.5]; % Initial solution guess

lower_bound = [0, 0, 0, 0, 0, 0]; % Lower bounds for variables
upper_bound = [100, 100, 100, 100, 1, 1]; % Upper bounds for variables

options = optimoptions(@simulannealbnd, 'MaxFunctionEvaluations', 50000);

total_runtime = 0;
runtime_n = 100;

optimal.x = zeros(runtime_n, size(initial_solution,2));
optimal.fval = zeros(runtime_n, 1);

% Call the Simulated Annealing function
for i = 1:runtime_n
    tic;
    [x, fval, exitflag, output] = simulannealbnd(@position_bound, initial_solution, lower_bound, upper_bound, options);
    runtime = toc;
    total_runtime = total_runtime + runtime;

    optimal.x(i,:) = x;
    optimal.fval(i) = fval;
end

ave_runtime = total_runtime / runtime_n;

% Display results
disp('Optimal solution:');
disp(x);
disp('Minimum value of the objective function:');
disp(fval);
disp('Exit flag:');
disp(exitflag);
disp('Output:');
disp(output);
disp('Average runtime:');
disp(ave_runtime);

disp('Minimum value among those n trials:');
disp(min(optimal.fval));
disp('Optimal gains:');
disp(optimal.x(find(optimal.fval == min(optimal.fval)),:))

