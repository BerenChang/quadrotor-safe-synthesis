% Define the Simulated Annealing parameters
initial_solution = [100, 1, 100, 1, .5, 0.35]; % Initial solution guess
lower_bound = [0, 0, 0, 0, 0, 0]; % Lower bounds for variables
upper_bound = [100, 100, 100, 100, 1, 1]; % Upper bounds for variables
max_iterations = 1000; % Maximum number of iterations

% Call the Simulated Annealing function
[x, fval, exitflag, output] = simulannealbnd(@position_bound, initial_solution, lower_bound, upper_bound);

% Display results
disp('Optimal solution:');
disp(x);
disp('Minimum value of the objective function:');
disp(fval);
disp('Exit flag:');
disp(exitflag);
disp('Output:');
disp(output);

