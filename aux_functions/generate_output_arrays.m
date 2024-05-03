function [error, desired, R, f, M] = generate_output_arrays(N)

error.x = zeros(3, N);
error.v = zeros(3, N);
error.R = zeros(3, N);
error.W = zeros(3, N);

desired.x = zeros(3, N);
desired.b1 = zeros(3, N);
desired.R = zeros(3, 3, N);

R = zeros(3, 3, N);
f = zeros(1, N);
M = zeros(3, N);

end