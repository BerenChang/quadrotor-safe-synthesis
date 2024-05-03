function desired = command_bezier(t)
desired_bezier = DesiredTrajectory(t,Points_Array,tau);

desired.x = desired_bezier.x;
desired.v = desired_bezier.x_dot;
desired.x_2dot = desired_bezier.x_2dot;
desired.x_3dot = desired_bezier.x_3dot;
desired.x_4dot = desired_bezier.x_4dot;

desired.b1 = [1, 0, 0]';
desired.b1_dot = [0, 0, 0]';
desired.b1_2dot = [0, 0, 0]';

end