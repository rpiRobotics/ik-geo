%% Simple demo using Product of Exponentials convention to define robot kinematics

% Define robot kinematics using product of exponentials
% (ABB IRB_6640)
clc

zv = [0;0;0];
ex = [1;0;0];
ey = [0;1;0];
ez = [0;0;1];
kin.H = [ez ey ey ex ey ex];
kin.P = [zv, 0.32*ex+0.78*ez, 1.075*ez, 1.1425*ex+0.2*ez, zv, zv, 0.2*ex];
kin.joint_type = zeros([6 1]);

% Identify intersecting and parallel axes and recommend solvers
[is_intersecting, is_intersecting_nonconsecutive, is_parallel, is_spherical] = detect_intersecting_parallel_axes(kin);
print_intersecting_parallel_axes(is_intersecting, is_intersecting_nonconsecutive, is_parallel, is_spherical);
fprintf("\n");
rec_solver_6_DOF(is_intersecting, is_intersecting_nonconsecutive, is_parallel, is_spherical)

% Generate a random robot pose
q = rand([6 1]);
[R_06, p_0T] = fwdkin(kin, q);

% Solve inverse kinematics using the recommended closed-form solver
[Q, is_LS] = IK.IK_spherical_2_parallel(R_06, p_0T, kin);

% One of the returned IK solutions should match
q %#ok<NOPTS>
Q %#ok<NOPTS>