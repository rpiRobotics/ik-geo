# Automatic IK

Automatically detect intersecting and parallel axes to determine which inverse kinematics solver to use. See [`demo_POE.m`](demo_POE.m) for a simple example.

Caveat: Some IK solvers require certain joint offsets to be set to 0. This is not done automatically right now. (It's also possible to modify the IK solvers to not require joint offsets to be 0.)

`automatic_IK.m`: Demonstrate automatically detecting compatible solvers based on robot kinematics

`demo_POE.m`: Simple demo using Product of Exponentials convention to define robot kinematics

`dh_to_kin.m`: Convert a robot define in Denavit-Hartenberg convention to Product of Exponentials

`detect_intersecting_parallel_axes.m`: Test robot kinematics to determine intersecting or parallel axes

`print_intersecting_parallel_axes.m`: Print intersecting / parallel axes to console

`rec_solvers_6_DOF`: Recommend 6-DOF solvers given detected intersecting / parallel axes

`rec_solvers_6_DOF`: Recommend 7-DOF solvers given detected intersecting / parallel axes

# Example

Below is the enirety of `demo_POE.m` along with the output.

``` MATLAB
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
```

```
Intersecting Joints: (4, 5) (5, 6) 
Intersecting Nonconsecutive Joints: (4, 6) 
Parallel Joints: (2, 3) 
Spherical Joints: (4, 5, 6) 

Spherical joint: Closed-form solutions exist
Intersecting axes: 1D search solutions exist
Nonconsecutive intersecting axes: 1D search solutions exist
Parallel axes: 1D search solutions exist

Compatible solvers:
IK_spherical_2_parallel (Closed-Form Quadratic)
IK_spherical (Closed-Form Quartic)
IK_2_intersecting (1D Search)
IK_2_parallel (1D Search)
IK_4_6_intersecting (1D Search)
IK_gen_6_dof (2D Search)

q =

    0.8147
    0.9058
    0.1270
    0.9134
    0.6324
    0.0975


Q =

    0.8147    0.8147    0.8147    0.8147   -2.3269   -2.3269
    2.5028    2.5028    0.9058    0.9058   -1.6665   -1.6665
   -2.9220   -2.9220    0.1270    0.1270   -1.3975   -1.3975
    0.5063   -2.6353    0.9134   -2.2282   -2.6544    0.4872
    1.8370   -1.8370    0.6324   -0.6324    1.5360   -1.5360
    1.0497   -2.0919    0.0975   -3.0441    0.8864   -2.2551
```
