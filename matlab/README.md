## Instructions
Download the code and make sure this folder in in the MATLAB path. Functions in folders with a `+` can be called using [dot notation](https://www.mathworks.com/help/matlab/matlab_oop/scoping-classes-with-packages.html#brfynt_-3).

e.g. `subproblem.sp_1(p1, p2, k)`.

## Folder breakdown
`+IK`: Inverse kinematics solutions for different robot types

`+IK_setups`: Test classes for IK solutions

`+hardcoded_IK`: Inverse kinemtics solutions for specific robots (kinematic parameters are set)

`+hardcoded_IK_setups`: Test classes for hardcoded IK solutions

`+subproblem_setups`: Subproblem solutions

`subproblems`: Test classes for subproblem solutions

`correctness_tests`: Tests to verify the correctness of subproblem and IK solutions

`general-robotics-toolbox`: Helper functions from [rpiRobotics/general-robotics-toolbox](https://github.com/rpiRobotics/general-robotics-toolbox)

`ikfast`: Generate robot XML files for use with IKFast

`matlab_robotics_toolbox_analytical`: Evaluation of MATLAB's Robotics Toolbox analytical inverse kinematics solver

`rand_helpers`: Functions to help generate random test cases

`robot_IK_helpers`: Functions to help with robot IK (and forward kinematics)

`robot_examples`: Inverse kinematics solutions for specific robots

`tan_half_angle`: Demonstration of singularity issues arising from $x = \tan(\theta/2)$

`timing_tests`: Measures the runtime of subproblem and IK solutions

`unit_tests`: Testing for smaller helper functions

## Dependencies

[matlab-diagrams](https://github.com/aelias36/matlab-diagrams) for a few visualizations
