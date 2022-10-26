## Folder breakdown
`+IK`: Inverse kinematics solutions for different robot types

`+IK_setups`: Test classes for IK solutions

`+hardcoded_IK`: Inverse kinemtics solutions for specific robots (kinematic parameters are set)

`+hardcoded_IK_setups`: Test classes for hardcoded IK solutions

`+subproblem`: Subproblem solutions

`+subproblem_setups`: Test classes for subproblem solutions

`correctness_tests`: Tests to verify the correctness of subproblem and IK solutions

`rand_helpers`: Functions to help generate random test cases

`robot_IK_helpers`: Functions to help with robot IK (and forward kinematics)

`robot_examples`: Inverse kinematics solutions for specific robots

`timing_tests`: Measures the runtime of subproblem and IK solutions

`unit_tests`: Testing for smaller helper functions

## Dependencies

[general-robotics-toolbox](https://github.com/rpiRobotics/general-robotics-toolbox) for rotation matrix, forward kinematics, Jacobian

[matlab-diagrams](https://github.com/aelias36/matlab-diagrams) for a few visualizations