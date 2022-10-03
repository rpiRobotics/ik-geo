# Linear Subproblem Solutions
Implementation of subproblem solutions from "Canonical Subproblems for Robot Inverse Kinematics"

### Subproblem 1: Cone and point

[Interactive figure](https://www.geogebra.org/calculator/stydabbr)

$$\min_\theta \lVert R(k,\theta)p_1 - p_2\rVert$$

### Subproblem 2: Two cones

[Interactive figure](https://www.geogebra.org/calculator/prj5wqsu)

$$\min_{\theta_1,\theta_2} \lVert R(k_1,\theta_1)p_1 - R(k_2,\theta_2)p_2\rVert$$

### Subproblem 2 extension: Two offset cones
$$p_0 + R(k_1,\theta_1)p_1= R(k_2,\theta_2)p_2$$

### Subproblem 3: Cone and sphere

$$\min_\theta \lvert \lVert R(k,\theta)p_1-p_2\rVert-d\rvert$$

### Subproblem 4: Cone and plane

$$\min_\theta \lvert h^\top R(k,\theta)p -d \rvert$$

### Subproblem 5: Three cones

$$ p_0 + R(k_1,\theta_1)p_1=
 R(k_2,\theta_2)(p_2+ R(k_3,\theta_3)p_3)$$

### Subproblem 6: Four cones

$$\begin{cases}
    h_1^\top R(k_1, \theta_1)p_1 + h_2^\top R(k_2, \theta_2)p_2 = d_1\\
    h_3^\top R(k_3, \theta_1)p_3 + h_3^\top R(k_4, \theta_2)p_4 = d_2
\end{cases}$$


## Folder breakdown
`+IK`: Inverse kinematics solutions for different robot types

`+IK_setups`: Test classes for IK solutions

`+subproblem`: Subproblem solutions

`+subproblem_setups`: Test classes for subproblem solutions

`rand_helpers`: Functions to help generate random test cases

`robot_IK_helpers`: Functions to help with robot IK (and forward kinematics)

`robot_examples`: Inverse kinematics solutions for specific robots

`correctness_tests`: Tests to verify the correctness of subproblem and IK solutions

`timing_tests`: Measures the runtime of subproblem and IK solutions

`unit_tests`: Testing for smaller helper functions

## Dependencies

[general-robotics-toolbox](https://github.com/rpiRobotics/general-robotics-toolbox) for rotation matrix, forward kinematics, Jacobian

[matlab-diagrams](https://github.com/aelias36/matlab-diagrams) for a few visualizations
