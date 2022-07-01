# linear-subproblem-solutions
Implementation of subproblem solutions from "Canonical Subproblems for Robot Inverse Kinematics"

Subproblem 1: Angle between two vectors on the edge of a cone

$$\min_\theta \lVert p_2- R(k,\theta)p_1\rVert$$

Subproblem 2: Intersections between two cones

$$\min_{\theta_1, \theta_2} \lVert R(k_2,\theta_2)p_2 - R(k_1,\theta_1)p_1 \rVert$$

Subproblem 3: Intersections between a cone and a sphere

$$\min_\theta \lvert \lVert p_2-R(k,\theta)p_1\rVert-d\rvert$$

Subproblem 4: Intersections between a cone and a plane

$$\min_\theta \lvert h^\top R(k,\theta)p -d \rvert$$

Subproblem 5: Intersections between two cones separated by a vector

$$\min_{\theta_1, \theta_2}\lVert R(k_2,\theta_2)p_2 - R(k_1,\theta_1)p_1-p_0\rVert$$

Subproblem 6: Intersections between three separated cones

$$\min_{\theta_1, \theta_2, \theta_3}\lVert p_0 + R(k_1,\theta_1)p_1+
 R(k_2,\theta_2)(p_2+ R(k_3,\theta_3)p_3) \rVert$$
