# linear-subproblem-solutions
Implementation of subproblem solutions from "Canonical Subproblems for Robot Inverse Kinematics"

Subproblem 1: Angle between two vectors on the edge of a cone

[Interactive figure](https://www.geogebra.org/calculator/stydabbr)

$$\min_\theta \lVert p_2- R(k,\theta)p_1\rVert$$

Subproblem 2: Intersections between two cones

$$\min_{\theta_1, \theta_2} \lVert R(k_2,\theta_2)p_2 - R(k_1,\theta_1)p_1 \rVert$$

Subproblem 2 extension: Intersection between 2 offset cones
$$p_0 + R(k_1,\theta_1)p_1= R(k_2,\theta_2)p_2$$

Subproblem 3: Intersections between a cone and a sphere

$$\min_\theta \lvert \lVert p_2-R(k,\theta)p_1\rVert-d\rvert$$

Subproblem 4: Intersections between a cone and a plane

$$\min_\theta \lvert h^\top R(k,\theta)p -d \rvert$$

Subproblem 5: Intersections among three separated cones

$$ p_0 + R(k_1,\theta_1)p_1=
 R(k_2,\theta_2)(p_2+ R(k_3,\theta_3)p_3)$$

Subproblem 6: 4 cones with 2 angles

$$\begin{cases}
    h^\top R(k_1, \theta_1)p_1 + h^\top R(k_2, \theta_2)p_2 = d_1\\
    h^\top R(k_1, \theta_1)p_3 + h^\top R(k_2, \theta_2)p_4 = d_2
\end{cases}$$
