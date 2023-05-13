# Linear Subproblem Solutions
Implementation of subproblem solutions using a linear algebra approach from ["Canonical Subproblems for Robot Inverse Kinematics"](https://arxiv.org/abs/2211.05737). We also include inverse kinematics solutions to a number of 6-dof robot types, examples with specific robots, and timing tests.

### Subproblem 1: Cone and point

$$\min_\theta \lVert R(k,\theta)p_1 - p_2\rVert$$

### Subproblem 2: Two cones

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
Make sure to switch to the right branch to see the most recent progress.

`matlab`: MATLAB implementation

`cpp`: C++ implementation (Work in progress)

`rust`: Rust implementation (Work in progress)

`python`: Python implementation (Work in progress)

`ikfast`: Comparison to Inverse Kinematics using IKFast (Work in progress)

## Contributing
If you have any improvements you'd like to make, or even ideas or requests for improvements, please start a GitHub issue.
