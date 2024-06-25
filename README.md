# IK-Geo
Inverse kinematics and subproblem solutions from ["IK-Geo: Unified Robot Inverse Kinematics Using Subproblem Decomposition"](https://arxiv.org/abs/2211.05737) implemented in MATLAB, C++, Rust, and Python. We also include examples and timing tests.  IK-Geo is the fastest general IK solver based on published literature. In this unifying approach, IK for any 6-DOF all-revolute (6R) manipulator is decomposed into six canonical geometric subproblems solved by intersecting circles with other geometric objects. IK-Geo finds all IK solutions including singular solutions and sometimes least-squares solutions by solving for subproblem solutions in all cases, including in a continuous and sometimes least-squares sense when a solution does not exist.

## Related Repos

We also connect our geometric method with polynomial-based method: 1D and 2D search solutions may be converted to a polynomial in the tangent half-angle of one joint. Examples are found in the [subproblem-polynomial](https://github.com/rpiRobotics/subproblem-polynomial) repo.

For 7-DOF inverse kinematics using the Shoulder-Elbow-Wrist (SEW) angle, see the [stereo-sew](https://github.com/rpiRobotics/stereo-sew) repo.

For diagrams, see the [matlab-diagrams](https://github.com/aelias36/matlab-diagrams) repo.

Improvements to the 1D search algorithm have been made in [1d_search_improvements](https://github.com/rpiRobotics/1d_search_improvements).

The pure python implementation can be found at [https://pypi.org/project/linearSubproblemSltns/](https://pypi.org/project/linearSubproblemSltns/).

The branch of this repo by TU Munich, [OstermD/ik-geo](https://github.com/OstermD/ik-geo), along with their [EAIK](https://github.com/OstermD/EAIK) repo, demonstrates automating the IK derivation process and provides a python wrapper for the C++ implementation. The Python wrapper can be found at [https://pypi.org/project/EAIK/](https://pypi.org/project/EAIK/).

The branch of this repo by Verdant Evolution, [Verdant-Evolution/ik-geo](https://github.com/Verdant-Evolution/ik-geo), provides a Python wrapper for the Rust implementation along with some improvements to the 1D and 2D search algorithms. The Python wrapper can be found at [https://pypi.org/project/ik-geo/](https://pypi.org/project/ik-geo/).

## Inverse Kinematics Solutions
Robots are classified into kinematic families based on cases of intersecting or parallel joint axes, and robots in the same family use the same IK algorithm.

6R robots with three intersecting or parallel axes are solved in closed form, and all solutions are found exactly without iteration. Other 6R robots are efficiently solved by searching for zeros of an error function of one or two joint angles. To the best of our knowledge, all commercially available industrial 6R robots and 7R robots parameterized by some joint angle have intersecting or parallel axes and therefore can be solved in closed form or with 1D search.

We provide implementations for many of the kinematic families shown below.

See [matlab/automatic_IK/](matlab/automatic_IK/) to automatically detect which IK solvers can be used for a given robot.


| Solution Type | Robot Kinematic Family                             | Example                            |
| ------------- | -------------------------------------------------- | ---------------------------------- |
| Closed-form   | Spherical joint                                    | Franka Production 3, fixed $q_5$   |
|               | &nbsp;&nbsp;&nbsp;&nbsp; and two intersecting axes | KUKA LBR iiwa 7 R800 , fixed $q_3$ |
|               | &nbsp;&nbsp;&nbsp;&nbsp; and two parallel axes     | ABB IRB 6640                       |
|               | Three parallel axes                                | N/A                                |
|               | &nbsp;&nbsp;&nbsp;&nbsp; and two intersecting axes | Universal Robots UR5               |
|               | &nbsp;&nbsp;&nbsp;&nbsp; and two parallel axes     | N/A                                |
| 1D search     | Two intersecting axes                              | Kassow Robots KR810, fixed $q_7$   |
|               | &nbsp;&nbsp;&nbsp;&nbsp; and two intersecting axes | FANUC CRX-10iA/L                   |
|               | &nbsp;&nbsp;&nbsp;&nbsp; and two parallel axes     | Kawasaki KJ125                     |
|               | Two parallel axes                                  | N/A                                |
|               | &nbsp;&nbsp;&nbsp;&nbsp; and two parallel axes     | N/A                                |
|               | Two intersecting axes $k, k+2$                     | ABB YuMi, fixed $q_3$              |
|               | &nbsp;&nbsp;&nbsp;&nbsp; and two intersecting axes | RRC K-1207i, fixed $q_6$           |
|               | &nbsp;&nbsp;&nbsp;&nbsp; and two parallel axes     | N/A                                |
| 2D search     | General 6R                                         | Kassow Robots KR810, fixed $q_6$   |

## Subproblem Solutions
We present efficient and singularity-robust solutions to the following subproblems using geometric and linear algebra methods.

Subproblems 2 and 3 are solved using Subproblem 4. Subproblems 5 and 6 are solved by finding the intersections between two ellipses.

### Subproblem 1: Circle and point

$$\min_\theta \lVert R(k,\theta)p_1 - p_2\rVert$$

### Subproblem 2: Two circles

$$\min_{\theta_1,\theta_2} \lVert R(k_1,\theta_1)p_1 - R(k_2,\theta_2)p_2\rVert$$

### Subproblem 3: Circle and sphere

$$\min_\theta \lvert \lVert R(k,\theta)p_1-p_2\rVert-d\rvert$$

### Subproblem 4: Circle and plane

$$\min_\theta \lvert h^\top R(k,\theta)p -d \rvert$$

### Subproblem 5: Three circles

$$ p_0 + R(k_1,\theta_1)p_1=
 R(k_2,\theta_2)(p_2+ R(k_3,\theta_3)p_3)$$

### Subproblem 6: Four circles

$$\begin{cases}
    h_1^\top R(k_1, \theta_1)p_1 + h_2^\top R(k_2, \theta_2)p_2 = d_1\\
    h_3^\top R(k_3, \theta_1)p_3 + h_3^\top R(k_4, \theta_2)p_4 = d_2
\end{cases}$$

## Folder breakdown
Make sure to switch to the right branch to see the most recent progress.

`cpp`: C++ implementation (Work in progress)

`ikfast`: Inverse kinematics comparison with IKFast

`matlab`: Reference MATLAB implementation

`python`: Python implementation (Work in progress)

`rust`: Rust implementation

## Questions and Contributing
If you have any questions, improvements you'd like to make, or even ideas or requests for improvements, please start a GitHub issue or send an email. 
