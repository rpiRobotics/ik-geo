| Specification| Value                         |
| :----------- | :---------------------------- |
| CPU          | AMD Ryzen 9 4900HS @ 3.00 GHz |
| OS           | Windows 10 Home               |


# Criterion

| Subproblem | Time (ns)   |
| :--------- | ----------: |
| 1          |      26.782 |
| 2          |      121.52 |
| 2 Ex       |      92.794 |
| 3          |      58.857 |
| 4          |      50.471 |
| 5          |      678.99 |
| 6          |      1427.9 |

| Inverse Kinematics              | Time (ns)   |
| :------------------------------ | ----------: |
| Spherical Two Parallel          |      2315.9 |
| Spherical Two Intersecting      |      2120.6 |
| Spherical                       |      2993.6 |
| Three Parallel Two Intersecting |      2993.6 |
| Three Parallel                  |      3446.3 |


# Manual Timing

| Subproblem | Separate Time (ns)   | Batched Time (ns)   |
| :--------- | -------------------: | ------------------: |
| 1          |                   34 |                  62 |
| 2          |                  152 |                 182 |
| 2 Ex       |                  108 |                 138 |
| 3          |                   78 |                 107 |
| 4          |                   71 |                 101 |
| 5          |                  898 |                 872 |
| 6          |                 1582 |                1553 |

| Ik                              | Separate Time (ns)   | Batched Time (ns)   |
| :------------------------------ | -------------------: | ------------------: |
| Spherical                       |                 1807 |                1787 |
| Spherical Two Intersecting      |                 1451 |                1424 |
| Spherical Two Parallel          |                 2919 |                2922 |
| Three Parallel                  |                 2759 |                2733 |
| Three Parallel Two Intersecting |                 1354 |                1325 |
