| Specification| Value                         |
| :----------- | :---------------------------- |
| CPU          | AMD Ryzen 9 4900HS @ 3.00 GHz |
| OS           | Windows 10 Home               |


# Criterion

| Subproblem | Time (ns) |
| :--------- | --------: |
| 1          |    26.782 |
| 2          |    119.58 |
| 2 Ex       |    92.794 |
| 3          |    58.857 |
| 4          |    50.471 |
| 5          |    678.99 |
| 6          |    1427.9 |

| Inverse Kinematics              | Time (ns) |
| :------------------------------ | --------: |
| Spherical Two Parallel          |    2315.9 |
| Spherical Two Intersecting      |    2120.6 |
| Spherical                       |    2993.6 |
| Three Parallel Two Intersecting |    2993.6 |
| Three Parallel                  |    3446.3 |
| Two Parallel                    |    437870 |
| Two Intersecting                |    244410 |

# Manual Timing

| Subproblem | Separate Time (ns) | Batched Time (ns) |
| :--------- | -----------------: | ----------------: |
| 1          |                 34 |                62 |
| 2          |                204 |               169 |
| 2 Ex       |                108 |               138 |
| 3          |                 78 |               107 |
| 4          |                 71 |               101 |
| 5          |                898 |               872 |
| 6          |               1582 |              1553 |

| Ik                              | Separate Time (ns) | Batched Time (ns) |
| :------------------------------ | -----------------: | ----------------: |
| Spherical                       |               1691 |              1661 |
| Spherical Two Intersecting      |               2064 |              2067 |
| Spherical Two Parallel          |               2265 |              2245 |
| Three Parallel                  |               6103 |              6091 |
| Three Parallel Two Intersecting |               2146 |              2098 |
| Two Parallel                    |             328937 |            328445 |
| Two Intersecting                |             212851 |            210588 |

## IkFast

| Ik                     | Example       | Separate Time (ns) | Batched Time (ns) |
| :--------------------- | :------------ | -----------------: | ----------------: |
| Spherical              | Spherical Bot |              28293 |             29242 |
| Spherical Two Parallel | IRB 6640      |               6941 |              7359 |
