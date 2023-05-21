| Specification| Value                         |
| :----------- | :---------------------------- |
| CPU          | AMD Ryzen 9 4900HS @ 3.00 GHz |
| OS           | Windows 10 Home               |

# Criterion

| Subproblem | Time (ns) |
| :--------- | --------: |
| 1          |    26.782 |
| 2          |   119.580 |
| 2 Ex       |    92.794 |
| 3          |    58.857 |
| 4          |    50.471 |
| 5          |   678.990 |
| 6          |  1427.900 |

| Inverse Kinematics              | Time (ns) |
| :------------------------------ | --------: |
| Spherical Two Parallel          |    2315.9 |
| Spherical Two Intersecting      |    2120.6 |
| Spherical                       |    2993.6 |
| Three Parallel Two Intersecting |    2993.6 |
| Three Parallel                  |    3446.3 |
| Two Parallel                    |  437870.0 |
| Two Intersecting                |  244410.0 |
| Gen Six DOF                     | 7198200.0 |

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
| Gen Six DOF                     |            8452051 |           8812318 |

| Hardcoded Ik       | Separate Time (ns) | Batched Time (ns) |
| :----------------- | -----------------: | ----------------: |
| IRB 6640           |               3176 |              3188 |
| KUKA R800 Fixed Q3 |               3125 |              3045 |
| RRC Fixed Q6       |             124508 |            124889 |
| Yumi Fixed Q3      |            6642416 |           6690284 |
| UR5                |               2994 |              2960 |
| Three Parallel Bot |               3490 |              3479 |
| Two Parallel Bot   |             375022 |            374500 |
| Spherical Bot      |               2078 |              2049 |

## IkFast

| Ik                     | Example       | Separate Time (ns) | Batched Time (ns) |
| :--------------------- | :------------ | -----------------: | ----------------: |
| Spherical              | Spherical Bot |              28293 |             29242 |
| Spherical Two Parallel | IRB 6640      |               6941 |              7359 |
