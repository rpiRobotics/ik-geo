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

| Ik                              | Time (ns) |
| :------------------------------ | --------: |
| Spherical Two Parallel          |    1857.5 |
| Spherical Two Intersecting      |    2120.6 |
| Spherical                       |    2993.6 |
| Three Parallel Two Intersecting |    2053.8 |
| Three Parallel                  |    2359.3 |
| Two Parallel                    |  437870.0 |
| Two Intersecting                |  244410.0 |
| Gen Six DOF                     | 7198200.0 |

| Hardcoded Ik       | Time (ns) |
| :----------------- | --------: |
| IRB 6640           |    2380.4 |
| KUKA R800 Fixed Q3 |    2327.5 |
| RRC Fixed Q6       |    194660 |
| Yumi Fixed Q3      |   5903000 |
| UR5                |    2412.7 |
| Three Parallel Bot |    5326.0 |
| Two Parallel Bot   |    348130 |
| Spherical Bot      |    1627.4 |

# Manual Timing

| Subproblem | Separate Time (ns) | Batched Time (ns) |
| :--------- | -----------------: | ----------------: |
| 1          |                 34 |                35 |
| 2          |                204 |               169 |
| 2 Ex       |                108 |               124 |
| 3          |                 78 |                76 |
| 4          |                 71 |                76 |
| 5          |                898 |               872 |
| 6          |               1582 |              1553 |

| Ik                              | Separate Time (ns) | Batched Time (ns) |
| :------------------------------ | -----------------: | ----------------: |
| Spherical                       |               1682 |              1608 |
| Spherical Two Intersecting      |               2064 |              1843 |
| Spherical Two Parallel          |               1976 |              1919 |
| Three Parallel                  |               2579 |              3273 |
| Three Parallel Two Intersecting |               2146 |              2008 |
| Two Parallel                    |             302951 |            328445 |
| Two Intersecting                |             212851 |            210588 |
| Gen Six DOF                     |            7945598 |           7863735 |

| Hardcoded Ik       | Separate Time (ns) | Batched Time (ns) |
| :----------------- | -----------------: | ----------------: |
| IRB 6640           |               2729 |              2711 |
| KUKA R800 Fixed Q3 |               3125 |              3045 |
| RRC Fixed Q6       |             116077 |            114669 |
| Yumi Fixed Q3      |            6467410 |           6493333 |
| UR5                |               2723 |              2664 |
| Three Parallel Bot |               3490 |              3479 |
| Two Parallel Bot   |             375022 |            374500 |
| Spherical Bot      |               1971 |              1925 |

## IkFast

| Ik                     | Example       | Separate Time (ns) | Batched Time (ns) |
| :--------------------- | :------------ | -----------------: | ----------------: |
| Spherical              | Spherical Bot |              28293 |             29242 |
| Spherical Two Parallel | IRB 6640      |               6941 |              7359 |
