## Subproblem

CPU: Intel(R) Core(TM) i7-9750H CPU @ 2.60GHz  
OS: Ubuntu 20.04.1 LTS

| Subproblem | Separate Time |
| :--------- | ------------: |
| 1          |   67.112 ns   |
| 2          |   183.40 ns   |
| 2 Ex       |   171.59 ns   |
| 3          |   122.74 ns   |
| 4          |   105.45 ns   |
| 5          |   1353.2 ns   |
| 6          |   2077.8 ns   |

&emsp;  
CPU: AMD Ryzen 7 4700U APU @ 2.00GHz   
OS: Ubuntu 20.04.3 LTS

| Subproblem | Separate Time |
| :--------- | ------------: |
| 1          |   108.99 ns   |
| 2          |   299.42 ns   |
| 2 Ex       |   239.06 ns   |
| 3          |   216.57 ns   |
| 4          |   181.86 ns   |
| 5          |   2124.9 ns   |
| 6          |   2593.2 ns   |

&emsp;
## IK

CPU: Intel(R) Core(TM) i7-9750H CPU @ 2.60GHz  
OS: Ubuntu 20.04.1 LTS

| IK                        | Separate Time |
| :------------------------ | ------------: |
| spherical                 |   4310.7 ns   |
| spherical 2 intersecting  |               |
| spherical 2 parallel      |   3632.8 ns   |
| 3 parallel                |               |
| 3 parallel 2 intersecting |               |
| 2 intersecting            |               |
| 2 parallel                |               |
| gen 6 dof                 |               |

&emsp;  
CPU: AMD Ryzen 7 4700U APU @ 2.00GHz  
OS: Ubuntu 20.04.3 LTS

| IK                        | Separate Time |
| :------------------------ | ------------: |
| spherical                 |               |
| spherical 2 intersecting  |               |
| spherical 2 parallel      |               |
| 3 parallel                |               |
| 3 parallel 2 intersecting |               |
| 2 intersecting            |               |
| 2 parallel                |               |
| gen 6 dof                 |               |
