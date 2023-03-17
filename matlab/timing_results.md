| Specification| Value                         |
| :----------- | :---------------------------- |
| CPU          | Intel Core i7-3770K @ 3.50 GHz|
| OS           | Windows 10 Home               |


| Subproblem | `m` Time (ns)   |  `MEX` Time (ns)   |
| :--------- | ----------: |  ----------: |
| 1          |      16,512.0 | 39.92 |
| 2          |      41,027.2 | 396.51 |
| 2 Ex       |      45,913.5 | 339.16 |
| 3          |      22,768.9 | 188.52 |
| 4          |      19,184.6 | 110.64 |
| 5          |      107,764.8 | 2,472.32 |
| 6          |      113,294.4 | 1,990.30 |


| Inverse Kinematics              | Hardcoded Example   | `m` Time (us)   | `MEX` Time (us) |Hardcoded `MEX` Time (us) | N |
| :------------------------------ |  :----------        | ----------:     | ----------:     | --:                      | --:|
| General 6-dof                   | Yumi (fixed q3)     | 332,759.41      | 7,145.03        | 5,581.96                 | 100  |
| 2 Intersecting                  | RRC (fixed q6)      | 37,889.28       |   578.89        | 153.3720                 | 100  |
| 2 Parallel                      | Two-Parallel-Bot    | 82,191.55       |   504.92        | 566.6760                 | 100  |
| Spherical                       | Spherical-Bot       |    541.14       |   4.8332        | 4.0720                   | 10e3 |
| Spherical Two Intersecting      | KUKA R800 (fixed q3)|    666.37       |   3.9413        | 3.6478                   | 10e3 |
| Spherical Two Parallel          | IRB 6640            |    758.80       |   3.8589        | 3.1370                   | 10e3 |
| Three Parallel                  | Three-Parallel-Bot  |    411.98       |   5.0142        | 4.3541                   | 10e3 |
| Three Parallel Two Intersecting | UR5                 |    456.67       |   3.5232        | 3.4960                   | 10e3 |