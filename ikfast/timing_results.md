# IKFast Timing Results (Preliminary results - only 1 pose)

| Specification| Value                         |
| :----------- | :---------------------------- |
| CPU          | Intel Core i7-3770K @ 3.50 GHz|
| OS           | Ubuntu 14.04.6 LTS running on Windows 10 Home WSL2|

| Inverse Kinematics              | Hardcoded Example   | Time (us)  | Comment                              | N    |
| :------------------------------ |  :----------        | ----------:| :------                              | :--  |
| Spherical                       | Spherical-Bot       | 91.40      | -                                    | 1e5  |
| Spherical Two Intersecting      | KUKA R800 (fixed q3)| -          | Doesn't compile!                     | -    |
| Spherical Two Parallel          | IRB 6640            | 6.25       | -                                    | 1e6  |
| Three Parallel                  | Three-Parallel-Bot  | -          | Wrong IK                             | -    |
| Three Parallel Two Intersecting | UR5                 | 2610.17    | Very long compilation:  9 min 32 sec | 1e3  |