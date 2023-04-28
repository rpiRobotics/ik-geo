Evaluation of MATLAB's Robotics Toolbox analytical inverse kinematics solver.

According to the documentation, a robot with a spherical wrist and 2 intersecting axes should be able to work with this method.
However, `analyticalInverseKinematics()` crashes for a robot of this type.

References:

[analyticalInverseKinematics](https://www.mathworks.com/help/robotics/ref/analyticalinversekinematics.html)

[generateIKFunction](https://www.mathworks.com/help/robotics/ref/analyticalinversekinematics.generateikfunction.html)


Robot type                | Robot example           | `generateIKFunction` Result
---                       | ---                     | ---
General 6R                | Yumi (fixed q3)         | :x: Failure
2 Intersecting Axes       | RRC (fixed q6)          | :x: Failure
2 Parallel Axes           | Two-Parallel-Bot        | :x: Failure
Spherical                 | Spherical-Bot           | :heavy_check_mark: Success
spherical 2 intersecting  | Kuka R800 (fixed q3)    | :x: Failure
Spherical 2 parallel      | ABB IRB 6640            | :heavy_check_mark: Success
3 parallel                | Three-Parallel-Bot      | :x: Failure
3 Parallel 2 Intersecting | UR5                     | :x: Failure

| Inverse Kinematics              | Hardcoded Example   | `m` Time (us)      | `MEX` Time (us) | N   |
| :------------------------------ |  :----------        | ----------:        | --:             | --: |
| Spherical                       | Spherical-Bot       |    1176.4168       | 11.8820         | 1e3 |
| Spherical Two Parallel          | IRB 6640            |    1384.6896       | 14.5846         | 1e3 |