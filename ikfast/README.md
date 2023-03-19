# IKFast Comparison

[IKFast Installation Instructions](https://github.com/rpiRobotics/installation-tutorials/blob/main/ikfast.md)

XML files were generated using [generate_xml.m](/matlab/ikfast/generate_xml.m). As shown below, IKFast could only generate 5 of 8 robot inverse kinematics solution files.
IKFast could not find solutions for Yumi, RRC, and Two-Parallel-Bot after running in excess of 5 hours.

Robot type                | Robot example           | IKFast C++ Generation Result
---                       | ---                     | ---
General 6R                | Yumi (fixed q3)         | :x: Failure
2 Intersecting Axes       | RRC (fixed q6)          | :x: Failure
2 Parallel Axes           | Two-Parallel-Bot        | :x: Failure
Spherical                 | Spherical-Bot           | :heavy_check_mark: Success
spherical 2 intersecting  | Kuka R800 (fixed q3)    | :heavy_check_mark: Success
Spherical 2 parallel      | ABB IRB 6640            | :heavy_check_mark: Success
3 parallel                | Three-Parallel-Bot      | :heavy_check_mark: Success
3 Parallel 2 Intersecting | UR5                     | :heavy_check_mark: Success (55 min!)

The command to generate a `cpp` file from an `xml` file is

```
python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot=robot_name.xml --baselink=0 --eelink=7 --savefile=robot_name.cpp
```

To test timing (in an albeit crude way), use `test_timing.cpp`.
This program has a modified version of `main()` from the generated IKfast files which runs the IK function `N` times and times the execution.
To compile this program and also measure compulation time, run
```
time g++ test_timing.cpp -lrt -O2 -DIKFAST_NO_MAIN -include robot_name.cpp  -DN=1000000
```
Change `N=` to some other value if desired. Use the matlab script `generate_example_call.m` to generate a random pose.
For some robots (e.g. ur5), adding `-llapack` is also required.

# IKFast Timing Results (Preliminary results - only 1 pose)

Among the 5 robots where IKFast could generate the `cpp` file, only 3 robots were able to compile and provide correct IK solutions.

| Specification| Value                         |
| :----------- | :---------------------------- |
| CPU          | Intel Core i7-3770K @ 3.50 GHz|
| OS           | Ubuntu 14.04.6 LTS running on Windows 10 Home WSL2|

| Robot type                      | Robot example       | Time (us)  | Comment                              | N    |
| :------------------------------ |  :----------        | ----------:| :------                              | :--  |
| Spherical                       | Spherical-Bot       | 91.40      | -                                    | 1e5  |
| Spherical Two Intersecting      | KUKA R800 (fixed q3)| -          | Doesn't compile!                     | -    |
| Spherical Two Parallel          | IRB 6640            | 6.25       | -                                    | 1e6  |
| Three Parallel                  | Three-Parallel-Bot  | -          | Wrong IK                             | -    |
| Three Parallel Two Intersecting | UR5                 | 2610.17    | Very long compilation:  9 min 32 sec | 1e3  |