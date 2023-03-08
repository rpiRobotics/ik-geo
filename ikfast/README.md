# IKFast Comparison

[IKFast Installation Instructions](https://github.com/rpiRobotics/installation-tutorials/blob/main/ikfast.md)

XML files were generated using [generate_xml.m](/matlab/ikfast/generate_xml.m). As shown below, IKFast could only generate 5 of 8 robot inverse kinematics solution files.
IKFast could not find solutions for Yumi and RRC after running in excess of 5 hours. Although IKFast could apparently find a solution for the 2 parallel axis case, 
python crashed during writing the `cpp` file due to a max recursion depth error.

Robot type                | Robot example           | IKFast Result
---                       | ---                     | ---
General 6R                | Yumi (fixed q3)         | :x: Failure
2 Intersecting Axes       | RRC (fixed q6)          | :x: Failure
2 Parallel Axes           | (Random 2 parallel)     | :x: Failure (Max recursion depth reached)
Spherical                 | spherical-bot           | :heavy_check_mark: Success
spherical 2 intersecting  | Kuka R800 (fixed q3)    | :heavy_check_mark: Success
Spherical 2 parallel      | ABB IRB 6640            | :heavy_check_mark: Success
3 parallel                | 3-parallel-bot          | :heavy_check_mark: Success
3 Parallel 2 Intersecting | UR5                     | :heavy_check_mark: Success (55 min!)

The command to generate a `cpp` file from an `xml` file is

```
python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot=robot_name.xml --baselink=0 --eelink=7 --savefile=robot_name.cpp
```