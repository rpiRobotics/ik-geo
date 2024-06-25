# Automatic IK

Automatically detect intersecting and parallel axes to determine which inverse kinematics solver to use. See [`demo_POE.m`](demo_POE.m) for a simple example.

Caveat: Some IK solvers require certain joint offsets to be set to 0. This is not done automatically right now. (It's also possible to modify the IK solvers to not require joint offsets to be 0.)

`automatic_IK.m`: Demonstrate automatically detecting compatible solvers based on robot kinematics

`demo_POE.m`: Simple demo using Product of Exponentials convention to define robot kinematics

`dh_to_kin.m`: Convert a robot define in Denavit-Hartenberg convention to Product of Exponentials

`detect_intersecting_parallel_axes.m`: Test robot kinematics to determine intersecting or parallel axes

`print_intersecting_parallel_axes.m`: Print intersecting / parallel axes to console

`rec_solvers_6_DOF`: Recommend 6-DOF solvers given detected intersecting / parallel axes

`rec_solvers_6_DOF`: Recommend 7-DOF solvers given detected intersecting / parallel axes