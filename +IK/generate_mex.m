% TODO change path

setup = IK_setups.IK_spherical_2_parallel;

[P, S_given] = setup.setup();
codegen -report +IK\IK_spherical_2_parallel.m -args {P.R, P.T, P.kin}

%% 
setup = IK_setups.IK_3_parallel;

[P, S_given] = setup.setup();
codegen -report +IK\IK_3_parallel.m -args {P.R, P.T, P.kin}