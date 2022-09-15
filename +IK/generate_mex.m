% TODO change path

setup = IK_setups.IK_spherical_2_parallel;

[P, S_given] = setup.setup();
codegen -report +IK\IK_spherical_2_parallel.m -args {P.R, P.T, P.kin}

%% 
setup = IK_setups.IK_3_parallel;

[P, S_given] = setup.setup();
codegen -report +IK\IK_3_parallel.m -args {P.R, P.T, P.kin}

%%
setup = IK_setups.IK_spherical;

[P, S_given] = setup.setup();
codegen -report +IK\IK_spherical.m -args {P.R, P.T, P.kin}
%%
setup = IK_setups.IK_spherical_2_intersecting;

[P, S_given] = setup.setup();
codegen -report +IK\IK_spherical_2_intersecting.m -args {P.R, P.T, P.kin}

%%
setup = IK_setups.IK_2R_2R_3R;

[P, S_given] = setup.setup();
codegen -report +IK\IK_2R_2R_3R.m -args {P.R, P.T, P.sew, P.psi, P.kin}