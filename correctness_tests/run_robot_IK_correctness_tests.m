% setup = IK_setups.IK_spherical_2_parallel;
% setup = IK_setups.IK_3_parallel;
% setup = IK_setups.IK_3_parallel_2_intersecting;
% setup = IK_setups.IK_spherical_2_intersecting;
% setup = IK_setups.IK_spherical;
% setup = IK_setups.IK_2R_2R_3R;
setup = IK_setups.IK_2_intersecting;
% setup = IK_setups.IK_gen_6_dof;

[P, S_given] = setup.setup();
setup.error(P,S_given)


S = setup.run(P);
%S = setup.run_mex(P);

S.is_LS

[e, e_R, e_T] = robot_IK_error(P,S);
e

% S_exact.Q = S.Q(:,~S.is_LS);
% e = setup.error(P, S_exact)