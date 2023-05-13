% Check correctness of IK when kinematics parameters are hardcoded


% setup = hardcoded_IK_setups.yumi_fixed_q3;
% setup = hardcoded_IK_setups.RRC_fixed_q6;
setup = hardcoded_IK_setups.two_parallel_bot;
% setup = hardcoded_IK_setups.IRB_6640;
% setup = hardcoded_IK_setups.ur5;
% setup = hardcoded_IK_setups.three_parallel_bot;
% setup = hardcoded_IK_setups.spherical_bot;
% setup = hardcoded_IK_setups.KUKA_R800_fixed_q3;


[P, S_given] = setup.setup();

setup.error(P,S_given)

 S = setup.run(P);
% S = setup.run_mex(P);

[e, e_R, e_T] = setup.error(P,S);
S.is_LS
e

%% 
[kin_partial, R_6T] = setup.get_kin_partial();
[R_1, p_1] = fwdkin(setup.get_kin, S_given.Q);
[R_2, p_2] = fwdkin(kin_partial,   S_given.Q_partial);

[R_1      p_1]
[R_2*R_6T p_2]