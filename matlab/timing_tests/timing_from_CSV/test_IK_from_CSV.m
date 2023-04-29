clear all

% setup = IK_setups.IK_gen_6_dof;
% setup = IK_setups.IK_2_intersecting;
% setup = IK_setups.IK_2_parallel;
% setup = IK_setups.IK_spherical;
% setup = IK_setups.IK_spherical_2_intersecting;
% setup = IK_setups.IK_spherical_2_parallel;
% setup = IK_setups.IK_3_parallel;
% setup = IK_setups.IK_3_parallel_2_intersecting;

% setup = hardcoded_IK_setups.yumi_fixed_q3;
setup = hardcoded_IK_setups.RRC_fixed_q6;
% setup = hardcoded_IK_setups.two_parallel_bot;
% setup = hardcoded_IK_setups.spherical_bot;
% setup = hardcoded_IK_setups.KUKA_R800_fixed_q3;
% setup = hardcoded_IK_setups.IRB_6640;
% setup = hardcoded_IK_setups.three_parallel_bot;
% setup = hardcoded_IK_setups.ur5;


class_name = string(class(setup)).split(".");
file_name = class_name(end) + ".mat";

load("../../../test_cases/"+file_name, "P_list", "S_list")
%% Use MATLAB coder to generate MEX code
codegen -report /test_IK_from_CSV_inner.m -args {P_list}

%% Test m (MATLAB) file

[T_avg, Q_testing] = test_IK_from_CSV_inner(P_list);
1e6*T_avg

%% Test MEX (compiled) file

[T_avg, Q_testing] = test_IK_from_CSV_inner_mex(P_list);
1e6*T_avg

%% Profile

%codegen -report /test_IK_from_CSV_inner.m -args {P_list} -profile

profile on;
[T_avg, Q_testing] = test_mex_from_CSV_inner_mex(P_list);
profile viewer;