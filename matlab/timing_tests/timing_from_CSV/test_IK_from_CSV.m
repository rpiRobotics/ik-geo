clear all

%% Load in problem and solution lists
dir = "../../../test_cases/";

lists.IK_gen_6_dof                  = load(dir+"IK_gen_6_dof.mat", "P_list", "S_list");
lists.IK_2_intersecting             = load(dir+"IK_2_intersecting.mat", "P_list", "S_list");
lists.IK_2_parallel                 = load(dir+"IK_2_parallel.mat", "P_list", "S_list");
lists.IK_spherical                  = load(dir+"IK_spherical.mat", "P_list", "S_list");
lists.IK_spherical_2_intersecting   = load(dir+"IK_spherical_2_intersecting.mat", "P_list", "S_list");
lists.IK_spherical_2_parallel       = load(dir+"IK_spherical_2_parallel.mat", "P_list", "S_list");
lists.IK_3_parallel                 = load(dir+"IK_3_parallel.mat", "P_list", "S_list");
lists.IK_3_parallel_2_intersecting  = load(dir+"IK_3_parallel_2_intersecting.mat", "P_list", "S_list");

lists.yumi_fixed_q3         = load(dir+"yumi_fixed_q3.mat", "P_list", "S_list");
lists.RRC_fixed_q6          = load(dir+"RRC_fixed_q6.mat", "P_list", "S_list");
lists.two_parallel_bot      = load(dir+"two_parallel_bot.mat", "P_list", "S_list");
lists.spherical_bot         = load(dir+"spherical_bot.mat", "P_list", "S_list");
lists.KUKA_R800_fixed_q3    = load(dir+"KUKA_R800_fixed_q3.mat", "P_list", "S_list");
lists.IRB_6640              = load(dir+"IRB_6640.mat", "P_list", "S_list");
lists.three_parallel_bot    = load(dir+"three_parallel_bot.mat", "P_list", "S_list");
lists.ur5                   = load(dir+"ur5.mat", "P_list", "S_list");
%% Generate MEX files
cd ./+test_timing/

codegen -report /+test_timing/IK_gen_6_dof.m -args {lists.IK_gen_6_dof.P_list}
codegen -report /+test_timing/IK_2_intersecting.m -args {lists.IK_2_intersecting.P_list}
codegen -report /+test_timing/IK_2_parallel.m -args {lists.IK_2_parallel.P_list}
codegen -report /+test_timing/IK_spherical.m -args {lists.IK_spherical.P_list}
codegen -report /+test_timing/IK_spherical_2_intersecting.m -args {lists.IK_spherical_2_intersecting.P_list}
codegen -report /+test_timing/IK_spherical_2_parallel.m -args {lists.IK_spherical_2_parallel.P_list}
codegen -report /+test_timing/IK_3_parallel.m -args {lists.IK_3_parallel.P_list}
codegen -report /+test_timing/IK_3_parallel_2_intersecting.m -args {lists.IK_3_parallel_2_intersecting.P_list}

codegen -report /+test_timing/yumi_fixed_q3.m -args {lists.yumi_fixed_q3.P_list}
codegen -report /+test_timing/RRC_fixed_q6.m -args {lists.RRC_fixed_q6.P_list}
codegen -report /+test_timing/two_parallel_bot.m -args {lists.two_parallel_bot.P_list}
codegen -report /+test_timing/spherical_bot.m -args {lists.spherical_bot.P_list}
codegen -report /+test_timing/KUKA_R800_fixed_q3.m -args {lists.KUKA_R800_fixed_q3.P_list}
codegen -report /+test_timing/IRB_6640.m -args {lists.IRB_6640.P_list}
codegen -report /+test_timing/three_parallel_bot.m -args {lists.three_parallel_bot.P_list}
codegen -report /+test_timing/ur5.m -args {lists.ur5.P_list}

codegen -report /+test_timing/spherical_bot_MRT.m -args {lists.spherical_bot.P_list}
codegen -report /+test_timing/IRB_6640_MRT.m -args {lists.IRB_6640.P_list}

cd ../
%%
N = 10;
T = NaN(N,1);
%% m-files  
% for i = 1:N; [T(i), Q_testing] = test_timing.IK_gen_6_dof(lists.IK_gen_6_dof.P_list); end;                                  plot(1e6*T); vpa(1e6*min(T))
[T, Q_testing] = test_timing.IK_gen_6_dof(lists.IK_gen_6_dof.P_list); vpa(1e6*T) % N = 100
% for i = 1:N; [T(i), Q_testing] = test_timing.IK_2_intersecting(lists.IK_2_intersecting.P_list); end;                        plot(1e6*T); vpa(1e6*min(T))
[T, Q_testing] = test_timing.IK_2_intersecting(lists.IK_2_intersecting.P_list); vpa(1e6*T) % N=1000
% for i = 1:N; [T(i), Q_testing] = test_timing.IK_2_parallel(lists.IK_2_parallel.P_list); end;                                plot(1e6*T); vpa(1e6*min(T))
[T, Q_testing] = test_timing.IK_2_parallel(lists.IK_2_parallel.P_list); vpa(1e6*T) % N=1000
for i = 1:N; [T(i), Q_testing] = test_timing.IK_spherical(lists.IK_spherical.P_list); disp(i); end;                         plot(1e6*T); vpa(1e6*min(T))
for i = 1:N; [T(i), Q_testing] = test_timing.IK_spherical_2_intersecting(lists.IK_spherical_2_intersecting.P_list); disp(i); end;    plot(1e6*T); vpa(1e6*min(T))
for i = 1:N; [T(i), Q_testing] = test_timing.IK_spherical_2_parallel(lists.IK_spherical_2_parallel.P_list); disp(i); end;            plot(1e6*T); vpa(1e6*min(T))
for i = 1:N; [T(i), Q_testing] = test_timing.IK_3_parallel(lists.IK_3_parallel.P_list); disp(i); end;                                plot(1e6*T); vpa(1e6*min(T))
for i = 1:N; [T(i), Q_testing] = test_timing.IK_3_parallel_2_intersecting(lists.IK_3_parallel_2_intersecting.P_list); disp(i); end;  plot(1e6*T); vpa(1e6*min(T))

% for i = 1:N; [T(i), Q_testing] = test_timing.yumi_fixed_q3(lists.yumi_fixed_q3.P_list); disp(i); end;            plot(1e6*T); vpa(1e6*min(T))
[T, Q_testing] = test_timing.yumi_fixed_q3(lists.yumi_fixed_q3.P_list); vpa(1e6*T) % N = 100
% for i = 1:N; [T(i), Q_testing] = test_timing.RRC_fixed_q6(lists.RRC_fixed_q6.P_list); disp(i); end;              plot(1e6*T); vpa(1e6*min(T))
[T, Q_testing] = test_timing.RRC_fixed_q6(lists.RRC_fixed_q6.P_list); vpa(1e6*T) % N = 1000
% for i = 1:N; [T(i), Q_testing] = test_timing.two_parallel_bot(lists.two_parallel_bot.P_list); disp(i); end;      plot(1e6*T); vpa(1e6*min(T))
[T, Q_testing] = test_timing.two_parallel_bot(lists.two_parallel_bot.P_list); vpa(1e6*T) % N = 1000

for i = 1:N; [T(i), Q_testing] = test_timing.spherical_bot(lists.spherical_bot.P_list); disp(i); end;            plot(1e6*T); vpa(1e6*min(T))
for i = 1:N; [T(i), Q_testing] = test_timing.KUKA_R800_fixed_q3(lists.KUKA_R800_fixed_q3.P_list); disp(i); end;  plot(1e6*T); vpa(1e6*min(T))
for i = 1:N; [T(i), Q_testing] = test_timing.IRB_6640(lists.IRB_6640.P_list); disp(i); end;                      plot(1e6*T); vpa(1e6*min(T))
for i = 1:N; [T(i), Q_testing] = test_timing.three_parallel_bot(lists.three_parallel_bot.P_list); disp(i); end;  plot(1e6*T); vpa(1e6*min(T))
for i = 1:N; [T(i), Q_testing] = test_timing.ur5(lists.ur5.P_list); disp(i); end;                                plot(1e6*T); vpa(1e6*min(T))

% for i = 1:N; [T(i), Q_testing] = test_timing.spherical_bot_MRT(lists.spherical_bot.P_list); end;            plot(1e6*T); vpa(1e6*min(T))
% for i = 1:N; [T(i), Q_testing] = test_timing.IRB_6640_MRT(lists.IRB_6640.P_list); end;                      plot(1e6*T); vpa(1e6*min(T))
%% MEX files
% for i = 1:N; [T(i), Q_testing] = test_timing.IK_gen_6_dof_mex(lists.IK_gen_6_dof.P_list); disp(i); end;                         plot(1e6*T); vpa(1e6*min(T))
[T, Q_testing] = test_timing.IK_gen_6_dof_mex(lists.IK_gen_6_dof.P_list); vpa(1e6*T)
% for i = 1:N; [T(i), Q_testing] = test_timing.IK_2_intersecting_mex(lists.IK_2_intersecting.P_list); end;                        plot(1e6*T); vpa(1e6*min(T))
[T, Q_testing] = test_timing.IK_2_intersecting_mex(lists.IK_2_intersecting.P_list); vpa(1e6*T)
% for i = 1:N; [T(i), Q_testing] = test_timing.IK_2_parallel_mex(lists.IK_2_parallel.P_list); end;                                plot(1e6*T); vpa(1e6*min(T))
[T, Q_testing] = test_timing.IK_2_parallel_mex(lists.IK_2_parallel.P_list); vpa(1e6*T)
for i = 1:N; [T(i), Q_testing] = test_timing.IK_spherical_mex(lists.IK_spherical.P_list); end;                                  plot(1e6*T); vpa(1e6*min(T))
for i = 1:N; [T(i), Q_testing] = test_timing.IK_spherical_2_intersecting_mex(lists.IK_spherical_2_intersecting.P_list); end;    plot(1e6*T); vpa(1e6*min(T))
for i = 1:N; [T(i), Q_testing] = test_timing.IK_spherical_2_parallel_mex(lists.IK_spherical_2_parallel.P_list); end;            plot(1e6*T); vpa(1e6*min(T))
for i = 1:N; [T(i), Q_testing] = test_timing.IK_3_parallel_mex(lists.IK_3_parallel.P_list); end;                                plot(1e6*T); vpa(1e6*min(T))
for i = 1:N; [T(i), Q_testing] = test_timing.IK_3_parallel_2_intersecting_mex(lists.IK_3_parallel_2_intersecting.P_list); end;  plot(1e6*T); vpa(1e6*min(T))

% for i = 1:N; [T(i), Q_testing] = test_timing.yumi_fixed_q3_mex(lists.yumi_fixed_q3.P_list); end;            plot(1e6*T); vpa(1e6*min(T))
[T, Q_testing] = test_timing.yumi_fixed_q3_mex(lists.yumi_fixed_q3.P_list); vpa(1e6*T)
% for i = 1:N; [T(i), Q_testing] = test_timing.RRC_fixed_q6_mex(lists.RRC_fixed_q6.P_list); end;              plot(1e6*T); vpa(1e6*min(T))
[T, Q_testing] = test_timing.RRC_fixed_q6_mex(lists.RRC_fixed_q6.P_list); vpa(1e6*T)
% for i = 1:N; [T(i), Q_testing] = test_timing.two_parallel_bot_mex(lists.two_parallel_bot.P_list); end;      plot(1e6*T); vpa(1e6*min(T))
[T, Q_testing] = test_timing.two_parallel_bot_mex(lists.two_parallel_bot.P_list); vpa(1e6*T)
for i = 1:N; [T(i), Q_testing] = test_timing.spherical_bot_mex(lists.spherical_bot.P_list); end;            plot(1e6*T); vpa(1e6*min(T))
for i = 1:N; [T(i), Q_testing] = test_timing.KUKA_R800_fixed_q3_mex(lists.KUKA_R800_fixed_q3.P_list); end;  plot(1e6*T); vpa(1e6*min(T))
for i = 1:N; [T(i), Q_testing] = test_timing.IRB_6640_mex(lists.IRB_6640.P_list); end;                      plot(1e6*T); vpa(1e6*min(T))
for i = 1:N; [T(i), Q_testing] = test_timing.three_parallel_bot_mex(lists.three_parallel_bot.P_list); end;  plot(1e6*T); vpa(1e6*min(T))
for i = 1:N; [T(i), Q_testing] = test_timing.ur5_mex(lists.ur5.P_list); end;                                plot(1e6*T); vpa(1e6*min(T))

for i = 1:N; [T(i), Q_testing] = test_timing.spherical_bot_MRT_mex(lists.spherical_bot.P_list); end;            plot(1e6*T); vpa(1e6*min(T))
for i = 1:N; [T(i), Q_testing] = test_timing.IRB_6640_MRT_mex(lists.IRB_6640.P_list); end;                      plot(1e6*T); vpa(1e6*min(T))
%% Profile

%codegen -report /test_IK_from_CSV_inner.m -args {P_list} -profile

profile on;
[T_avg, Q_testing] = test_mex_from_CSV_inner_mex(P_list);
profile viewer;