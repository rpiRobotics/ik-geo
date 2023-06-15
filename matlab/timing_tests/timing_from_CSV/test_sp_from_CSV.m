clear all

%% Load in problem and solution lists
dir = "../../../test_cases/";

lists.sp_1 = load(dir+"sp_1.mat", "P_list", "S_list");
lists.sp_2 = load(dir+"sp_2.mat", "P_list", "S_list");
lists.sp_3 = load(dir+"sp_3.mat", "P_list", "S_list");
lists.sp_4 = load(dir+"sp_4.mat", "P_list", "S_list");
lists.sp_5 = load(dir+"sp_5.mat", "P_list", "S_list");
lists.sp_6 = load(dir+"sp_6.mat", "P_list", "S_list");

%% Generate MEX files
cd ./+test_timing/

codegen -report /+test_timing/sp_1.m -args {lists.sp_1.P_list}
codegen -report /+test_timing/sp_2.m -args {lists.sp_2.P_list}
codegen -report /+test_timing/sp_3.m -args {lists.sp_3.P_list}
codegen -report /+test_timing/sp_4.m -args {lists.sp_4.P_list}
codegen -report /+test_timing/sp_5.m -args {lists.sp_5.P_list}
codegen -report /+test_timing/sp_6.m -args {lists.sp_6.P_list}

cd ../
%%
N = 100;
T = NaN(N,1);
%% m-files
for i = 1:N; [T(i), Q_testing] = test_timing.sp_1(lists.sp_1.P_list); end; plot(1e9*T); vpa(1e9*min(T))
for i = 1:N; [T(i), Q_testing] = test_timing.sp_2(lists.sp_2.P_list); end; plot(1e9*T); vpa(1e9*min(T))
for i = 1:N; [T(i), Q_testing] = test_timing.sp_3(lists.sp_3.P_list); end; plot(1e9*T); vpa(1e9*min(T))
for i = 1:N; [T(i), Q_testing] = test_timing.sp_4(lists.sp_4.P_list); end; plot(1e9*T); vpa(1e9*min(T))
for i = 1:N; [T(i), Q_testing] = test_timing.sp_5(lists.sp_5.P_list); disp(i); end; plot(1e9*T); vpa(1e9*min(T))
for i = 1:N; [T(i), Q_testing] = test_timing.sp_6(lists.sp_6.P_list); disp(i); end; plot(1e9*T); vpa(1e9*min(T))

%% MEX files
for i = 1:N; [T(i), Q_testing] = test_timing.sp_1_mex(lists.sp_1.P_list); end; plot(1e9*T); vpa(1e9*min(T))
for i = 1:N; [T(i), Q_testing] = test_timing.sp_2_mex(lists.sp_2.P_list); end; plot(1e9*T); vpa(1e9*min(T))
for i = 1:N; [T(i), Q_testing] = test_timing.sp_3_mex(lists.sp_3.P_list); end; plot(1e9*T); vpa(1e9*min(T))
for i = 1:N; [T(i), Q_testing] = test_timing.sp_4_mex(lists.sp_4.P_list); end; plot(1e9*T); vpa(1e9*min(T))
for i = 1:N; [T(i), Q_testing] = test_timing.sp_5_mex(lists.sp_5.P_list); end; plot(1e9*T); vpa(1e9*min(T))
for i = 1:N; [T(i), Q_testing] = test_timing.sp_6_mex(lists.sp_6.P_list); end; plot(1e9*T); vpa(1e9*min(T))

%% Profile
% codegen -report /test_mex_from_CSV_inner.m -args {P_list} -profile
profile on;
[T_avg, Q_testing] = test_mex_from_CSV_inner_mex(P_list);
profile viewer;