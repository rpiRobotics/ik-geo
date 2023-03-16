clear all


% setup = subproblem_setups.sp_1;
% setup = subproblem_setups.sp_2;
% setup = subproblem_setups.sp_2E;
% setup = subproblem_setups.sp_3;
% setup = subproblem_setups.sp_4;
% setup = subproblem_setups.sp_5;
setup = subproblem_setups.sp_6;

class_name = string(class(setup)).split(".");
file_name = class_name(end) + ".mat";

load("../../../test_cases/"+file_name, "P_list", "S_list")
%%
codegen -report /test_sp_from_CSV_inner.m -args {P_list}

% codegen -report /test_mex_from_CSV_inner.m -args {P_list} -profile

%% m file
[T_avg, Q_testing] = test_sp_from_CSV_inner(P_list);
1e9*T_avg 

%% MEX file

[T_avg, Q_testing] = test_sp_from_CSV_inner_mex(P_list);
1e9*T_avg 


%% Profile
profile on;
[T_avg, Q_testing] = test_mex_from_CSV_inner_mex(P_list);
profile viewer;