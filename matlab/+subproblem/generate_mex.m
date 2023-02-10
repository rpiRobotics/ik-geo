% Generate MEX files using MATLAB coder

% TODO change path

setups = {
    subproblem_setups.sp_1
    subproblem_setups.sp_2
    subproblem_setups.sp_2E
    subproblem_setups.sp_3
    subproblem_setups.sp_4
    subproblem_setups.sp_5
    subproblem_setups.sp_6
};

for i = 1:length(setups)
    setups{i}.generate_mex()
end