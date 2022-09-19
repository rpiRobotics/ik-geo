setups = {
    subproblem_setups.sp_1
    subproblem_setups.sp_2
    subproblem_setups.sp_2E
    subproblem_setups.sp_3
    subproblem_setups.sp_4
    subproblem_setups.sp_5
    subproblem_setups.sp_6
};

errors = NaN(size(setups));
for i = 1:length(setups)
    errors(i) = correctness_test(setups{i});
end
errors

function e = correctness_test(prob_setup)

P  = prob_setup.setup();

% S_t = prob_setup.run(P);
S_t = prob_setup.run_mex(P);


e = prob_setup.error(P, S_t);
end