% Check correctness of subproblem solutions when exact solution exists

setups = {
    subproblem_setups.sp_1
    subproblem_setups.sp_2
    subproblem_setups.sp_2E
    subproblem_setups.sp_3
    subproblem_setups.sp_4
    subproblem_setups.sp_5
    subproblem_setups.sp_6
};

%% Run once
errors = NaN(size(setups));
for i = 1:length(setups)
    errors(i) = correctness_test(setups{i});
end
errors

%% Run many times
N_trials = 1e4;
errors = NaN(numel(setups), N_trials);

for i = 1:length(setups)
    for j = 1:N_trials
        errors(i,j) = correctness_test(setups{i});
    end
end

max(errors')
mean(errors', 1, "omitnan")
sum(isnan(errors'))

histogram(errors(1,:)); hold on
histogram(errors(2,:)); 
histogram(errors(3,:));
histogram(errors(4,:));
histogram(errors(5,:));
histogram(errors(6,:));
histogram(errors(7,:));hold off
set(gca,'XScale','log')

function e = correctness_test(prob_setup)
P  = prob_setup.setup();
S_t = prob_setup.run(P);
e = prob_setup.error(P, S_t);
end