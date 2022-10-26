N_trials = 100;

%%
runtimes_sp1 = run_timing_test(@subproblem_setups.sp_1.setup, {@subproblem_setups.sp_1.run}, N_trials, 300)
%%
runtimes_sp2 = run_timing_test(@subproblem_setups.sp_2.setup, {@subproblem_setups.sp_2.run}, N_trials, 60)
%%
runtimes_sp2E = run_timing_test(@subproblem_setups.sp_2E.setup, {@subproblem_setups.sp_2E.run}, N_trials, 100)
%%
runtimes_sp3 = run_timing_test(@subproblem_setups.sp_3.setup, {@subproblem_setups.sp_3.run}, N_trials, 200)
%%
runtimes_sp4 = run_timing_test(@subproblem_setups.sp_4.setup, {@subproblem_setups.sp_4.run}, N_trials, 200)
%%
runtimes_sp5 = run_timing_test(@subproblem_setups.sp_5.setup, {@subproblem_setups.sp_5.run}, N_trials, 20)
%%
runtimes_sp6 = run_timing_test(@subproblem_setups.sp_6.setup, {@subproblem_setups.sp_6.run}, N_trials, 10)
%% 
histogram(runtimes_sp1); hold on
histogram(runtimes_sp2);
histogram(runtimes_sp2E);
histogram(runtimes_sp3);
histogram(runtimes_sp4);
histogram(runtimes_sp5);
histogram(runtimes_sp6); hold off
legend(["1", "2","2E","3","4","5","6"])
%set(gca,'XScale','log')
%xlabel("Runtimes (s)  - Log scale")
xlabel("Runtimes (s)")
ylabel("Frequency")
%%
all_subproblem_runtimes = [runtimes_sp1 runtimes_sp2 runtimes_sp2E runtimes_sp3 runtimes_sp4 runtimes_sp5 runtimes_sp6]
1e6 * mean(all_subproblem_runtimes)'
1e6 * median(all_subproblem_runtimes)'
1e6 * std(all_subproblem_runtimes)'

%% And now mex
runtimes_sp1_mex = run_timing_test(@subproblem_setups.sp_1.setup, {@subproblem_setups.sp_1.run_mex}, N_trials, 100)
%%
runtimes_sp2_mex = run_timing_test(@subproblem_setups.sp_2.setup, {@subproblem_setups.sp_2.run_mex}, N_trials, 60)
%%
runtimes_sp2E_mex = run_timing_test(@subproblem_setups.sp_2E.setup, {@subproblem_setups.sp_2E.run_mex}, N_trials, 100)
%%
runtimes_sp3_mex = run_timing_test(@subproblem_setups.sp_3.setup, {@subproblem_setups.sp_3.run_mex}, N_trials, 200)
%%
runtimes_sp4_mex = run_timing_test(@subproblem_setups.sp_4.setup, {@subproblem_setups.sp_4.run_mex}, N_trials, 200)
%%
runtimes_sp5_mex = run_timing_test(@subproblem_setups.sp_5.setup, {@subproblem_setups.sp_5.run_mex}, N_trials, 50)
%%
runtimes_sp6_mex = run_timing_test(@subproblem_setups.sp_6.setup, {@subproblem_setups.sp_6.run_mex}, N_trials, 50)
%% 
histogram(runtimes_sp1_mex); hold on
histogram(runtimes_sp2_mex);
histogram(runtimes_sp2E_mex);
histogram(runtimes_sp3_mex);
histogram(runtimes_sp4_mex);
histogram(runtimes_sp5_mex);
histogram(runtimes_sp6_mex); hold off
legend(["1", "2","2E","3","4","5","6"])
%set(gca,'XScale','log')
%xlabel("Runtimes (s)  - Log scale")
xlabel("Runtimes (s)")
ylabel("Frequency")
%%
all_subproblem_runtimes_mex = [runtimes_sp1_mex runtimes_sp2_mex runtimes_sp2E_mex runtimes_sp3_mex runtimes_sp4_mex runtimes_sp5_mex runtimes_sp6_mex]
1e6 * mean(all_subproblem_runtimes_mex)'
1e6 * median(all_subproblem_runtimes_mex)'
1e6 * std(all_subproblem_runtimes_mex)'