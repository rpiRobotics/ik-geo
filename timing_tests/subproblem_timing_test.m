N_trials = 100;

disp("Starting SP1")
runtimes_sp1 = run_timing_test(@subproblem_setups.sp_1.setup, {@subproblem_setups.sp_1.run}, N_trials, 300)
disp("Starting SP2")
runtimes_sp2 = run_timing_test(@subproblem_setups.sp_2.setup, {@subproblem_setups.sp_2.run}, N_trials, 60)
disp("Starting SP2E")
runtimes_sp2E = run_timing_test(@subproblem_setups.sp_2E.setup, {@subproblem_setups.sp_2E.run}, N_trials, 100)
disp("Starting SP3")
runtimes_sp3 = run_timing_test(@subproblem_setups.sp_3.setup, {@subproblem_setups.sp_3.run}, N_trials, 100)
disp("Starting SP4")
runtimes_sp4 = run_timing_test(@subproblem_setups.sp_4.setup, {@subproblem_setups.sp_4.run}, N_trials, 200)
disp("Starting SP5")
runtimes_sp5 = run_timing_test(@subproblem_setups.sp_5.setup, {@subproblem_setups.sp_5.run}, N_trials, 20)
disp("Starting SP6")
runtimes_sp6 = run_timing_test(@subproblem_setups.sp_6.setup, {@subproblem_setups.sp_6.run}, N_trials, 10)

histogram(runtimes_sp1); hold on
histogram(runtimes_sp2);
histogram(runtimes_sp2E);
histogram(runtimes_sp3);
histogram(runtimes_sp4);
histogram(runtimes_sp5);
histogram(runtimes_sp6); hold off
legend(["1", "2","2E","3","4","5","6"])
set(gca,'XScale','log')

all_runtimes = [runtimes_sp1 runtimes_sp2 runtimes_sp2E runtimes_sp3 runtimes_sp4 runtimes_sp5 runtimes_sp6]
1e6 * mean(all_runtimes)'
1e6 * median(all_runtimes)'
1e6 * std(all_runtimes)'