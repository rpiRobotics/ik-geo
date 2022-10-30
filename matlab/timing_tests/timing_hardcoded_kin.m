N_trials = 100;
%%
runtimes_ur5 = run_timing_test(@hardcoded_IK_setups.ur5.setup, {@hardcoded_IK_setups.ur5.run_mex}, N_trials, 100)
%%
runtimes_IRB_6640 = run_timing_test(@hardcoded_IK_setups.IRB_6640.setup, {@hardcoded_IK_setups.IRB_6640.run_mex}, N_trials, 100)
%%
runtimes_three_parallel_bot = run_timing_test(@hardcoded_IK_setups.three_parallel_bot.setup, {@hardcoded_IK_setups.three_parallel_bot.run_mex}, N_trials, 80)
%%
runtimes_KUKA = run_timing_test(@hardcoded_IK_setups.KUKA_R800_fixed_q3.setup, {@hardcoded_IK_setups.KUKA_R800_fixed_q3.run_mex}, N_trials, 100)
%%
runtimes_yumi = run_timing_test(@hardcoded_IK_setups.yumi_fixed_q3.setup, {@hardcoded_IK_setups.yumi_fixed_q3.run_mex}, 10, 10, true)
%%
runtimes_RRC = run_timing_test(@hardcoded_IK_setups.RRC_fixed_q6.setup, {@hardcoded_IK_setups.RRC_fixed_q6.run_mex}, N_trials, 10)
%%
runtimes_spherical_bot = run_timing_test(@hardcoded_IK_setups.spherical_bot.setup, {@hardcoded_IK_setups.spherical_bot.run_mex}, N_trials, 50)

%%
mean(runtimes_yumi)
mean(runtimes_RRC)
mean(runtimes_spherical_bot)
mean(runtimes_KUKA)
mean(runtimes_IRB_6640)
mean(runtimes_three_parallel_bot)
mean(runtimes_ur5)

%histogram(runtimes_yumi);
histogram(runtimes_RRC);  hold on
histogram(runtimes_spherical_bot);
histogram(runtimes_KUKA);
histogram(runtimes_IRB_6640);
histogram(runtimes_three_parallel_bot);
histogram(runtimes_ur5); hold off
%legend(["YUMI", "RRC","Spherical-Bot","KUKA","IRB-6640","Three-Parallel-Bot","UR5"])
legend(["RRC","Spherical-Bot","KUKA","IRB-6640","Three-Parallel-Bot","UR5"])
%set(gca,'XScale','log')
%xlabel("Runtimes (s)  - Log scale")
xlabel("Runtimes (s)")
ylabel("Frequency")
