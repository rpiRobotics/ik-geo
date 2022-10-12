N_trials = 100;
%%
runtimes_ur5 = run_timing_test(@IK_ur5_setup.setup, {@IK_ur5_setup.run_mex}, N_trials, 100)
%%
runtimes_IRB_6640 = run_timing_test(@hardcoded_IK_setups.IRB_6640.setup, {@hardcoded_IK_setups.IRB_6640.run_mex}, N_trials, 100)
%%
runtimes_three_parallel_bot = run_timing_test(@hardcoded_IK_setups.three_parallel_bot.setup, {@hardcoded_IK_setups.three_parallel_bot.run_mex}, N_trials, 80)
%%
runtimes_KUKA = run_timing_test(@hardcoded_IK_setups.KUKA_R800_fixed_q3.setup, {@hardcoded_IK_setups.KUKA_R800_fixed_q3.run_mex}, N_trials, 100)
%%
runtimes_yumi = run_timing_test(@hardcoded_IK_setups.yumi_fixed_q3.setup, {@hardcoded_IK_setups.yumi_fixed_q3.run_mex}, 10, 10, true)
%%
runtimes_RRC = run_timing_test(@hardcoded_IK_setups.RRC_fixed_q6.setup, {@hardcoded_IK_setups.RRC_fixed_q6.run_mex}, N_trials, 40)