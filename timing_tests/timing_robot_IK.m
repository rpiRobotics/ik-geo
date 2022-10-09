N_trials = 100;

runtimes_gen_6_dof = run_timing_test(@IK_setups.IK_gen_6_dof.setup, {@IK_setups.IK_gen_6_dof.run_mex}, 10, 10, true)
%%
runtimes_IK_2_intersecting = run_timing_test(@IK_setups.IK_2_intersecting.setup, {@IK_setups.IK_2_intersecting.run_mex}, 10, 10, true)
%%
runtimes_IK_spherical = run_timing_test(@IK_setups.IK_spherical.setup, {@IK_setups.IK_spherical.run_mex}, N_trials, 50)
%%
runtimes_IK_spherical_2_intersecting = run_timing_test(@IK_setups.IK_spherical_2_intersecting.setup, {@IK_setups.IK_spherical_2_intersecting.run_mex}, N_trials, 40)
%%
runtimes_IK_spherical_2_parallel = run_timing_test(@IK_setups.IK_spherical_2_parallel.setup, {@IK_setups.IK_spherical_2_parallel.run_mex}, N_trials, 100)
%%
runtimes_IK_3_parallel = run_timing_test(@IK_setups.IK_3_parallel.setup, {@IK_setups.IK_3_parallel.run_mex}, N_trials, 100)
%%
runtimes_IK_3_parallel_2_intersecting = run_timing_test(@IK_setups.IK_3_parallel_2_intersecting.setup, {@IK_setups.IK_3_parallel_2_intersecting.run_mex}, N_trials, 40)
%% 
histogram(runtimes_gen_6_dof); hold on
histogram(runtimes_IK_2_intersecting); 
histogram(runtimes_IK_spherical); 
histogram(runtimes_IK_spherical_2_intersecting);
histogram(runtimes_IK_spherical_2_parallel);
histogram(runtimes_IK_3_parallel);
histogram(runtimes_IK_3_parallel_2_intersecting);
hold off
%histogram(runtimes_IK_2R_2R_3R); 
legend(["gen\_6\_dof", "2\_intersecting", "spherical","spherical\_2\_intersecting","spherical\_2\_parallel", "3\_parallel","3\_parallel\_2\_intersecting"])
set(gca,'XScale','log')
%%
runtimes_gen_6_dof_padded = NaN(100,1);
runtimes_gen_6_dof_padded(1:length(runtimes_gen_6_dof)) = runtimes_gen_6_dof;
runtimes_IK_2_intersecting_padded = NaN(100,1);
runtimes_IK_2_intersecting_padded(1:length(runtimes_IK_2_intersecting)) = runtimes_IK_2_intersecting;

all_robot_IK_timing = [runtimes_gen_6_dof_padded runtimes_IK_2_intersecting_padded runtimes_IK_spherical runtimes_IK_spherical_2_intersecting runtimes_IK_spherical_2_parallel runtimes_IK_3_parallel runtimes_IK_3_parallel_2_intersecting];
1e6 * mean(all_robot_IK_timing, 1, "omitnan")'
1e6 * median(all_robot_IK_timing, 1,"omitnan")'
1e6 * std(all_robot_IK_timing, 1, "omitnan")'