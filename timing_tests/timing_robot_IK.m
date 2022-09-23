runtimes_IK_spherical_2_parallel = run_timing_test(@IK_setups.IK_spherical_2_parallel.setup, {@IK_setups.IK_spherical_2_parallel.run_mex}, 100, 10)

%%
runtimes_IK_3_parallel = run_timing_test(@IK_setups.IK_3_parallel.setup, {@IK_setups.IK_3_parallel.run_mex}, 100, 10)
%%
runtimes_IK_3_parallel_2_intersecting = run_timing_test(@IK_setups.IK_3_parallel_2_intersecting.setup, {@IK_setups.IK_3_parallel_2_intersecting.run_mex}, 100, 10)
%%
runtimes_IK_spherical_2_intersecting = run_timing_test(@IK_setups.IK_spherical_2_intersecting.setup, {@IK_setups.IK_spherical_2_intersecting.run_mex}, 100, 10)
%%
runtimes_IK_spherical = run_timing_test(@IK_setups.IK_spherical.setup, {@IK_setups.IK_spherical.run_mex}, 100, 10)
%%
runtimes_IK_2R_2R_3R = run_timing_test(@IK_setups.IK_2R_2R_3R.setup, {@IK_setups.IK_2R_2R_3R.run_mex}, 100, 10)

%% 
histogram(runtimes_IK_spherical_2_parallel); hold on
histogram(runtimes_IK_3_parallel);
histogram(runtimes_IK_3_parallel_2_intersecting);
histogram(runtimes_IK_spherical_2_intersecting);
histogram(runtimes_IK_spherical);
histogram(runtimes_IK_2R_2R_3R); hold off
legend(["spherical\_2\_parallel", "3\_parallel","3\_parallel\_2\_intersecting","spherical\_2\_intersecting","spherical","2R\_2R\_3R"])
%%
histogram(1./runtimes_IK_spherical_2_parallel); hold on
histogram(1./runtimes_IK_3_parallel);
histogram(runtimes_IK_3_parallel_2_intersecting);
histogram(1./runtimes_IK_spherical_2_intersecting);
histogram(1./runtimes_IK_spherical);
histogram(1./runtimes_IK_2R_2R_3R); hold off
legend(["spherical\_2\_parallel", "3\_parallel","3\_parallel\_2\_intersecting","spherical\_2\_intersecting","spherical","2R\_2R\_3R"])