% TODO change path

P = hardcoded_IK_setups.ur5.setup;
codegen -report +hardcoded_IK/ur5.m -args {P.R, P.T}
%%
P = hardcoded_IK_setups.IRB_6640.setup;
codegen -report +hardcoded_IK/IRB_6640.m -args {P.R, P.T}
%%
P = hardcoded_IK_setups.three_parallel_bot.setup;
codegen -report +hardcoded_IK/three_parallel_bot.m -args {P.R, P.T}
%%
P = hardcoded_IK_setups.KUKA_R800_fixed_q3.setup;
codegen -report +hardcoded_IK\KUKA_R800_fixed_q3.m -args {P.R, P.T}
%%
P = hardcoded_IK_setups.yumi_fixed_q3.setup;
codegen -report +hardcoded_IK\yumi_fixed_q3.m -args {P.R, P.T}
%%
P = hardcoded_IK_setups.RRC_fixed_q6.setup;
codegen -report +hardcoded_IK\RRC_fixed_q6.m -args {P.R, P.T}
%%
P = hardcoded_IK_setups.spherical_bot.setup;
codegen -report +hardcoded_IK\spherical_bot.m -args {P.R, P.T}