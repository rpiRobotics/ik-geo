%%
kin = hardcoded_IK_setups.yumi_fixed_q3.get_kin_partial();
generate_openrave_xml(kin, "yumi_fixed_q3");
%%
kin = hardcoded_IK_setups.RRC_fixed_q6.get_kin_partial();
generate_openrave_xml(kin, "RRC_fixed_q6");
%%
kin = hardcoded_IK_setups.two_parallel_bot.get_kin();
generate_openrave_xml(kin, "two_parallel_bot");
%%
kin = hardcoded_IK_setups.spherical_bot.get_kin();
generate_openrave_xml(kin, "spherical_bot");
%%
kin = hardcoded_IK_setups.KUKA_R800_fixed_q3.get_kin_partial();
generate_openrave_xml(kin, "KUKA_R800_fixed_q3");
%%
kin = hardcoded_IK_setups.IRB_6640.get_kin();
generate_openrave_xml(kin, "IRB_6640")
%%
kin = hardcoded_IK_setups.three_parallel_bot.get_kin;
generate_openrave_xml(kin, "three_parallel_bot");
%%
kin = hardcoded_IK_setups.ur5.get_kin;
generate_openrave_xml(kin, "ur5");