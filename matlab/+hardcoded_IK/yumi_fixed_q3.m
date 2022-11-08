function [Q, is_LS] = yumi_fixed_q3(R, T)
    [kin_partial, R_6T] = hardcoded_IK_setups.yumi_fixed_q3.get_kin_partial();
    [Q, is_LS] = IK.IK_gen_6_dof(R*R_6T', T, kin_partial);
end