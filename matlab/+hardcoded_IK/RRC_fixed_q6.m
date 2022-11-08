function [Q, is_LS] = RRC_fixed_q6(R, T)
    [kin_partial, R_6T] = hardcoded_IK_setups.RRC_fixed_q6.get_kin_partial();
    [Q, is_LS] = IK.IK_2_intersecting(R*R_6T', T, kin_partial);
end