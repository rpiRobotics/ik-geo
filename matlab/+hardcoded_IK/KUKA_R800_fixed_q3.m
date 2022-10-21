function [Q, is_LS] = KUKA_R800_fixed_q3(R, T)
    [kin_partial, R_6T] = hardcoded_IK_setups.KUKA_R800_fixed_q3.get_kin_partial();
    [Q, is_LS] = IK.IK_spherical_2_intersecting(R*R_6T', T, kin_partial);
end