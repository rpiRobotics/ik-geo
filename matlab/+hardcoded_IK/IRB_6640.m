function [Q, is_LS] = IRB_6640(R, T)
    [Q, is_LS] = IK.IK_spherical_2_parallel(R, T, hardcoded_IK_setups.IRB_6640.get_kin());
end