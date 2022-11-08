function [Q, is_LS] = ur5(R, T)
    [Q, is_LS] = IK.IK_3_parallel_2_intersecting(R, T, hardcoded_IK_setups.ur5.get_kin());
end