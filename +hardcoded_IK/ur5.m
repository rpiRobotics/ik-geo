function [Q, is_LS] = ur5(R, T)
    [Q, is_LS] = IK.IK_3_parallel_2_intersecting(R, T, IK_ur5_setup.get_kin());
end