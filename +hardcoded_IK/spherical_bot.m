function [Q, is_LS] = spherical_bot(R, T)
    [Q, is_LS] = IK.IK_spherical(R, T, hardcoded_IK_setups.spherical_bot.get_kin());
end