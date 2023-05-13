function [Q, is_LS] = two_parallel_bot(R, T)
    [Q, is_LS] = IK.IK_2_parallel(R, T, hardcoded_IK_setups.two_parallel_bot.get_kin());
end