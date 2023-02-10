function [Q, is_LS] = three_parallel_bot(R, T)
    [Q, is_LS] = IK.IK_3_parallel(R, T, hardcoded_IK_setups.three_parallel_bot.get_kin());
end