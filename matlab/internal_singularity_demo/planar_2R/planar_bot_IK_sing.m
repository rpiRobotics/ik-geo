function [Q, is_LS] = planar_bot_IK_sing(p_0T, L1, L2, e_sing)

Q = [];
is_LS = [];
ex = [1;0;0];
ey = [0;1;0];
ez = [0;0;1];
zv = [0;0;0];

p_12 = ex*L1;
p_2T = ex*L2;
h = ez;


[t2, t2_is_LS] = subproblem.sp_3(p_2T, -p_12, h, norm(p_0T));

% Duplicate LS result
t2 = [t2(1) t2(end)];

for i_t2 = 1:length(t2)
    q2 = t2(i_t2);
    R_12 = rot(h, q2);

    if norm(p_0T) == 0
        [t1, t1_is_LS] = subproblem.sp_4(cross(h,e_sing), R_12*p_2T, h, 0); % q_1 found with velocity information
        Q = [[t1(1); q2] [t1(2); q2]];
        is_LS = [t2_is_LS t2_is_LS; t1_is_LS t1_is_LS];
        return
    end
    
    [q1, q1_is_LS] = subproblem.sp_1(p_12+R_12*p_2T, p_0T, h);
    Q = [Q [q1;q2]];
    is_LS = [is_LS [t2_is_LS; q1_is_LS]];
end

end