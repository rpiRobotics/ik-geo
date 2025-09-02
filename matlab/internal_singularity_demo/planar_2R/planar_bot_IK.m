function [Q, is_LS] = planar_bot_IK(p_0T, L1, L2)
Q = [];
is_LS = [];
ex = [1;0;0];
ey = [0;1;0];
ez = [0;0;1];
zv = [0;0;0];

p_01 = zv;
p_12 = ex*L1;
p_2T = ex*L2;
h = ez;


[t2, t2_is_LS] = subproblem.sp_3(p_2T, -p_12, h, norm(p_0T-p_01));

% Duplicate LS result
t2 = [t2(1) t2(end)];

for i_t2 = 1:length(t2)
    q2 = t2(i_t2);
    R_12 = rot(h, q2);
    [q1, q1_is_LS] = subproblem.sp_1(p_12+R_12*p_2T, p_0T-p_01, h);

    Q = [Q [q1;q2]];
    is_LS = [is_LS [t2_is_LS; q1_is_LS]];
end

end