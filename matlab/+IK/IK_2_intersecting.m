function [Q, is_LS_vec] = IK_2_intersecting(R_06, p_0T, kin)
% Axes 5 and 6 intersect
Q = [];
is_LS_vec = [];

p_06 = p_0T - kin.P(:,1) - R_06*kin.P(:,7);

[q4_vec, soln_num_vec] = search_1D( ...
    @(q4)(alignment_err_given_q4(q4, p_06, R_06, kin)), ...
    -pi, pi, 200, false);

for i_q4 = 1:length(q4_vec)
    [e, Q_partial] = alignment_err_given_q4(q4_vec(i_q4), p_06, R_06, kin);
    q_partial = Q_partial(:,soln_num_vec(:,i_q4));

    R_04 = rot(kin.H(:,1),q_partial(1)) * rot(kin.H(:,2),q_partial(2)) ...
         * rot(kin.H(:,3),q_partial(3)) * rot(kin.H(:,4),q_partial(4));
    [q5, q5_is_LS] = subproblem.sp_1(kin.H(:,6), R_04'*R_06*kin.H(:,6),  kin.H(:,5));
    [q6, q6_is_LS] = subproblem.sp_1(kin.H(:,5), R_06'*R_04*kin.H(:,5), -kin.H(:,6));
    q_i = [q_partial; q5; q6];
    Q = [Q q_i];
    is_LS_vec = [is_LS_vec [q5_is_LS; q6_is_LS; e(soln_num_vec(:,i_q4))] ];
end

end

function [e_vec, Q_partial] = alignment_err_given_q4(q4, p_16, R_06, kin)
    e_vec = NaN([1 4]);
    Q_partial = NaN([4 4]);
    % find up to 4 solutions of (q1, q2, q3) using Subproblem 5
    p_35_3 = kin.P(:,4) + rot(kin.H(:,4), q4)*kin.P(:,5);
    
    [t1, t2, t3] = subproblem.sp_5( ...
                   -kin.P(:,2), p_16, kin.P(:,3), p_35_3, ...
                   -kin.H(:,1), kin.H(:,2), kin.H(:,3));

    for i_q123 = 1:length(t1)
        q1 = t1(i_q123);
        q2 = t2(i_q123);
        q3 = t3(i_q123);

        R_04 = rot(kin.H(:,1),q1) * rot(kin.H(:,2),q2) ...
             * rot(kin.H(:,3),q3) * rot(kin.H(:,4),q4);

        e_i = kin.H(:,5)' * R_04' * R_06* kin.H(:,6) - kin.H(:,5)'*kin.H(:,6);
        e_vec(i_q123) = e_i;
        Q_partial(:,i_q123) = [q1; q2; q3; q4];
    end
end
