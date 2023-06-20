function [Q, is_LS_vec] = IK_2_parallel(R_06, p_0T, kin)
% h_2 h_3 parallel

Q = [];
is_LS_vec = [];

p_06 = p_0T - kin.P(:,1) - R_06*kin.P(:,7);

[q1_vec, soln_num_vec] = search_1D(@error_given_q1, -pi, pi, 200, false);
% [q1_vec, soln_num_vec] = search_1D(@error_given_q1, -pi, pi, 1000, true);

for i_q1 = 1:length(q1_vec)
    q1 = q1_vec(i_q1);
    [e_vec, t4, t6]  = error_given_q1(q1);
    if isnan(e_vec(soln_num_vec(i_q1)))
        continue % Odd edge case, TODO figure out why it happens
    end
    e_i = e_vec(soln_num_vec(i_q1));
    q4 = t4(soln_num_vec(i_q1));
    q6 = t6(soln_num_vec(i_q1));
    R_01 = rot(kin.H(:,1), q1);
    R_34 = rot(kin.H(:,4), q4);
    R_56 = rot(kin.H(:,6), q6);
    
    [t23, t23_is_LS] = subproblem.sp_1(R_34*kin.H(:,5), R_01'*R_06*R_56'*kin.H(:,5), kin.H(:,2));
    R_13 = rot(kin.H(:,2), t23);

    [q2, q2_is_ls] = subproblem.sp_1(kin.P(:,3), ...
        R_01'*p_06 - kin.P(:,2) - R_13*kin.P(:,4) - R_13*R_34*kin.P(:,5) - R_01'*R_06*R_56'*kin.P(:,6), ...
        kin.H(:,2));
    [q5, q5_is_LS] = subproblem.sp_1(R_34'*kin.H(:,2), R_56 * R_06' * R_01 * kin.H(:,2), -kin.H(:,5));
    q3 = wrapToPi(t23 - q2);

    q_i = [q1 q2 q3 q4 q5 q6]';
    Q = [Q q_i];
    is_LS_vec = [is_LS_vec [e_i; t23_is_LS; q2_is_ls; q5_is_LS]];
end

function [e, t4, t6]  = error_given_q1(q1)
    e = NaN([1 4]);
    R_01 = rot(kin.H(:,1), q1);

    h1 = (kin.H(:,2)'*R_01'*R_06)';
    h2 = kin.H(:,2);
    h3 = h1;
    h4 = h2; % no negative here
    sp_H = [h1 h2 h3 h4];
    sp_K = [-kin.H(:,6) kin.H(:,4) -kin.H(:,6) kin.H(:,4)];
    sp_P = [kin.P(:,6) kin.P(:,5) kin.H(:,5) -kin.H(:,5)]; % negative is here
    d1 = kin.H(:,2)'*(R_01'*p_06 - kin.P(:,2) - kin.P(:,3) - kin.P(:,4));
    d2 = 0;
    [t6, t4] = subproblem.sp_6(sp_H, sp_K, sp_P, d1, d2);
    for i_46 = 1:length(t4)
        R_34 = rot(kin.H(:,4), t4(i_46));
        R_56 = rot(kin.H(:,6), t6(i_46));
        t23 = subproblem.sp_1(R_34*kin.H(:,5), R_01'*R_06*R_56'*kin.H(:,5), kin.H(:,2));
        R_13 = rot(kin.H(:,2), t23);

        e(i_46) = ...
            norm(R_01'*p_06 - kin.P(:,2) - R_13*kin.P(:,4) - R_13*R_34*kin.P(:,5) - R_01'*R_06*R_56'*kin.P(:,6))...
            - norm(kin.P(:,3));
    end
end

end