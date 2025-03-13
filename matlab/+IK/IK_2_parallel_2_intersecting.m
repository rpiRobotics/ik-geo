function [Q, is_LS_vec] = IK_2_parallel_2_intersecting(R_06, p_0T, kin)
% h_2 h_3 parallel
% h_4 and h_5 intersecting
% e.g. ABB GoFa

Q = [];
is_LS_vec = [];

p_06 = p_0T - kin.P(:,1) - R_06*kin.P(:,7);

% [q1_vec, soln_num_vec] = search_1D(@(q1)error_given_q1(q1, R_06, p_06, kin), -pi, pi, 200, true);
[q1_vec, soln_num_vec] = search_1D(@(q1)error_given_q1(q1, R_06, p_06, kin), -pi, pi, 2000, true);
% [q1_vec, soln_num_vec] = search_1D(@(q1)error_given_q1(q1, R_06, p_06, kin), -pi, pi, 1e6, false);

for i_q1 = 1:length(q1_vec)
    q1 = q1_vec(i_q1);
    [e_vec, t4, t6]  = error_given_q1(q1, R_06, p_06, kin);
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

end

function [e, Q4, Q6]  = error_given_q1(q1, R_06, p_06, kin)
    e = NaN([1 4]);
    R_01 = rot(kin.H(:,1), q1);
    i_soln = 1;
    Q4 = NaN([4 1]);
    Q6 = NaN([4 1]);
    e = NaN([4 1]);

    % Solve for q6 using subproblem 4
    [t6, t6_is_LS] = subproblem.sp_4((kin.H(:,2)'*R_01'*R_06)', kin.P(:,6), -kin.H(:,6), kin.H(:,2)'*(R_01'*p_06 - kin.P(:,2) - kin.P(:,3) - kin.P(:,4)));
    if t6_is_LS
        return
    end
    for i_t6 = 1:length(t6)
        q6 = t6(i_t6);
        R_56 = rot(kin.H(:,6), q6);

        % Solve for q4 using subproblem 4
        [t4, t4_is_LS] = subproblem.sp_4(kin.H(:,2), kin.H(:,5), kin.H(:,4), kin.H(:,2)'*R_01'*R_06*R_56'*kin.H(:,5));
        if t4_is_LS
            i_soln = i_soln+2;
            continue
        end

        for i_t4 = 1:length(t4)
            q4 = t4(i_t4);
            R_34 = rot(kin.H(:,4), q4);
            t23 = subproblem.sp_1(R_34*kin.H(:,5), R_01'*R_06*R_56'*kin.H(:,5), kin.H(:,2));
            R_13 = rot(kin.H(:,2), t23);

            Q4(i_soln) = q4;
            Q6(i_soln) = q6;
            e(i_soln) = ...
                norm(R_01'*p_06 - kin.P(:,2) - R_13*kin.P(:,4) - R_13*R_34*kin.P(:,5) - R_01'*R_06*R_56'*kin.P(:,6))...
                - norm(kin.P(:,3));
            i_soln = i_soln+1;
        end
    end
end