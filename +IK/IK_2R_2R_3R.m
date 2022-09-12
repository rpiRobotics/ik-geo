function [Q, is_LS_vec] = IK_2R_2R_3R(R_07, p_0T, SEW_class, psi, kin)
Q = [];
is_LS_vec = [];

% Find wrist position
W = p_0T - R_07 * kin.P(:,8);

% Find shoulder position
S = kin.P(:,1);

% Use subproblem 3 to find theta_SEW
% || p_17 - R(n_SEW, theta) ||p_23|| e_17 || = || p_45 ||
d_S_E = norm(sum(kin.P(:,2:4), 2));
d_E_W = norm(sum(kin.P(:,5:7), 2));
p_17 = W-S;
e_17 = p_17/ norm(p_17);

[~, n_SEW] = SEW_class.inv_kin(S, W, psi);
[theta_SEW, theta_SEW_is_LS] = subproblem.sp_3(d_S_E*e_17, p_17, n_SEW, d_E_W);

% Pick theta_SEW > 0 so R_02 p_23 falls in the correct half plane
q_SEW = max(theta_SEW);
p_S_E = rot(n_SEW, q_SEW) * d_S_E*e_17;
E = p_S_E + S;

% Find q_1 and q_2 using subproblem 2
    h_1 = kin.H(:,1);
    h_2 = kin.H(:,2);
    p_S_E_0 = sum(kin.P(:,2:4), 2);


    [t1, t2, t12_is_ls] = subproblem.sp_2(p_S_E,p_S_E_0, -h_1, h_2);

for i_q12 = 1:length(t1)
    q1 = t1(i_q12);
    q2 = t2(i_q12);

    % Find q_3 and q_4 using subproblem 2
    h_3 = kin.H(:,3);
    h_4 = kin.H(:,4);
    p_E_W_0 = sum(kin.P(:,5:7), 2);
    p_E_W = W - E;
    
    R_2 = rot(h_1, q1) * rot(h_2, q2);

    [t3, t4, t34_is_ls] = subproblem.sp_2(R_2'*p_E_W,p_E_W_0, -h_3, h_4);

    for i_q34 = 1:length(t3)
        q3 = t3(i_q34);
        q4 = t4(i_q34);

        % Find q_5 and q_6 using subproblem 2
        h_5 = kin.H(:,5);
        h_6 = kin.H(:,6);
    
        R_4 = R_2*rot(h_3, q3) * rot(h_4, q4);
        [t5, t6, t56_is_ls] = subproblem.sp_2(R_4'*R_07*kin.H(:,7), kin.H(:,7), -h_5, h_6);

        for i_q56 = 1:length(t5)
            q5 = t5(i_q56);
            q6 = t6(i_q56);

            % Find q_7
            h_7 = kin.H(:,7);
            R_6 = R_4 * rot(h_5, q5) * rot(h_6, q6);
            [q7, q7_is_ls] = subproblem.sp_1(h_6, R_6'*R_07*h_6, h_7); 

            q_i = [q1; q2; q3; q4; q5; q6; q7];
            Q = [Q q_i];
            is_LS_vec = [is_LS_vec [theta_SEW_is_LS; t12_is_ls; t34_is_ls; t56_is_ls; q7_is_ls]];
        end
    end

end

end