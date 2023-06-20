function [Q, is_LS_vec] = IK_3_parallel(R_06, p_0T, kin)
% h_2 = h_3 = h_4

P = kin.P;
H = kin.H;

p_06 = p_0T - P(:,1) - R_06*P(:,7);

% Sove for (q1, q5) using Subproblem 6
H_sp = [H(:,2) H(:,2) H(:,2) H(:,2)];
K_sp = [-H(:,1) H(:,5) -H(:,1) H(:,5)];
P_sp = [p_06 -P(:,6) R_06*H(:,6) -H(:,6)];
d1 = H(:,2)'* (P(:,3)+P(:,4)+P(:,5) + P(:,2));
d2 = 0;
[theta1, theta5] = subproblem.sp_6(H_sp, K_sp, P_sp, d1, d2);

Q = [];
is_LS_vec = [];
for i = 1:length(theta1)
    q_1 = theta1(i);
    q_5 = theta5(i);
    R_01 = rot(H(:,1), q_1);
    R_45 = rot(H(:,5), q_5);

    % solve for R_14 using Subproblem 1
    [theta_14, theta_14_is_LS] = subproblem.sp_1(R_45*H(:,6), R_01'*R_06*H(:,6), H(:,2));
    
    % solve for q6 using Subproblem 1
    [q_6, q_6_is_LS] = subproblem.sp_1(R_45'*H(:,2), R_06'*R_01*H(:,2), -H(:,6));

    % solve for q3 using Subproblem 3
    R_14 = rot(H(:,2), theta_14);
    d_inner = R_01'*p_06-P(:,2) - R_14*R_45*P(:,6)-R_14*P(:,5);
    d = norm(d_inner);
    [theta_3, theta_3_is_LS] = subproblem.sp_3(-P(:,4), P(:,3), H(:,2), d);

    for i_q3 = 1:length(theta_3)
        q_3 = theta_3(i_q3);
        % solve for q2 using Subproblem 1
        [q_2, q_2_is_LS] = subproblem.sp_1(P(:,3) + rot(H(:,2), q_3)*P(:,4), d_inner, H(:,2));

        % q4 by subtraction
        q_4 = wrapToPi(theta_14 - q_2 - q_3);
        
        q_i = [q_1; q_2; q_3; q_4; q_5; q_6];
        Q = [Q q_i];
        %is_LS_vec = [is_LS_vec theta_14_is_LS||theta_3_is_LS||q_2_is_LS||q_6_is_LS];
        is_LS_vec = [is_LS_vec [theta_14_is_LS theta_3_is_LS q_2_is_LS q_6_is_LS]'];
    end
end
end
