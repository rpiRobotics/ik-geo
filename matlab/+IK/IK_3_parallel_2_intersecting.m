function [Q, is_LS_vec] = IK_3_parallel_2_intersecting(R_06, p_0T, kin)
% h_2 = h_3 = h_4 and p_56 = 0

P = kin.P;
H = kin.H;
Q = [];
is_LS_vec = [];

p_06 = p_0T - P(:,1) - R_06*P(:,7);

% Find q1 using Subproblem 4
% h_2' R_01' p_16 = h2' (p_12 + p_23 + p_34 + p_45)
[theta1, theta1_is_ls] = subproblem.sp_4(H(:,2), p_06, -H(:,1), H(:,2)'*sum(P(:,2:5), 2));

for i_t1 = 1:length(theta1)
    q_1 = theta1(i_t1);
    R_01 = rot(H(:,1), q_1);

    % Find q5 using Subproblem 4
    % h_2 ' R_01' R_06 h_6 = h_2' R_45 h_6
    [theta5, theta5_is_ls] = subproblem.sp_4(H(:,2),H(:,6),H(:,5), ...
        H(:,2)' * R_01' * R_06 * H(:,6));
    
    for i_t5 = 1:length(theta5)
        q_5 = theta5(i_t5);
        R_45 = rot(H(:,5), q_5);
    
        % solve for R_14 using Subproblem 1
        [theta_14, theta_14_is_LS] = subproblem.sp_1(R_45*H(:,6),R_01'*R_06*H(:,6), H(:,2));

        % solve for q_6 using Subproblem 1
        [q_6, q_6_is_LS] = subproblem.sp_1(R_45'*H(:,2), R_06'*R_01*H(:,2), -H(:,6));
    
        % solve for q3
        d_inner = R_01'*p_06-P(:,2) - rot(H(:,2), theta_14)*P(:,5); % Recall p_56 = 0
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
            %is_LS_vec = [is_LS_vec theta1_is_ls||theta5_is_ls||theta_14_is_LS||theta_3_is_LS||q_2_is_LS||q_6_is_LS];
            is_LS_vec = [is_LS_vec [theta1_is_ls theta5_is_ls theta_14_is_LS theta_3_is_LS q_2_is_LS q_6_is_LS]'];
        end
    
    end
end
end
