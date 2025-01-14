function [Q, is_LS_vec, is_LS_mat] = IK_spherical_2_parallel_sing(R_06, p_0T, kin, e_sing)

Q = [];
is_LS_vec = [];
is_LS_mat = [];

% Use subproblem 4 to find up to two solutions for q1
p_0T_prime = p_0T-R_06*kin.P(:,7)-kin.P(:,1);

if norm(cross(kin.H(:,1),p_0T_prime)) > 0
    % Use p_0T to find q_1
    [t1, q1_is_LS] = subproblem.sp_4(kin.H(:,2), p_0T_prime, -kin.H(:,1), kin.H(:,2)'*(kin.P(:,2)+kin.P(:,3)+kin.P(:,4)));
else % Internal singularity: Use e_sing to find q_1
    [t1, q1_is_LS] = subproblem.sp_4(e_sing, cross(kin.H(:,2), kin.H(:,1)), kin.H(:,1), 0);
end

t1 = [t1(1) t1(end)]; % Duplicate solution if LS

% Use subproblem 3 to find up to two solutions for q3
for i_q1 = 1:length(t1)
    q1 = t1(i_q1);
    [t3, q3_is_LS] = subproblem.sp_3( -kin.P(:,4), kin.P(:,3), kin.H(:,3), norm(rot(-kin.H(:,1), q1)*p_0T_prime - kin.P(:,2)));
    t3 = [t3(1) t3(end)]; 

    % Solve for q3 using subproblem 1
    for i_q3 = 1:length(t3)
        q3 = t3(i_q3);
        [q2, q2_is_LS] = subproblem.sp_1(-kin.P(:,3) - rot(kin.H(:,3), q3)*kin.P(:,4), rot(-kin.H(:,1), q1)*(-p_0T_prime) + kin.P(:,2), kin.H(:,2));

        R_36 = rot(-kin.H(:,3), q3)*rot(-kin.H(:,2), q2)*rot(-kin.H(:,1), q1)*R_06;
        % Solve for q5 using subproblem 4
        [t5, q5_is_LS] = subproblem.sp_4(kin.H(:,4), kin.H(:,6), kin.H(:,5), kin.H(:,4)'*R_36*kin.H(:,6));
        t5 = [t5(1) t5(end)]; 
        
        % Solve for q4 using subproblem 1
        for i_q5 = 1:length(t5)
            q5 = t5(i_q5);
            [q4, q4_is_LS] = subproblem.sp_1( rot(kin.H(:,5), q5)*kin.H(:,6),R_36*kin.H(:,6), kin.H(:,4));
            % Solve for q6 using subproblem 1
            [q6, q6_is_LS] = subproblem.sp_1( rot(-kin.H(:,5), q5) * kin.H(:,4), R_36'*kin.H(:,4), -kin.H(:,6));
            q_i = [q1 q2 q3 q4 q5 q6]';
            Q = [Q q_i];
            is_LS_vec = [is_LS_vec q1_is_LS||q2_is_LS||q3_is_LS||q4_is_LS||q5_is_LS||q6_is_LS];
            is_LS_mat = [is_LS_mat [q1_is_LS q2_is_LS q3_is_LS q4_is_LS q5_is_LS q6_is_LS]'];
            % TODO Do we need to check all of these?
        end
    end
end
end