function [Q, is_LS_vec] = IK_spherical(R_06, p_0T, kin)

Q = [];
is_LS_vec = [];

p_06 = p_0T - kin.P(:,1) - R_06*kin.P(:,7);


% Use subproblem 5 to find (q1, q2, q3)

% -p_12 + R_01^T p_16 = R_12 (p_23 + R_23(p_34))

[t1, t2, t3] = subproblem.sp_5( ...
    -kin.P(:,2), p_06, kin.P(:,3), kin.P(:,4), ...
    -kin.H(:,1), kin.H(:,2), kin.H(:,3));


for i_q123 = 1:length(t1)
    q1 = t1(i_q123);
    q2 = t2(i_q123);
    q3 = t3(i_q123);

    R_36 = rot(-kin.H(:,3), q3)*...
           rot(-kin.H(:,2), q2)*...
           rot(-kin.H(:,1), q1)*R_06;
    % Solve for q5 using subproblem 4
    [t5, q5_is_LS] = subproblem.sp_4( ...
        kin.H(:,4), ...
        kin.H(:,6), ...
        kin.H(:,5), ...
        kin.H(:,4)'*R_36*kin.H(:,6));
    
    % Solve for q4 using subproblem 1
    for i_q5 = 1:length(t5)
        q5 = t5(i_q5);
        [q4, q4_is_LS] = subproblem.sp_1( ...
            rot(kin.H(:,5), q5)*kin.H(:,6), ...
            R_36*kin.H(:,6), ...
            kin.H(:,4));
        % Solve for q6 using subproblem 1
        [q6, q6_is_LS] = subproblem.sp_1( ...
            rot(-kin.H(:,5), q5) * kin.H(:,4), ...
            R_36'*kin.H(:,4), ...
            -kin.H(:,6));
        q_i = [q1 q2 q3 q4 q5 q6]';
        Q = [Q q_i];
        is_LS_vec = [is_LS_vec q5_is_LS||q4_is_LS||q6_is_LS];
        % TODO Do we need to check all of these?
    end
end

end