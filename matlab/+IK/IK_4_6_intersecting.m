function [Q, is_LS_vec] = IK_4_6_intersecting(R_06, p_0T, kin)
% Axes 4 and 6 always intersect

Q = [];
is_LS_vec = [];

p_06 = p_0T - kin.P(:,1) - R_06*kin.P(:,7);

[q5_vec, soln_num_vec] = search_1D( ...
    @(q5)(alignment_err_given_q5(q5, p_06, R_06, kin)), ...
    -pi, pi, 5000, true)

for i_q5 = 1:length(q5_vec)
    [e, Q_partial] = alignment_err_given_q5(q5_vec(i_q5), p_06, R_06, kin);
    q_partial = Q_partial(:,soln_num_vec(:,i_q5));

    R_03 = rot(kin.H(:,1),q_partial(1)) * rot(kin.H(:,2),q_partial(2)) ...
         * rot(kin.H(:,3),q_partial(3));
    R_45 = rot(kin.H(:,5),q_partial(4));
    [q4, q4_is_LS] = subproblem.sp_1(R_45*kin.H(:,6), R_03'*R_06*kin.H(:,6),  kin.H(:,4));
    [q6, q6_is_LS] = subproblem.sp_1(R_45'*kin.H(:,4), R_06'*R_03*kin.H(:,4), -kin.H(:,6));
    q_i = [q_partial(1:3); q4; q_partial(4); q6];
    Q = [Q q_i];
    is_LS_vec = [is_LS_vec [q4_is_LS; q6_is_LS; e(soln_num_vec(:,i_q5))] ];
end

end

function [e_vec, Q_partial] = alignment_err_given_q5(q5, p_06, R_06, kin)
    e_vec = NaN([1 4]);
    Q_partial = NaN([4 4]);

    % Find p_04_prime and p_34_prime
    R_45 = rot(kin.H(:,5), q5);

    [p_34_prime, x] = find_line_intersection( ...
        kin.P(:,4), ...
        kin.P(:,4) + kin.P(:,5) + R_45*kin.P(:,6), ...
        kin.H(:,4), ...
        R_45*kin.H(:,6));
    p_04_prime = p_06 + R_06*kin.H(:,6)*x(2);

    % Use Subproblem 5 to find (q_1, q_2, q_3)
    [t1, t2, t3] = subproblem.sp_5( ...
       -kin.P(:,2), p_04_prime, kin.P(:,3), p_34_prime, ...
       -kin.H(:,1), kin.H(:,2), kin.H(:,3));

    % Calculate error
    for i_q123 = 1:length(t1)
        q1 = t1(i_q123);
        q2 = t2(i_q123);
        q3 = t3(i_q123);
    
        R_03 = rot(kin.H(:,1),q1) * rot(kin.H(:,2),q2) * rot(kin.H(:,3),q3);
    
        e_i = kin.H(:,4)' * R_03' * R_06* kin.H(:,6) - kin.H(:,4)'*R_45*kin.H(:,6);
        e_vec(i_q123) = e_i;
        Q_partial(:,i_q123) = [q1; q2; q3; q5];
    end
end

function [p_int, x] = find_line_intersection(p1, p2, h1, h2)
    % p_A = p1 + lambda_1 h1
    % p_B = p2 + lambda_2 h2
    
    A = [h1 -h2];
    b = p2 - p1;
    x = lsqminnorm(A,b);
    p_A = p1 + x(1) * h1;
    p_B = p2 + x(2) * h2;
    
    % These should be equal, but return avg
    p_int = p_A/2 + p_B/2;
    if abs(dot(h1,h2))<0.95
        assert(norm(p_A - p_B)<0.5)
    end

end