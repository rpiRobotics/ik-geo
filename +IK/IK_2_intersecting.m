function [Q, is_LS_vec] = IK_2_intersecting(R_06, p_0T, kin)
% Axes 5 and 6 intersect

% Q = [];
% is_LS_vec = [];
p_16 = p_0T - kin.P(:,1) - R_06*kin.P(:,7);

q4_vec = linspace(-pi, pi, 100);
e_q4 = NaN(size(q4_vec));

% for i = 1:length(q4_vec)
%     e_q4_i = alignment_err_given_q4(q4_vec(i), p_16, R_06, kin);
%     if ~isnan(e_q4_i)
%         e_q4(i) = e_q4_i;
%     end
% end

q4_star = fminbnd(@(x)alignment_err_given_q4(x, p_16, R_06, kin),-pi,pi);
%[~, q_guess_idx] = min(e_q4);

%q4_star = fminbnd(@(x)alignment_err_given_q4(x, p_16, R_06, kin),q4_vec(q_guess_idx)-1e-4,q4_vec(q_guess_idx)+1e-4);
%options = optimset('TolFun',1e-16);
%options = optimset('TolFun',1e-5);
%q4_star = fminsearch(@(x)alignment_err_given_q4(x, p_16, R_06, kin), q4_vec(q_guess_idx), options);

% plot(q4_vec, e_q4)
% xline(q4_star);

[~, Q, is_LS_vec] = alignment_err_given_q4(q4_star, p_16, R_06, kin);

end

function [e, Q, is_LS_vec] = alignment_err_given_q4(q4, p_16, R_06, kin)
    e_vec = NaN;
    Q = [];
    is_LS_vec = [];
    % find up to 4 solutions of (q1, q2, q3) using Subproblem 5
    p_35_3 = kin.P(:,4) + rot(kin.H(:,4), q4)*kin.P(:,5);
    
    [t1, t2, t3] = subproblem.sp_5( ...
                   -kin.P(:,2), p_16, kin.P(:,3), p_35_3, ...
                   -kin.H(:,1), kin.H(:,2), kin.H(:,3));
    
    % find up to two solutions of (q5, q6) by using Subproblem 2
    for i_q123 = 1:length(t1)
        q1 = t1(i_q123);
        q2 = t2(i_q123);
        q3 = t3(i_q123);

        R_04 = rot(kin.H(:,1),q1) * rot(kin.H(:,2),q2) ...
             * rot(kin.H(:,3),q3) * rot(kin.H(:,4),q4);

        [t5, t6, t56_is_LS] = subproblem.sp_2( ...
                              R_04'*R_06*kin.H(:,5), kin.H(:,5), ...
                              -kin.H(:,5), kin.H(:,6));
        for i_q56 = 1:length(t5)
            q5 = t5(i_q56);
            q6 = t6(i_q56);

            % Calculate alignment error for h_6
            e_i = norm(R_04*rot(kin.H(:,5),q5)*kin.H(:,6)-R_06*kin.H(:,6));
            e_vec = [e_vec e_i];
            q_i = [q1; q2; q3; q4; q5; q6];
            Q = [Q q_i];
            is_LS_vec = [is_LS_vec [t56_is_LS; e_i] ];
        end
    end
    e = min(e_vec);
end

