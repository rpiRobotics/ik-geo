function [Q, is_LS_vec] = IK_gen_6_dof(R_06, p_0T, kin)

p_16 = p_0T - kin.P(:,1) - R_06*kin.P(:,7);

q1_vec = linspace(-pi, pi, 50);
q2_vec = linspace(-pi, pi, 50);

[q1_mesh, q2_mesh] = meshgrid(q1_vec, q2_vec);

e_mesh = NaN(size(q1_mesh));

for i = 1:numel(e_mesh)
    q1 = q1_mesh(i);
    q2 = q2_mesh(i);
    e_mesh(i) = alignment_err_given_q12(q1, q2, p_16, R_06, kin);
end


[~, q12_guess_idx] = min(e_mesh(:));
q1_star = q1_mesh(q12_guess_idx);
q2_star = q2_mesh(q12_guess_idx);

% mesh(q1_mesh, q2_mesh, e_mesh); hold on
% plot3([q1_star q1_star], [q2_star q2_star], [0 max(e_mesh(:))], 'r', LineWidth=2); hold off

% optimize further
options = optimset('TolFun',1e-5);
q12_star = fminsearch(@(x)alignment_err_given_q12(x(1), x(2), p_16, R_06, kin),[q1_star;q2_star], options);
q1_star = q12_star(1);
q2_star = q12_star(2);

[~, Q, is_LS_vec] = alignment_err_given_q12(q1_star, q2_star, p_16, R_06, kin);

function [e, Q, is_LS_vec] = alignment_err_given_q12(q1, q2, p_16, R_06, kin)
    e_vec = NaN;
    Q = [];
    is_LS_vec = [];
    % find up to 4 solutions of (q3, q4, q5) using Subproblem 5
    p_63 = rot(-kin.H(:,2),q2) * (rot(-kin.H(:,1),q1)*p_16 - kin.P(:,2)) - kin.P(:,3);
    [t3, t4, t5] = subproblem.sp_5( ...
                   -kin.P(:,4), p_63, kin.P(:,5), kin.P(:,6), ...
                   -kin.H(:,3), kin.H(:,4), kin.H(:,5));
    for i_q345 = 1:length(t3)
        q3 = t3(i_q345);
        q4 = t4(i_q345);
        q5 = t5(i_q345);

        % Calculate alignment error for h_6
        R_05 = rot(kin.H(:,1),q1)*rot(kin.H(:,2),q2) ...
            * rot(kin.H(:,3),q3)*rot(kin.H(:,4),q4) ...
            * rot(kin.H(:,5),q5);

        e_i = norm(R_05*kin.H(:,6)-R_06*kin.H(:,6));
        e_vec = [e_vec e_i];

        if nargout > 1
            % Find q6 with subproblem 1
            p = [0;0;1]; % Can't be colinear with h_6
            [q6, q6_is_ls] = subproblem.sp_1(p, R_05'*R_06*p, kin.H(:,6));

            q_i = [q1; q2; q3; q4; q5; q6];
            Q = [Q q_i];
            is_LS_vec = [is_LS_vec [q6_is_ls; e_i] ];
        end
    end
                    
    e = min(e_vec);

end

end