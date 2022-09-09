function [Q, is_LS_vec] = IK_3_parallel(R_06, p_0T, kin)

P = kin.P;
H = kin.H;

% Double check the 2 equations we are solving
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% global q
% R_01 = rot(H(:,1), q(1));
% R_12 = rot(H(:,2), q(2));
% R_23 = rot(H(:,3), q(3));
% R_34 = rot(H(:,4), q(4));
% R_45 = rot(H(:,5), q(5));
% R_56 = rot(H(:,6), q(6));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p_16 = p_0T - P(:,1) - R_06*kin.P(:,7);

% position eqn
% H(:,2)'* (R_01'*p_16-P(:,2)-R_45*P(:,6))
% H(:,2)'* (P(:,3)+P(:,4)+P(:,5))
% 
% % rotation equation
% H(:,2)'*R_01'*R_06*H(:,6)
% H(:,2)'*R_45*H(:,6)


H_sp = [H(:,2) H(:,2) H(:,2) H(:,2)];
K_sp = [-H(:,1) H(:,5) -H(:,1) H(:,5)];
P_sp = [p_16 -P(:,6) R_06*H(:,6) -H(:,6)];
d1 = H(:,2)'* (P(:,3)+P(:,4)+P(:,5) + P(:,2));
d2 = 0;

% Test that subproblem is set up correctly
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% global x_ap
% x_ap = [sin(q(1)) cos(q(1)) sin(q(5)) cos(q(5))]'
% 
% sp.h'* rot(sp.k1, q(1)) * sp.p1  + sp.h'* rot(sp.k2, q(5)) * sp.p2  - sp.d1
% sp.h'* rot(sp.k1, q(1)) * sp.p3  + sp.h'* rot(sp.k2, q(5)) * sp.p4  - sp.d2
% 
% 
% k1Xp1 = cross(sp.k1,sp.p1); k2Xp2 = cross(sp.k2,sp.p2);
% k1Xp3 = cross(sp.k1,sp.p3); k2Xp4 = cross(sp.k2,sp.p4);
% A_1 = [k1Xp1, -cross(sp.k1, k1Xp1)];
% A_2 = [k2Xp2, -cross(sp.k2, k2Xp2)];
% A_3 = [k1Xp3, -cross(sp.k1, k1Xp3)];
% A_4 = [k2Xp4, -cross(sp.k2, k2Xp4)];
% 
% sp.h'*A_1*x_ap(1:2) + sp.h'*A_2*x_ap(3:4) - (sp.d1 -sp.h'*sp.k1*sp.k1'*sp.p1 - sp.h'*sp.k2*sp.k2'*sp.p2)
% sp.h'*A_3*x_ap(1:2) + sp.h'*A_4*x_ap(3:4) - (sp.d2 -sp.h'*sp.k1*sp.k1'*sp.p3 - sp.h'*sp.k2*sp.k2'*sp.p4)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Solve subproblem
[theta1, theta5] = subproblem.sp_6(H_sp, K_sp, P_sp, d1, d2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% for i = 1:length(theta1)
%     x_i = [sin(theta1(i)) cos(theta1(i)) sin(theta2(i)) cos(theta2(i))]'
%     sp.h'* rot(sp.k1, theta1(i)) * sp.p1  + sp.h'* rot(sp.k2, theta2(i)) * sp.p2  - sp.d1
%     sp.h'* rot(sp.k1, theta1(i)) * sp.p3  + sp.h'* rot(sp.k2, theta2(i)) * sp.p4  - sp.d2
% 
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Q = [];
is_LS_vec = [];
for i = 1:length(theta1)
    q_1 = theta1(i);
    q_5 = theta5(i);

    % solve for R_14
    [theta_14, theta_14_is_LS] = subproblem.sp_1(rot(H(:,5), q_5)*H(:,6), rot(H(:,1),-q_1)*R_06*H(:,6), H(:,2));

    % solve for q3
    R_01 = rot(H(:,1), q_1);
    R_45 = rot(H(:,5), q_5);
    R_14 = rot(H(:,2), theta_14);
    p_12 = P(:,2);
    p_23 = P(:,3);
    p_34 = P(:,4);
    p_45 = P(:,5);
    p_56 = P(:,6);
    d_inner = R_01'*p_16-p_12 - R_14*R_45*p_56-R_14*p_45;
    d = norm(d_inner);
    [theta_3, theta_3_is_LS] = subproblem.sp_3(-p_34, p_23, H(:,2), d);

    %for q_3 = theta_3
    for i_q3 = length(theta_3)
        q_3 = theta_3(i_q3);
        % solve for q2
        [q_2, q_2_is_LS] = subproblem.sp_1(p_23 + rot(H(:,2), q_3)*p_34, d_inner, H(:,2));

        % q4 by subtraction
        q_4 = wrapToPi( theta_14 - q_2 - q_3);

        % And finally q6 using rotation component
        % theta = subproblem1_linear(p1, p2, k)
        %  p2 = rot(k, theta)*p1
        [q_6, q_6_is_LS] = subproblem.sp_1(H(:,5), R_45'*R_14'*R_01'*R_06*H(:,5), H(:,6));
        q_i = [q_1; q_2; q_3; q_4; q_5; q_6];
        Q = [Q q_i];
        %is_LS_vec = [is_LS_vec theta_14_is_LS||theta_3_is_LS||q_2_is_LS||q_6_is_LS];
        is_LS_vec = [is_LS_vec [theta_14_is_LS theta_3_is_LS q_2_is_LS q_6_is_LS]'];
    end

end

end
