function [Q1, Q2, Q3, Q4, Q5, Q6] = gen_6_dof_soln_given_q12(q1, q2, R_0T, p_0T, kin)
ALIGNMENT_THRESH = 1 - 1e-3;


Q1 = []; Q2 = []; Q3 = [];
Q4 = []; Q5 = []; Q6 = [];


H = kin.H;
P = kin.P;
p_01 = P(:,1);
p_12 = P(:,2);
p_23 = P(:,3);
p_34 = P(:,4);
p_45 = P(:,5);
p_56 = P(:,6);
p_6T = P(:,7);

% Appply forward kin to get elbow position
kin_12 = kin;
kin_12.joint_type = zeros([1 2]);
[R_03, p_03] = fwdkin(kin_12, [q1 q2]);

p_3T = p_0T - p_03;
P_T3 = -R_0T'*p_3T;

% Use gen sp3 to find q4 q5 q6
[t4, t5, t6] = gen_subproblem3_linear(-p_45,-p_34,p_56,P_T3+p_6T,-H(:,4),H(:,5),H(:,6));
%[theta1, theta2, theta3] = gen_subproblem3_linear(p0,p1,p2,p3,k1,k2,k3)
% p0 + R(k1,theta1)p1 = R(k2,theta2)(p2+R(k3,theta3)p3)


% Check for alignment of h_3

h_3_base = R_03*H(:,3);

for i = 1:length(t4)
    h_3_T = R_0T* rot(H(:,6),-t6(i))*rot(H(:,5),-t5(i))*rot(H(:,4),-t4(i))*H(:,3);
    h_dot_h =  dot(h_3_base,h_3_T);

    if h_dot_h > ALIGNMENT_THRESH
        R_1 = rot(H(:,1),q1);
        R_2 = rot(H(:,2),q2);
        R_4 = rot(H(:,4),t4(i));
        R_5 = rot(H(:,5),t5(i));
        R_6 = rot(H(:,6),t6(i));
        % Find q_3 using sp1
        %t3 = subproblem1_linear(p1, p_0T-p_03, H(:,3))
        t3 = subproblem1_linear( ...
            p_34 + R_4*(p_45 + R_5*(p_56+R_6*p_6T)), ...
            -p_23+R_2'*(-p_12+R_1'*(p_0T-p_01)), ...
            H(:,3));
        %  p2 = rot(k, theta)*p1

        Q1 = [Q1 q1];
        Q2 = [Q2 q2];
        Q3 = [Q3 t3];
        Q4 = [Q4 t4(i)];
        Q5 = [Q5 t5(i)];
        Q6 = [Q6 t6(i)];
    end
end


end