function h_dot_h = gen_6_dof_alignment_vs_q12(q1, q2, R_0T, p_0T, kin)
H = kin.H;

P = kin.P;

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

h_dot_h = NaN([1 length(t4)]);
for i = 1:length(t4)
    h_3_T = R_0T* rot(H(:,6),-t6(i))*rot(H(:,5),-t5(i))*rot(H(:,4),-t4(i))*H(:,3);
    h_dot_h(i) = dot(h_3_base,h_3_T);
end



end