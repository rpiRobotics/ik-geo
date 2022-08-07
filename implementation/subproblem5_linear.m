function [theta1, theta2, theta3] = subproblem5_linear_new(p0,p1,p2,p3,k1,k2,k3)
% p0 + R(k1,theta1)p1 = R(k2,theta2)(p2+R(k3,theta3)p3)

theta1 = [];
theta2 = [];
theta3 = [];

p1_s = p0+k1*k1'*p1;
p3_s = p2+k3*k3'*p3;

beta1 = dot(k2,p1_s);
beta3 = dot(k2,p3_s);

[P_1, R_1] = cone_polynomials(p0, k1, p1, p1_s, k2);
[P_3, R_3] = cone_polynomials(p2, k3, p3, p3_s, k2);
% global H_ap
% global N_ap
% global x1_ap
% sign = [1 -1];
% t1 = (norm(k2)^2-2)*H_ap^2 + polyval(P_1, H_ap) + sign*sqrt(polyval(R_1, H_ap));
% t3 = (norm(k2)^2-2)*H_ap^2 + polyval(P_3, H_ap) + sign*sqrt(polyval(R_3, H_ap));

% Now solve the quadratic for H
P_13 = P_1 - P_3;
P_13_sq = conv2(P_13, P_13);

RHS = R_3 - R_1 - P_13_sq;

EQN = conv2(RHS,RHS)-4*conv2(P_13_sq,R_1);

all_roots = roots(EQN)';
H_vec = all_roots(real(all_roots) == all_roots);

% Find v_1(H_star) and v_3(H_star) for each branch
% and use subproblem 1

KxP1 = cross(k1,p1);
KxP3 = cross(k3,p3);
A_1 = [KxP1 -cross(k1,KxP1)];
A_3 = [KxP3 -cross(k3,KxP3)];

signs = [+1 +1 -1 -1
         +1 -1 +1 -1];
J = [0 1; -1 0];

for H = H_vec
for sign = signs
    sign_1 = sign(1);
    sign_3 = sign(2);


    sc1 = A_1'*k2*(H-beta1)+sign_1*J*A_1'*k2*sqrt(norm(A_1'*k2)^2 - (H-beta1)^2);
    sc1 = sc1/norm(A_1'*k2)^2;

    sc3 = A_3'*k2*(H-beta3)+sign_3*J*A_3'*k2*sqrt(norm(A_3'*k2)^2 - (H-beta3)^2);
    sc3 = sc3/norm(A_3'*k2)^2;

    v1 = A_1*sc1+p1_s;
    v3 = A_3*sc3+p3_s;

    if abs(norm(v1-H*k2) - norm(v3-H*k2)) < 1E-3
        theta1 = [theta1 atan2(sc1(1), sc1(2))];
        theta2 = [theta2 subproblem1_linear(v3, v1, k2)];
        theta3 = [theta3 atan2(sc3(1), sc3(2))];
    end

end
end


end

function [P, R] = cone_polynomials(p0_i, k_i, p_i, p_i_s, k2)
% ||A x + p_S - H k_2||^2 = 
% c H^2 + P(H) +- sqrt(R(H))
% Reperesent polynomials P_i, R_i as coefficient vectors
% (Highest powers of H first)
% Don't bother calculating c H^2, as these terms cancel out

beta = dot(k2,p_i_s);
c1 = 2 * k2' * (p0_i - k_i*k_i'*p0_i)/ norm(cross(k_i,k2))^2;
c2 = 2 * p0_i' * cross(k_i, k2)/ norm(cross(k_i,k2))^2;

P_const = -c1*beta + norm(cross(k_i,p_i))^2 + norm(p_i_s)^2;
P = [c1 P_const];

R = [1 -beta]; % (H-beta_i)
R = -conv2(R, R); % -(H-beta_i)^2
R(end) = R(end) + norm(cross(k_i,p_i))^2*norm(cross(k_i,k2))^2; % ||A_i' k_2||^2 - (H-beta_i)^2
R = c2^2 * R;

end