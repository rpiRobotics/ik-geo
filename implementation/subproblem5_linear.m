function [theta1, theta2, theta3] = subproblem5_linear(p0,p1,p2,p3,k1,k2,k3)
% p0 + R(k1,theta1)p1 = R(k2,theta2)(p2+R(k3,theta3)p3)

theta1 = NaN([1 8]);
theta2 = NaN([1 8]);
theta3 = NaN([1 8]);
i_soln = 0;

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
H_vec = real(H_vec); % for coder;

% Find v_1(H_star) and v_3(H_star) for each branch
% and use subproblem 1

KxP1 = cross(k1,p1);
KxP3 = cross(k3,p3);
A_1 = [KxP1 -cross(k1,KxP1)];
A_3 = [KxP3 -cross(k3,KxP3)];

signs = [+1 +1 -1 -1
         +1 -1 +1 -1];
J = [0 1; -1 0];

%for H = H_vec
for i_H = 1:length(H_vec)
H = H_vec(i_H);

const_1 = A_1'*k2*(H-beta1);
const_3 = A_3'*k2*(H-beta3);
pm_1 = J*A_1'*k2*sqrt(norm(A_1'*k2)^2 - (H-beta1)^2);
pm_3 = J*A_3'*k2*sqrt(norm(A_3'*k2)^2 - (H-beta3)^2);

for i_sign = 1:4
    sign_1 = signs(1,i_sign);
    sign_3 = signs(2,i_sign);


    sc1 =const_1+sign_1*pm_1;
    sc1 = sc1/norm(A_1'*k2)^2;

    sc3 = const_3+sign_3*pm_3;
    sc3 = sc3/norm(A_3'*k2)^2;

    v1 = A_1*sc1+p1_s;
    v3 = A_3*sc3+p3_s;

    if abs(norm(v1-H*k2) - norm(v3-H*k2)) < 1E-6
        i_soln = 1 + i_soln;
        theta1(i_soln) = atan2(sc1(1), sc1(2));
        theta2(i_soln) = subproblem1_linear(v3, v1, k2);
        theta3(i_soln) = atan2(sc3(1), sc3(2));     
    end

end
end

theta1 = theta1(1:i_soln);
theta2 = theta2(1:i_soln);
theta3 = theta3(1:i_soln);


end

function [P, R] = cone_polynomials(p0_i, k_i, p_i, p_i_s, k2)
% ||A x + p_S - H k_2||^2 = 
% c H^2 + P(H) +- sqrt(R(H))
% Reperesent polynomials P_i, R_i as coefficient vectors
% (Highest powers of H first)
% Don't bother calculating c H^2, as these terms cancel out

kiXk2 = cross(k_i,k2);
norm_kiXk2_sq = dot(kiXk2,kiXk2);

beta = dot(k2,p_i_s);
c1 = 2 * k2' * (p0_i - k_i*k_i'*p0_i)/ norm_kiXk2_sq;
c2 = 2 * p0_i' * kiXk2/ norm_kiXk2_sq;

P_const = -c1*beta + norm(cross(k_i,p_i))^2 + norm(p_i_s)^2;
P = [c1 P_const];

%R = [1 -beta]; % (H-beta_i)
%R = -conv2(R, R); % -(H-beta_i)^2
R = [-1 2*beta -beta^2]; % -(H-beta_i)^2
R(end) = R(end) + norm(cross(k_i,p_i))^2*norm_kiXk2_sq; % ||A_i' k_2||^2 - (H-beta_i)^2
R = c2^2 * R;

end