function [theta1, theta2, theta3] = sp_5(p0, p1, p2, p3, k1, k2, k3)
% subproblem.sp_5  Subproblem 5: Three cones
%   [theta1, theta2, theta3] = sp_5(p0, p1, p2, p3, k1, k2, k3) finds
%   theta1, theta2, theta3 such that
%       p0 + rot(k1,theta1)*p1 = rot(k2,theta2)*(p2+rot(k3,theta3)*p3)
%
%   If the problem is well-posed, there may be up to 4 solutions
%   theta1, theta2, and theta3 are column vectors of the solutions
%
%   This subproblem solution assumes rotation axes 1 and 2 aren't coplanar
%   If they are, rewrite the problem and solve using subproblem.sp_4() and
%   subproblem.sp_3()
%
%   The problem is ill-posed if (k1, p1), or (k2, p2) are parallel
%
%   Inputs:
%       p0: 3x1 vector
%       p1: 3x1 vector
%       p2: 3x1 vector
%       p3: 3x1 vector
%       k1: 3x1 vector with norm(k1) = 1
%       k2: 3x1 vector with norm(k2) = 1
%       k3: 3x1 vector with norm(k3) = 1
%   Outputs:
%       theta1: 1xN angle (in radians)
%           (N is the number of solutions)
%       theta2: 1xN angle (in radians)
%       theta3: 1xN angle (in radians)

theta1 = NaN([1 8]);
theta2 = NaN([1 8]);
theta3 = NaN([1 8]);
i_soln = 0;

p1_s = p0+k1*k1'*p1;
p3_s = p2+k3*k3'*p3;

delta1 = dot(k2,p1_s);
delta3 = dot(k2,p3_s);

[P_1, R_1] = cone_polynomials(p0, k1, p1, p1_s, k2);
[P_3, R_3] = cone_polynomials(p2, k3, p3, p3_s, k2);

% -H^2 + P(H) +- sqrt(R(H))
% E1_p = poly2sym(P_1) + sqrt(poly2sym(R_1));
% E1_n = poly2sym(P_1) - sqrt(poly2sym(R_1));
% E3_p = poly2sym(P_3) + sqrt(poly2sym(R_3));
% E3_n = poly2sym(P_3) - sqrt(poly2sym(R_3));
% fplot(E1_p); hold on
% fplot(E1_n);
% fplot(E3_p);
% fplot(E3_n); hold off

% Now solve the quadratic for H
P_13 = P_1 - P_3;
P_13_sq = conv2(P_13, P_13);

RHS = R_3 - R_1 - P_13_sq;

EQN = conv2(RHS,RHS)-4*conv2(P_13_sq,R_1);


all_roots = subproblem.quartic_roots(EQN)'; %all_roots = roots(EQN)';

H_vec = all_roots( abs(imag(all_roots)) < 1e-6 ); %H_vec = all_roots(real(all_roots) == all_roots);
H_vec = real(H_vec);

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

const_1 = A_1'*k2*(H-delta1);
const_3 = A_3'*k2*(H-delta3);
if (norm(A_1'*k2)^2 - (H-delta1)^2) < 0
    continue
end
if(norm(A_3'*k2)^2 - (H-delta3)^2) < 0
    continue
end

pm_1 = J*A_1'*k2*sqrt(norm(A_1'*k2)^2 - (H-delta1)^2);
pm_3 = J*A_3'*k2*sqrt(norm(A_3'*k2)^2 - (H-delta3)^2);

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
        theta2(i_soln) = subproblem.sp_1(v3, v1, k2);
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
% -H^2 + P(H) +- sqrt(R(H))
% Reperesent polynomials P_i, R_i as coefficient vectors
% (Highest powers of H first)

kiXk2 = cross(k_i,k2);
kiXkiXk2 = cross(k_i,kiXk2);
norm_kiXk2_sq = dot(kiXk2,kiXk2);

kiXpi = cross(k_i,p_i);
norm_kiXpi_sq = dot(kiXpi, kiXpi);

delta = dot(k2,p_i_s);
alpha = p0_i' * kiXkiXk2/ norm_kiXk2_sq;
beta =  p0_i' *  kiXk2  / norm_kiXk2_sq;

P_const = norm_kiXpi_sq + dot(p_i_s, p_i_s) + 2*alpha*delta;
P = [-2*alpha P_const];

R = [-1 2*delta -delta^2]; % -(H-delta_i)^2
R(end) = R(end) + norm_kiXpi_sq*norm_kiXk2_sq; % ||A_i' k_2||^2 - (H-delta_i)^2
R = (2*beta)^2 * R;

end