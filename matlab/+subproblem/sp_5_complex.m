function [theta1, theta2, theta3, z_vec] = sp_5_complex(p0, p1, p2, p3, k1, k2, k3)
% Approximate continuous version of Subproblem 5
% Returns uses real part of complex zeros to always return 4 solutions

theta1 = NaN([1 4]);
theta2 = NaN([1 4]);
theta3 = NaN([1 4]);
i_soln = 0;

p1_s = p0+k1*k1'*p1;
p3_s = p2+k3*k3'*p3;

delta1 = dot(k2,p1_s);
delta3 = dot(k2,p3_s);

[P_1, R_1] = circle_polynomials(p0, k1, p1, p1_s, k2);
[P_3, R_3] = circle_polynomials(p2, k3, p3, p3_s, k2);

% P(H) +- sqrt(R(H))
% E1_p = poly2sym(P_1) + sqrt(poly2sym(R_1));
% E1_n = poly2sym(P_1) - sqrt(poly2sym(R_1));
% E3_p = poly2sym(P_3) + sqrt(poly2sym(R_3));
% E3_n = poly2sym(P_3) - sqrt(poly2sym(R_3));
% fplot(E1_p); hold on
% fplot(E1_n);
% fplot(E3_p);
% fplot(E3_n); hold off

% Now solve the quartic for z
P_13 = P_1 - P_3;
P_13_sq = conv2(P_13, P_13);
RHS = R_3 - R_1 - P_13_sq;
EQN = conv2(RHS,RHS)-4*conv2(P_13_sq,R_1);

all_roots = subproblem.quartic_roots(EQN).'; %all_roots = roots(EQN)';

% z_vec = all_roots( abs(imag(all_roots)) < 1e-6 ); %H_vec = all_roots(real(all_roots) == all_roots);
% z_vec = real(z_vec);
% z_vec = subproblem.uniquetol_manual(z_vec);

z_vec = all_roots;

% Find A_1 x_1(z) + p1_s and A_3 x_3(z) + p3_s for each branch
% and use subproblem 1 to find theta_2

KxP1 = cross(k1,p1);
KxP3 = cross(k3,p3);
A_1 = [KxP1 -cross(k1,KxP1)];
A_3 = [KxP3 -cross(k3,KxP3)];

signs = [+1 +1 -1 -1
         +1 -1 +1 -1];
J = [0 1; -1 0];

for i_z = 1:length(z_vec)
    if i_soln == 4; break; end
    z = z_vec(i_z);
    
    const_1 = A_1'*k2*(z-delta1);
    const_3 = A_3'*k2*(z-delta3);
    % if (norm(A_1'*k2)^2 - (z-delta1)^2) < 0
    %     continue
    % end
    % if(norm(A_3'*k2)^2 - (z-delta3)^2) < 0
    %     continue
    % end
    
    pm_1 = J*A_1'*k2*sqrt(norm(A_1'*k2)^2 - (z-delta1)^2);
    pm_3 = J*A_3'*k2*sqrt(norm(A_3'*k2)^2 - (z-delta3)^2);
    
    for i_sign = 1:4
        if i_soln == 4; break; end
        sign_1 = signs(1,i_sign);
        sign_3 = signs(2,i_sign);
    
        sc1 = const_1+sign_1*pm_1;
        sc1 = sc1/norm(A_1'*k2)^2;
    
        sc3 = const_3+sign_3*pm_3;
        sc3 = sc3/norm(A_3'*k2)^2;
    
        Rp_1 = A_1*sc1+p1_s;
        Rp_3 = A_3*sc3+p3_s;
    
        if abs(Rp_1.'*Rp_1 - Rp_3.'*Rp_3) < 1E-6
            i_soln = 1 + i_soln;
            theta1(i_soln) = atan2(real(sc1(1)), real(sc1(2)));
            theta2(i_soln) = subproblem.sp_1(real(Rp_3), real(Rp_1), k2);
            theta3(i_soln) = atan2(real(sc3(1)), real(sc3(2)));
            if i_soln == 4; break; end
        end
    end
end

theta1 = theta1(1:i_soln);
theta2 = theta2(1:i_soln);
theta3 = theta3(1:i_soln);

end

function [P, R] = circle_polynomials(p0_i, k_i, p_i, p_i_s, k2)
% r^2 + z^2 = P(z) +- sqrt(R(z))
% Reperesent polynomials P_i, R_i as coefficient vectors
% (Highest powers of z first)

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

R = [-1 2*delta -delta^2]; % -(z-delta_i)^2
R(end) = R(end) + norm_kiXpi_sq*norm_kiXk2_sq; % ||A_i' k_2||^2 - (H-delta_i)^2
R = (2*beta)^2 * R;

end