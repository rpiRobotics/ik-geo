    function [theta1, theta2] = sp_6(H, K, P, d1, d2)
% subproblem.sp_6  Subproblem 6: Four circles
%   [theta1, theta2] = subproblem.sp_6(H, K, P, d1, d2) finds
%   theta1, theta2 such that
%       h1'*rot(k1,theta1)*p1 + h2'*rot(k2,theta2)*p2 = d1
%       h3'*rot(k3,theta1)*p3 + h4'*rot(k4,theta2)*p4 = d2
%   where
%       H = [h1 h2 h3 h4]
%       K = [k1 k2 k3 k4]
%       P = [p1 p2 p3 p4]
%
%   If the problem is well-posed, there may be up to 4 solutions
%   theta1 and theta2 are column vectors of the solutions
%
%   Inputs:
%       H: 3x4 matrix with norm(h_i) = 1
%       K: 3x4 matrix with norm(k_i) = 1
%       P: 3x4 matrix
%   Outputs:
%       theta1: 1xN angle (in radians)
%           (N is the number of solutions)
%       theta2: 1xN angle (in radians)

k1Xp1 = cross(K(:,1),P(:,1)); k2Xp2 = cross(K(:,2),P(:,2));
k3Xp3 = cross(K(:,3),P(:,3)); k4Xp4 = cross(K(:,4),P(:,4));
A_1 = [k1Xp1, -cross(K(:,1), k1Xp1)];
A_2 = [k2Xp2, -cross(K(:,2), k2Xp2)];
A_3 = [k3Xp3, -cross(K(:,3), k3Xp3)];
A_4 = [k4Xp4, -cross(K(:,4), k4Xp4)];

A = [H(:,1)'*A_1 H(:,2)'*A_2
     H(:,3)'*A_3 H(:,4)'*A_4];

% Minimum norm solution
% x_min = pinv(A)*[d1-h'*k1*k1'*p1-h'*k2*k2'*p2
%                  d2-h'*k1*k1'*p3-h'*k2*k2'*p4];

% % Minimum norm solution, plus some vector from the null space (so Ax=b)
% % The \ operator is fast but doesn't necessarily give the least-squares solution
% % but we are adding in a vector from the null space anyways later on
% x_min = A\[d1-H(:,1)'*K(:,1)*K(:,1)'*P(:,1)-H(:,2)'*K(:,2)*K(:,2)'*P(:,2)
%            d2-H(:,3)'*K(:,3)*K(:,3)'*P(:,3)-H(:,4)'*K(:,4)*K(:,4)'*P(:,4)];
% 
% % Null space
% x_null = qr_null(A); % x_null = null(A);

% Minimum norm solution and null space found simultaneously using QR
% decomposition
b = [d1-H(:,1)'*K(:,1)*K(:,1)'*P(:,1)-H(:,2)'*K(:,2)*K(:,2)'*P(:,2)
     d2-H(:,3)'*K(:,3)*K(:,3)'*P(:,3)-H(:,4)'*K(:,4)*K(:,4)'*P(:,4)];
[x_min, x_null] = qr_LS_and_null(A, b);

x_null_1 = x_null(:,1);
x_null_2 = x_null(:,2);

% solve intersection of 2 ellipses

[xi_1, xi_2] = subproblem.solve_2_ellipse_numeric(x_min(1:2), x_null(1:2,:), x_min(3:4), x_null(3:4,:));
%[xi_1, xi_2] = subproblem.solve_2_ellipse_symb(x_min(1:2), x_null(1:2,:), x_min(3:4), x_null(3:4,:));

% solve_2_ellipse_symb(x_min(1:2), x_null(1:2,:), x_min(3:4), x_null(3:4,:));
% hold on
% plot(xi_1, xi_2, 'kx')
% hold off

theta1 = NaN(size(xi_1));
theta2 = NaN(size(xi_2));
for i = 1:length(xi_1)
    x = x_min + x_null_1*xi_1(i) + x_null_2*xi_2(i);
    theta1(i) = atan2(x(1),x(2));
    theta2(i) = atan2(x(3),x(4));
end

end

function [x, n] = qr_LS_and_null(A, b)
    % QR = A' (Need to take QR of the transpose of A to find n)
    [Q,R] = qr(A');
    n = Q(:,3:4);

    % x = Q R'^-1 b
    % Since A is rank 2, only need to use first 2 cols of Q
    % and first 2x2 submatrix of R
    x = Q(:,1:2) * ((R(1:2,1:2)')\b);
end