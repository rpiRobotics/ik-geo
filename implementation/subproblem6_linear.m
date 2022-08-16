function [theta1, theta2] = subproblem6_linear(h, k1, k2, p1, p2, p3, p4, d1, d2)
% h' rot(k1, theta1) * p1 + h' rot(k2, theta2) * p2 = d1
% h' rot(k1, theta1) * p3 + h' rot(k2, theta2) * p4 = d2

k1Xp1 = cross(k1,p1); k2Xp2 = cross(k2,p2);
k1Xp3 = cross(k1,p3); k2Xp4 = cross(k2,p4);
A_1 = [k1Xp1, -cross(k1, k1Xp1)];
A_2 = [k2Xp2, -cross(k2, k2Xp2)];
A_3 = [k1Xp3, -cross(k1, k1Xp3)];
A_4 = [k2Xp4, -cross(k2, k2Xp4)];

A = [h'*A_1 h'*A_2
     h'*A_3 h'*A_4];

% Minimum norm solution
% x_min = pinv(A)*[d1-h'*k1*k1'*p1-h'*k2*k2'*p2
%                  d2-h'*k1*k1'*p3-h'*k2*k2'*p4];
x_min = A\[d1-h'*k1*k1'*p1-h'*k2*k2'*p2
                 d2-h'*k1*k1'*p3-h'*k2*k2'*p4];

% Null space
x_null = null(A);
x_null_1 = x_null(:,1);
x_null_2 = x_null(:,2);

% solve intersection of 2 ellipses

[xi_1, xi_2] = solve_2_ellipse_numeric(x_min(1:2), x_null(1:2,:), x_min(3:4), x_null(3:4,:));
%[xi_1, xi_2] = solve_2_ellipse_symb(x_min(1:2), x_null(1:2,:), x_min(3:4), x_null(3:4,:));

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