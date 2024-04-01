function [theta1, theta2, y_roots] = sp_6_complex(H, K, P, d1, d2)
% Approximate continuous version of Subproblem 6
% Returns uses real part of complex zeros to always return 4 solutions

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

[xi_1, xi_2, y_roots] = solve_2_ellipse_numeric_LS(x_min(1:2), x_null(1:2,:), x_min(3:4), x_null(3:4,:));
%[xi_1, xi_2] = subproblem.solve_2_ellipse_symb(x_min(1:2), x_null(1:2,:), x_min(3:4), x_null(3:4,:));

% solve_2_ellipse_symb(x_min(1:2), x_null(1:2,:), x_min(3:4), x_null(3:4,:));
% hold on
% plot(xi_1, xi_2, 'kx')
% hold off

theta1 = NaN(size(xi_1));
theta2 = NaN(size(xi_2));
for i = 1:length(xi_1)
    x = x_min + x_null_1*xi_1(i) + x_null_2*xi_2(i);
    theta1(i) = atan2(real(x(1)),real(x(2)));
    theta2(i) = atan2(real(x(3)),real(x(4)));
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


function [xi_1, xi_2, y_roots] = solve_2_ellipse_numeric_LS(xm1, xn1, xm2, xn2)
% subproblem.solve_2_ellipse_numeric Numeric Ellipse Intersection Solution
% Solve for intersection of 2 ellipses defined by
%
% xm1'*xm1 + xi'*xn1'*xn1*xi  + xm1'*xn1*xi == 1
% xm2'*xm2 + xi'*xn2'*xn2*xi  + xm2'*xn2*xi == 1
% Where xi = [xi_1; xi_2]
%
% https://elliotnoma.wordpress.com/2013/04/10/a-closed-form-solution-for-the-intersections-of-two-ellipses/


A_1 = xn1'*xn1;
a = A_1(1,1);
b = 2*A_1(2,1);
c = A_1(2,2);
B_1 = 2*xm1'*xn1;
d = B_1(1);
e = B_1(2);
f = xm1'*xm1-1;

A_2 = xn2'*xn2;
a1 = A_2(1,1);
b1 = 2*A_2(2,1);
c1 = A_2(2,2);
B_2 = 2*xm2'*xn2;
d1 = B_2(1);
e1 = B_2(2);
fq = xm2'*xm2-1;

% f1 :  a*x^2 + b*x*y + c*y^2 + d*x + e*y + f = 0
% f2 :  a1*x^2 + b1*x*y + c1*y^2 + d1*x + e1*y + fq = 0

% z0 + z1 * y + z2 * y2 + z3 * y3 + z4 * y4  = 0
z0 = f*a*d1^2+a^2*fq^2-d*a*d1*fq+a1^2*f^2-2*a*fq*a1*f-d*d1*a1*f+a1*d^2*fq;

z1 = e1*d^2*a1-fq*d1*a*b-2*a*fq*a1*e-f*a1*b1*d+2*d1*b1*a*f+2*e1*fq*a^2+d1^2*a*e-e1*d1*a*d-...
2*a*e1*a1*f-f*a1*d1*b+2*f*e*a1^2-fq*b1*a*d-e*a1*d1*d+2*fq*b*a1*d;

z2 = e1^2*a^2+2*c1*fq*a^2-e*a1*d1*b+fq*a1*b^2-e*a1*b1*d-fq*b1*a*b-2*a*e1*a1*e+...
2*d1*b1*a*e-c1*d1*a*d-2*a*c1*a1*f+b1^2*a*f+2*e1*b*a1*d+e^2*a1^2-c*a1*d1*d-...
e1*b1*a*d+2*f*c*a1^2-f*a1*b1*b+c1*d^2*a1+d1^2*a*c-e1*d1*a*b-2*a*fq*a1*c;

z3 = -2*a*a1*c*e1+e1*a1*b^2+2*c1*b*a1*d-c*a1*b1*d+b1^2*a*e-e1*b1*a*b-2*a*c1*a1*e-...
e*a1*b1*b-c1*b1*a*d+2*e1*c1*a^2+2*e*c*a1^2-c*a1*d1*b+2*d1*b1*a*c-c1*d1*a*b;

z4 = a^2*c1^2-2*a*c1*a1*c+a1^2*c^2-b*a*b1*c1-b*b1*a1*c+b^2*a1*c1+c*a*b1^2;

y = subproblem.quartic_roots([z4 z3 z2 z1 z0]); %y = roots([z4 z3 z2 z1 z0]);
y_roots = y;
% y = y( abs(imag(y)) < 1e-6 ); %y = y(y==real(y));
% y = real(y);
% y = subproblem.uniquetol_manual(y);

x = -(a*fq+a*c1*y.^2-a1*c*y.^2+a*e1*y-a1*e*y-a1*f)./(a*b1*y+a*d1-a1*b*y-a1*d);

xi_1 = x; xi_2 = y;
end