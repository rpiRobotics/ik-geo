function [xi_1, xi_2] = solve_2_ellipse_numeric(xm1, xn1, xm2, xn2)
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
y = y( abs(imag(y)) < 1e-6 ); %y = y(y==real(y));
y = real(y);
y = subproblem.uniquetol_manual(y);

x = -(a*fq+a*c1*y.^2-a1*c*y.^2+a*e1*y-a1*e*y-a1*f)./(a*b1*y+a*d1-a1*b*y-a1*d);

xi_1 = x; xi_2 = y;
end