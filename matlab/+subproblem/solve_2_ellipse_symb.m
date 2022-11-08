function [xi_1, xi_2] = solve_2_ellipse_symb(xm1, xn1, xm2, xn2)
% subproblem.solve_2_ellipse_symb Symbolic Ellipse Intersection Solution
% Using symbolic toolbox, solve for intersection of 2 ellipses defined by
%
% xm1'*xm1 + xi'*xn1'*xn1*xi  + 2*xm1'*xn1*xi == 1
% xm2'*xm2 + xi'*xn2'*xn2*xi  + 2*xm2'*xn2*xi == 1
% Where xi = [xi_1; xi_2]

syms x y real
xi = [x;y];
E1 = xm1'*xm1 + xi'*xn1'*xn1*xi  + 2*xm1'*xn1*xi == 1;
E2 = xm2'*xm2 + xi'*xn2'*xn2*xi  + 2*xm2'*xn2*xi == 1;

s = vpasolve(E1, E2);

xi_1 = double(s.x);
xi_2 = double(s.y);

fimplicit(E1);
hold on
fimplicit(E2); hold off

end