syms x real
syms y real


xm_1  = [0.1;1];
xn1_1  = [3;1];
xn2_1  = [1;1];
xn1 = [xn1_1 xn2_1];
xi = [x;y];


xm_2  = [1;0];
xn1_2  = [4;0];
xn2_2  = [1;2];
xn2 = [xn1_2 xn2_2];

[xi_1, xi_2] = subproblem.solve_2_ellipse_symb(xm_1, xn1, xm_2, xn2)

hold on
plot(xi_1, xi_2, 'ro', 'MarkerFaceColor','auto')
hold off

[xi_1_num, xi_2_num] = subproblem.solve_2_ellipse_numeric(xm_1, xn1, xm_2, xn2)
hold on
plot(xi_1_num, xi_2_num, 'rx')
hold off
    