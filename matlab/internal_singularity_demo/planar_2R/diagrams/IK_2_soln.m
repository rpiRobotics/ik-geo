L1 = 1.5;
L2 = 1;

r_inner = L1 - L2;
r_outer = L1 + L2;

T = [-1.2;1.2;0];
Q = planar_bot_IK(T, L1, L2);

diagrams.setup([2 3]); hold on
view(2);

plot_planar_bot_dots(L1, L2, Q(1,1), Q(2,1));
plot_planar_bot_dots(L1, L2, Q(1,2), Q(2,2));

plot_planar_bot_dots(L1, L2, pi/3, 0);
T2 = rot(ez, pi/3)*ex*r_outer*1.1;
diagrams.dot(T2, color=diagrams.colors.red);

diagrams.circle(zv, ez, r_inner, LineStyle=":");
diagrams.circle(zv, ez, r_outer, LineStyle=":");
% diagrams.arrow(zv, T);

diagrams.circle(zv, ez, L1, color=diagrams.colors.blue);
diagrams.circle(T,  ez, L2, color=diagrams.colors.blue);
diagrams.circle(T2,  ez, L2, color=diagrams.colors.blue);

diagrams.redraw(); hold off
