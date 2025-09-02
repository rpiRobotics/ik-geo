L1 = 1.5;
L2 = 1;

r_inner = L1 - L2;
r_outer = L1 + L2;

T = [-1.2;1.2;0];
Q = planar_bot_IK(T, L1, L2);

diagrams.setup([2 3]); hold on
view(2);

plot_planar_bot_dots(L1, L2, 0, pi);
plot_planar_bot_dots(L1, L2, pi/3, 0);

plot_planar_bot_dots(L1, L2, Q(1,1), Q(2,1));
plot_planar_bot_dots(L1, L2, Q(1,2), Q(2,2));

diagrams.circle(zv, ez, r_inner, LineStyle=":");
diagrams.circle(zv, ez, r_outer, LineStyle=":");
% diagrams.arrow(zv, T);

diagrams.redraw(); hold off
