L1 = 1;
L2 = 1;

r_outer = L1 + L2;

ex = [1;0;0];
ey = [0;1;0];
ez = [0;0;1];
zv = [0;0;0];

diagrams.setup([2 3]); hold on
view(2);


T = [0.1;0.1;0];
Q = planar_bot_IK(T, L1, L2);
plot_planar_bot_dots(L1, L2, Q(1,1), Q(2,1));
% plot_planar_bot_dots(L1, L2, Q(1,2), Q(2,2));


T = [0.1;0;0];
Q = planar_bot_IK(T, L1, L2);
plot_planar_bot_dots(L1, L2, Q(1,1), Q(2,1));
% plot_planar_bot_dots(L1, L2, Q(1,2), Q(2,2));

T = [0.1;-0.1;0];
Q = planar_bot_IK(T, L1, L2);
plot_planar_bot_dots(L1, L2, Q(1,1), Q(2,1));
% plot_planar_bot_dots(L1, L2, Q(1,2), Q(2,2));


diagrams.circle(zv, ez, r_outer, LineStyle=":");


diagrams.redraw(); hold off
