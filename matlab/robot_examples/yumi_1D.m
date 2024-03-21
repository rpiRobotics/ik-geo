% YuMi with fixed q3
% Axes 5 and 7 always intersect
% (aka axes 4 and 6 once we convert to 6-dof)

kin = hardcoded_IK_setups.orth_yumi_fixed_q3.get_kin_partial;

q = deg2rad([1 2 3 4 5 6]'*10);
% q = deg2rad([1 1 1 1 1 1]*90)

[R_06, p_0T] = fwdkin(kin, q);

[Q, is_LS_vec] = IK.IK_4_6_intersecting(R_06, p_0T, kin)
xline(q(5), 'r');

%%
p_04_prime =...
[   -23.9620
  308.3021
  279.2168];

h_fig = diagrams.setup;
view(-15,30)
hold on
diagrams.robot_plot(kin, q, ...
    unit_size=200, ...
    cyl_half_length=50, ...
    cyl_radius=10)

diagrams.dot(p_04_prime, color=diagrams.colors.green)
hold off
diagrams.redraw();