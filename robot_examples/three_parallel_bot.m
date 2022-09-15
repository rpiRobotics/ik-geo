% Made-up example of a robot with three parallel joints (none intersecting)

zv = [0;0;0];
ex = [1;0;0];
ey = [0;1;0];
ez = [0;0;1];

P.kin.H = [ez ex ex ex ez ex];
P.kin.P = [ez, ey, ey, ey, ey, ey+ex, ex];
P.kin.joint_type = zeros([6 1]);

q_true = rand_angle([6 1]);
[P.R, P.T] = fwdkin(P.kin, q_true);

[S.Q, S.is_LS] = IK.IK_3_parallel(P.R, P.T, P.kin);
S.Q
S.is_LS

e = IK_setups.IK_3_parallel.error(P,S)

%% 
h_fig = diagrams.setup;
view(45, 30)
hold on
diagrams.robot_plot(P.kin, zeros([6 1]));
diagrams.redraw();
hold off