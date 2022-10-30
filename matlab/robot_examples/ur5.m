% robot = importrobot("universalUR5.urdf")
% 
% show(robot, "Visuals","off")
% %show(robot)
% 
% for i = 1:robot.NumBodies
%     robot.Bodies{i}.Name
%     robot.Bodies{i}.Joint.JointToParentTransform
% end
% 
% %% 
zv = [0;0;0];
ex = [1;0;0];
ey = [0;1;0];
ez = [0;0;1];

P.kin.H = [ez ey ey ey -ez ey];
P.kin.P = [0.089159*ez, 0.1358*ey, -0.1197*ey+0.425*ex, 0.3922*ex, 0.093*ey, -0.0946*ez, 0.0823*ey];
P.kin.joint_type = zeros([6 1]);

% Pick a joint configuration find the associated end effector pose
q_true = rand_angle([6 1]);
[P.R, P.T] = fwdkin(P.kin, q_true);



[S.Q, S.is_LS] = IK.IK_3_parallel_2_intersecting(P.R, P.T, P.kin);
S.Q
S.is_LS
IK_setups.IK_3_parallel_2_intersecting.error(P,S)