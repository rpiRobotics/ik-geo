% Robot inverse kinematics example: ABB IRB-6640
% 6-DOF robot with spherical wrist and 2 parallel axes

% Set Up Inverse Kinematics Problem
% Define kinematic parameters
zv = [0;0;0];
ex = [1;0;0];
ey = [0;1;0];
ez = [0;0;1];

P.kin.H = [ez ey ey ex ey ex];
P.kin.P = [zv, 0.32*ex+0.78*ez, 1.075*ez, 1.1425*ex+0.2*ez, zv, zv, 0.2*ex];
P.kin.joint_type = zeros([6 1]);

%%
P.kin = hardcoded_IK_setups.IRB_6640.get_kin();
%P.kin.P(:,end) = 0;

% Pick a joint configuration find the associated end effector pose
q_true = rand_angle([6 1]);
[P.R, P.T] = fwdkin(P.kin, q_true);

%%
RT = NaN([4 3])
RT(:) = [-0.5703122771151268 0.2400099363883497 -0.7855820370951839 -0.43436285148318904 0.6801671472696081 0.6741969003213641 -0.28780408505125055 0.5040335387041744 0.4605611342122582 -0.6984652962258932 -0.5477496431958613 0.23125507107533388]
RT = RT';

P.R = RT(:, 1:3);
P.T = RT(:,end);

%% Solve inverse kinematics using h_2 // h_3

[S.Q, S.is_LS] = IK.IK_spherical_2_parallel(P.R, P.T, P.kin);
S.Q
S.is_LS
[e, e_R, e_T] = IK_setups.IK_spherical_2_parallel.error(P,S)