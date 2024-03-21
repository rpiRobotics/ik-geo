% Pick a robot with least-squares IK

setup = IK_setups.IK_spherical_2_parallel;

% Normal setup, don't use global LS IK kinematics
P = setup.setup;

%%
% Find a singular pose
P.R = eye(3);
P.T = [0;0;10];
S = setup.run(P);

q = S.Q(:,1)

% Verify singularity condition
J = robotjacobian(P.kin, q)
svd(J)

% Find actual singular pose
[R,T] = fwdkin(P.kin,q)

%%

% Slightly perturb pose and see if we still get that q
P_1 = P;
P_1.R = R;
P_1.T = T + rand_normal_vec*1e-2;
S_1 = setup.run(P_1);
[q_1, i, e_q] = closest_q(S_1.Q, q);
[R_1,T_1] = fwdkin(P.kin,q_1)

norm(T_1 - P_1.T)
norm(T_1 - T)
e_q
S_1.is_LS(i)