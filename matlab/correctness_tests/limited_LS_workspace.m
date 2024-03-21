% Start with a robot which can achieve LS IK for all space
zv = [0;0;0];
ex = [1;0;0];
ey = [0;1;0];
ez = [0;0;1];

P.kin.H = [ez ey ey ex ey ex];
P.kin.P = [zv ex ex ex zv zv zv];
P.kin.joint_type = zeros(6,1);

% Rotate 
P.kin.H(:,2) = rot(ex, deg2rad(45))*ey;
P.kin.H(:,3) = P.kin.H(:,2);
%P.kin.H(:,3) = rot(ex, deg2rad(45))*ey;

%P.T = 2*rand_vec;
P.T = [5;0;6];
P.T = P.T / norm(P.T)*10
P.R = eye(3);

S.Q = IK.IK_spherical_2_parallel(P.R,P.T, P.kin);
% S.Q = IK.IK_spherical_2_intersecting(P.R,P.T, P.kin);
e = robot_IK_error(P,S);
assert(min(e) > 1e-2)
[~, idx_min_error] = min(e);
q = S.Q(:,idx_min_error);
sp_is_LS = is_LS_mat(:,idx_min_error)

J = robotjacobian(P.kin, q);
[R, T] = fwdkin(P.kin, q); 
err_vec = P.T - T;
err_vec = err_vec / norm(err_vec);
J'*[0;0;0; err_vec]

T_normal = P.T / norm(P.T);
[acosd(dot(T_normal, ez)) acosd(dot(T_normal, -ez))]