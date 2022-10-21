% setup = IK_setups.IK_spherical_2_parallel;
% setup = IK_setups.IK_spherical_2_intersecting;
setup = IK_setups.IK_3_parallel_2_intersecting;

%P = setup.setup_LS()
%S = setup.run(P)

zv = [0;0;0];
ex = [1;0;0];
ey = [0;1;0];
ez = [0;0;1];
P.kin.joint_type = zeros(1,6);
P.kin.H = [ez ey ey ey ez ex];
P.kin.P = [zv ex ex ex ex zv zv];
P.R = rot(rand_normal_vec, rand_angle);
%P.T = rand_vec*10;
P.T = [5; 0; 5];


S = setup.run(P)

[e, e_R, e_T] = setup.error(P,S);

% Make sure it's a LS solution, not an exact solution
assert(min(e) > 1e-2)

% Pick the minimum-error solution
[~, idx_min_error] = min(e)
q = S.Q(:,idx_min_error)

%% Check for when rotation and position decoupled
J = robotjacobian(P.kin, q);
[R, T] = fwdkin(P.kin, q);
err_vec = P.T - T;
err_vec = err_vec / norm(err_vec);
J'*[0;0;0; err_vec]

%% Check for any case, including rotation and position coupled
q_iter = q;
[R, T] = fwdkin(P.kin, q);

err_hist = NaN(100, 1);
for i = 1:2000
J = robotjacobian(P.kin, q_iter);
[R_iter, T_iter] = fwdkin(P.kin, q_iter);
err_vec = P.T - T_iter;
err_hist(i) = norm(err_vec);

J_R = J(1:3,:);
J_T = J(4:6,:);
decreasing_dir = J_T' * err_vec; decreasing_dir = decreasing_dir/norm(decreasing_dir);
C_1 = J_R' * hat(R(:,1));
C_2 = J_R' * hat(R(:,2));
feas_mat = [C_1'; C_2'];
feas_dir = null(feas_mat);

delta_q = feas_dir * pinv(feas_dir)*decreasing_dir;

q_iter = q_iter + 1e-2*del  ta_q;
end
plot(err_hist)
feas_dir' * decreasing_dir
rad2deg(q-q_iter)
1e3*norm(T_iter - T)

1e3*(err_hist(end)-err_hist(1))
%% 

%for i = 1:width(S.Q)
for i = 2
q = S.Q(:,i);
%q = rand(6,1)

J = robotjacobian(P.kin, q);
[R, T] = fwdkin(P.kin, q);

err_vec = P.T - T;
err = norm(err_vec);
assert(norm(err_vec) > 1e-2)
err_vec = err_vec / norm(err_vec);

J'*[0;0;0; err_vec]


J_R = J(1:3,:);
J_T = J(4:6,:);

C_0 = J_T' * err_vec;
%R = P.R;
C_1 = J_R' * hat(R(:,1));
C_2 = J_R' * hat(R(:,2));
C_3 = J_R' * hat(R(:,3));
C_123 = [C_1 C_2 C_3];

%C_0 - C_123 * pinv(C_123)*C_0
%null(C_123) * C_0

decreasing_dir = J_T' * err_vec;
decreasing_dir = decreasing_dir/norm(decreasing_dir);
feas_mat = [C_1'; C_2'];
feas_dir = null(feas_mat);

decreasing_dir'* feas_dir
end


delta_q = feas_dir * pinv(feas_dir)*decreasing_dir; delta_q = 1e-3 * delta_q / norm(delta_q);

[~, T_test] = fwdkin(P.kin, q+delta_q);
err_vec_test = norm(P.T - T_test);
(err_vec_test - err)/1e-3
%%
rot_err = NaN(100,1);
for i = 1:100
delta_q = 1e-2*feas_dir * rand_normal_vec;

[R_test, T_test] = fwdkin(P.kin, q+delta_q);
rot_err(i) = norm(R_test(:) - P.R(:));
end
max(rot_err)
%%
delta_err = NaN(100, 1);
decreasing_dir = J_T' * err_vec;
decreasing_dir = decreasing_dir/norm(decreasing_dir);
for i = 1:100
%delta_q = rand(6, 1); delta_q = 1e-1 * delta_q / norm(delta_q);
%delta_q = 1e-1 * decreasing_dir;

%delta_q = 1e-1*feas_dir * rand_normal_vec;
delta_q = feas_dir * pinv(feas_dir)*decreasing_dir; delta_q = 1e-2 * delta_q / norm(delta_q);

[~, T_test] = fwdkin(P.kin, q+delta_q);
err_vec_test = norm(P.T - T_test);
delta_err(i) = err_vec_test - err;
end
min(delta_err)
%%

zv =[0;0;0];
kin = P.kin;

[~, ~, p_inter] = fwdkin_inter(P.kin, q, [1 2 3 4 5 6]);

p_01 = p_inter(:,1);
p_02 = p_inter(:,2);
p_03 = p_inter(:,3);
p_04 = p_inter(:,4);
p_05 = p_inter(:,5);
p_06 = p_inter(:,6);

h_fig = diagrams.setup;
view(0, 0)
axis padded on
hold on
diagrams.arrow(zv, P.T, color=diagrams.colors.red);
diagrams.dot(P.T, color=diagrams.colors.red);

diagrams.dot(p_02);
diagrams.dot(p_03);
diagrams.dot(p_04);

diagrams.line(zv, p_01);
diagrams.line(p_01, p_02);
diagrams.line(p_02, p_03);
diagrams.line(p_03, p_04);
diagrams.line(p_04, p_05);
diagrams.line(p_05, p_06);
diagrams.dot(zv);
diagrams.dot(p_06, color = [0.1 0.9 0.1]);

diagrams.line(P.T - (p_06 - p_04), P.T, color = diagrams.colors.blue);
diagrams.line(p_02, P.T - (p_06 - p_04), LineStyle=':')


[~, ~, p_inter] = fwdkin_inter(P.kin, q_iter, [1 2 3 4 5 6]);

p_01 = p_inter(:,1);
p_02 = p_inter(:,2);
p_03 = p_inter(:,3);
p_04 = p_inter(:,4);
p_05 = p_inter(:,5);
p_06 = p_inter(:,6);

diagrams.dot(p_02);
diagrams.dot(p_03);
diagrams.dot(p_04);
diagrams.dot(p_06);

diagrams.line(p_01, p_02, color=diagrams.colors.red);
diagrams.line(p_02, p_03, color=diagrams.colors.red);
diagrams.line(p_03, p_04, color=diagrams.colors.red);
diagrams.line(p_04, p_05, color=diagrams.colors.red);
diagrams.line(p_05, p_06, color=diagrams.colors.red);

% diagrams.line(p_01, p_01+kin.H(:,1), color=diagrams.colors.blue);
% diagrams.line(p_02, p_02+kin.H(:,2), color=diagrams.colors.blue);
% diagrams.line(p_03, p_03+kin.H(:,3), color=diagrams.colors.blue);
% diagrams.line(p_04, p_04+kin.H(:,4), color=diagrams.colors.blue);
% diagrams.line(p_05, p_05+kin.H(:,5), color=diagrams.colors.blue);
% diagrams.line(p_06, p_06+kin.H(:,6), color=diagrams.colors.blue);
diagrams.redraw();
hold off
%%
norm(P.T - p_01)
norm(p_04 - p_01)
%%
q_iter = q;

err_hist = NaN(100, 1);
for i = 1:1000
J = robotjacobian(P.kin, q_iter);
[R_iter, T_iter] = fwdkin(P.kin, q_iter);
err_vec = P.T - T_iter;
err_hist(i) = norm(err_vec);

J_R = J(1:3,:);
J_T = J(4:6,:);
decreasing_dir = J_T' * err_vec; decreasing_dir = decreasing_dir/norm(decreasing_dir);
C_1 = J_R' * hat(R(:,1));
C_2 = J_R' * hat(R(:,2));
feas_mat = [C_1'; C_2'];
feas_dir = null(feas_mat);

delta_q = feas_dir * pinv(feas_dir)*decreasing_dir;

q_iter = q_iter + 1e-2*delta_q;
end
plot(err_hist)
feas_dir' * decreasing_dir
rad2deg(q-q_iter)
%S_iter.Q = q_iter;
%[e_iter, e_R_iter, e_T_iter] = setup.error(P,S_iter)