% Test correctness of robot IK least-squares solutions



% setup = IK_setups.IK_spherical_2_parallel;
setup = IK_setups.IK_spherical_2_intersecting;
% setup = IK_setups.IK_3_parallel_2_intersecting;

P = setup.setup_LS()
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

q_iter = q_iter + 1e-2*delta_q;
end
plot(err_hist)
feas_dir' * decreasing_dir
rad2deg(q-q_iter)
1e3*norm(T_iter - T)

1e3*(err_hist(end)-err_hist(1))