 setup = IK_setups.IK_spherical_2_parallel;
% setup = IK_setups.IK_spherical_2_intersecting;
% setup = IK_setups.IK_3_parallel_2_intersecting;

 P = setup.setup_LS()

% zv = [0;0;0];
% ex = [1;0;0];
% ey = [0;1;0];
% ez = [0;0;1];
% 
% P.kin.H = [ez ey ey ey ez ey];
% P.kin.P = 10*[ez, ex, ex+ez, ex+ez, ex, zv, zv];
% P.kin.joint_type = zeros([6 1]);
% 
% P.T = 10*[5; 2; 2];
% %P.R = eye(3);
% P.R = rot(ex, pi/2)*rot(ey, 0.1)*rot(ez, 0.1);

S = setup.run(P)

[e, e_R, e_T] = setup.error(P,S)
%% 

%for i = 1:width(S.Q)
for i = 2
q = S.Q(:,i);

J = robotjacobian(P.kin, q);
[~, T] = fwdkin(P.kin, q);

err_vec = P.T - T;
assert(norm(err_vec) > 1e-2)
err_vec = err_vec / norm(err_vec);


%pinv(J)*[0;0;0; err_vec]
J'*[0;0;0; err_vec]

%null(J(1:3,:))' * pinv(J(4:6,:)) * err_vec
%null(J(4:6,:))
%null(J(1:3,:))' * null(J(4:6,:))
end

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

diagrams.line(p_01, p_01+kin.H(:,1), color=diagrams.colors.blue);
diagrams.line(p_02, p_02+kin.H(:,2), color=diagrams.colors.blue);
diagrams.line(p_03, p_03+kin.H(:,3), color=diagrams.colors.blue);
diagrams.line(p_04, p_04+kin.H(:,4), color=diagrams.colors.blue);
diagrams.line(p_05, p_05+kin.H(:,5), color=diagrams.colors.blue);
diagrams.line(p_06, p_06+kin.H(:,6), color=diagrams.colors.blue);
diagrams.redraw();
hold off
%%
norm(P.T - p_01)
norm(p_04 - p_01)
%%
q_iter = q;

for i = 1:1
J = robotjacobian(P.kin, q_iter);
[~, T_iter] = fwdkin(P.kin, q_iter);
err_vec = P.T - T_iter;
norm(err_vec)
delta_q = pinv(J, 1e-4)*[0;0;0; err_vec];
%q_iter = q_iter + 1e-2*delta_q/max(delta_q);
q_iter = q_iter + 1e-6*delta_q;
end

%S_iter.Q = q_iter;
%[e_iter, e_R_iter, e_T_iter] = setup.error(P,S_iter)