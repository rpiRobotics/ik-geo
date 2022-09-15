ex = [1;0;0];
ey = [0;1;0];
ez = [0;0;1];
zv = zeros(3,1);

l1 = 0.48;
l2 = 0.37;
l3 = 1.06;
l4 = 0.28;
l5 = 1.18;
l6 = 0.37;

P.kin.H = [ez -ey ez -ey ez -ey ez];
P.kin.P = [l1*ez zv [0; -l2; l3] zv [0; l4; l5] zv zv l6*ez];
P.kin.joint_type = zeros(1,7);


P.R = eye(3);
%P.T = 1e-2*[1;2;3];
P.T = rand_vec;

P.sew = sew_conv([1;0;0]);
P.psi = deg2rad(45);

N_trial = 1000;
tic
for i = 1:N_trial
[S.Q, S.is_LS] = IK.IK_2R_2R_3R_mex(P.R, P.T, P.sew, P.psi, P.kin);
end
T = toc/N_trial
f = 1/T
[e, e_R, e_T, e_psi] = IK_setups.IK_2R_2R_3R.error(P, S)
