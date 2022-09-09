%setup = IK_setups.IK_spherical_2_parallel;
setup = IK_setups.IK_3_parallel;

[P, S_given] = setup.setup();
setup.error(P,S_given)

global q
q = S_given.Q;

N_trial = 1;
tic
for i = 1:N_trial
S = setup.run(P);
%S = setup.run_mex(P);
end
t  = toc / N_trial


% S_given.Q
% S.Q
% S.is_LS

[e, e_R, e_T] = setup.error(P,S);
e

% S_exact.Q = S.Q(:,~S.is_LS);
% e = setup.error(P, S_exact)