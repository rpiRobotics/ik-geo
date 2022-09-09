setup = IK_setups.IK_spherical_2_parallel;

[P, S_given] = setup.setup();

N_trial = 1000;
tic
for i = 1:N_trial
%S = setup.run(P);
S = setup.run_mex(P);
end
t  = toc / N_trial

% 
% S_given.q
% S.Q
% S.is_LS

[e, e_R, e_T] = setup.error(P,S);
e

S_exact.Q = S.Q(:,~S.is_LS);
e = setup.error(P, S_exact)