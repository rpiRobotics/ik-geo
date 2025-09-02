% setup = hardcoded_IK_setups.yumi_fixed_q3;
% setup = hardcoded_IK_setups.RRC_fixed_q6;
% setup = hardcoded_IK_setups.spherical_bot;
% setup = hardcoded_IK_setups.KUKA_R800_fixed_q3;
% setup = hardcoded_IK_setups.IRB_6640;
% setup = hardcoded_IK_setups.three_parallel_bot;
setup = hardcoded_IK_setups.ur5;

[P, S] = setup.setup;

RT_t = [P.R P.T]';

vpa(RT_t(:)')
S.Q
%%
S_run = setup.run(P);
S_run.is_LS
setup.error(P,S_run)
%%
S_ikfast.Q = [ -2.767094232221555, -3.121155023575794, -2.525485165259557, 3.131870669845513, -1.543617028688230, -2.738217582052697]';
setup.error(P,S_ikfast)

%%
RT = P.R';
str_arr = string(vpa([ RT(:)' P.T']));
str_arr = strjoin(str_arr, " ");
disp(strcat(str_arr{:}))
