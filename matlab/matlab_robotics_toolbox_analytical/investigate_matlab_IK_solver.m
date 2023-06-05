% setup = hardcoded_IK_setups.ur5; % fail
% setup = hardcoded_IK_setups.IRB_6640; % success
% setup = hardcoded_IK_setups.two_parallel_bot; % fail
% setup = hardcoded_IK_setups.three_parallel_bot; % fail
% setup = hardcoded_IK_setups.KUKA_R800_fixed_q3; % fail
% setup = hardcoded_IK_setups.yumi_fixed_q3; % fail
% setup = hardcoded_IK_setups.RRC_fixed_q6; % fail
setup = hardcoded_IK_setups.spherical_bot; % success

% setup = IK_setups.IK_gen_6_dof;
% setup = IK_setups.IK_2_intersecting;
% setup = IK_setups.IK_2_parallel;
% setup = IK_setups.IK_spherical;
% setup = IK_setups.IK_spherical_2_intersecting;
% setup = IK_setups.IK_spherical_2_parallel;
% setup = IK_setups.IK_3_parallel;
% setup = IK_setups.IK_3_parallel_2_intersecting;

if ismethod(setup,"get_kin_partial") % hardcoded with fixed q_i
    kin = setup.get_kin_partial;
elseif ismethod(setup,"get_kin") % hardcoded
    kin = setup.get_kin;
else % randomly generated
    P = setup.setup;
    kin = P.kin;
end

robot = kin2robot(kin)

%%
aik = analyticalInverseKinematics(robot);
showdetails(aik)

%%
generateIKFunction(aik,'robotIK');

%% 
generateIKFunction(aik,'robotIK_IRB_6640');

%%
generateIKFunction(aik,'robotIK_spherical_bot');

%%
clc
[P, S_given] = setup.setup();

N = 100;

tic
for i = 1:N
    S.Q =   robotIK([P.R P.T; 0 0 0 1])';
end
toc/N * 1e6

setup.error(P, S)

tic
for i = 1:N
    S_sp = setup.run(P);
end
toc/N * 1e6

setup.error(P, S_sp)