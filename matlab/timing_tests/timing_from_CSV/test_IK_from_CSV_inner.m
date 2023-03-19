function [T_avg, Q_testing] = test_IK_from_CSV_inner(P_list)

% setup = IK_setups.IK_gen_6_dof;
% setup = IK_setups.IK_2_intersecting;
% setup = IK_setups.IK_2_parallel;
% setup = IK_setups.IK_spherical;
% setup = IK_setups.IK_spherical_2_intersecting;
% setup = IK_setups.IK_spherical_2_parallel;
% setup = IK_setups.IK_3_parallel;
% setup = IK_setups.IK_3_parallel_2_intersecting;

% setup = hardcoded_IK_setups.yumi_fixed_q3;
setup = hardcoded_IK_setups.RRC_fixed_q6;
% setup = hardcoded_IK_setups.two_parallel_bot;
% setup = hardcoded_IK_setups.spherical_bot;
% setup = hardcoded_IK_setups.KUKA_R800_fixed_q3;
% setup = hardcoded_IK_setups.IRB_6640;
% setup = hardcoded_IK_setups.three_parallel_bot;
% setup = hardcoded_IK_setups.ur5;

setup = coder.const(setup);

%     N = 10e3;
    N = 1e3;
%     N = 100;

    Q_testing = NaN(6,16, N); % each q is 6 long. Up to 16 solns. N poses.
    
    tic
    for i = 1:N
        S_i = setup.run(P_list(i));
        Q_testing(:,1:width(S_i.Q),i) = S_i.Q;

        
%         Q =   robotIK([P_list(i).R P_list(i).T; 0 0 0 1])'; % Matlab Robotics Toolbox Solver
%         Q_testing(:,1:width(Q),i) = Q;
    end
    T = toc;
    
    T_avg = T/N;
end