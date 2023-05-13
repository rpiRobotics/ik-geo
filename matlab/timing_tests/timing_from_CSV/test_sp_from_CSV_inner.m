function [T_avg, Q_testing] = test_sp_from_CSV_inner(P_list)   
% setup = subproblem_setups.sp_1; % 1
% setup = subproblem_setups.sp_2; % 2
% setup = subproblem_setups.sp_2E; % 2
% setup = subproblem_setups.sp_3; % 1
% setup = subproblem_setups.sp_4; % 1
% setup = subproblem_setups.sp_5; % 3
setup = subproblem_setups.sp_6; % 2

    N = 10e3;
    Q1_testing = NaN(4, N);
    Q2_testing = NaN(4, N);
%     Q3_testing = NaN(4, N);
    
    tic
    for i = 1:N
        S_i = setup.run(P_list(i));
%         Q1_testing(1:width(S_i.theta), i) = S_i.theta;
        Q1_testing(1:numel(S_i.theta1), i) = S_i.theta1;
        Q2_testing(1:numel(S_i.theta2), i) = S_i.theta2;
%         Q3_testing(1:width(S_i.theta3), i) = S_i.theta3;
    end
    T = toc;
    
    T_avg = T/N;
%     Q_testing = Q1_testing;
    Q_testing = [Q1_testing; Q2_testing];
%     Q_testing = [Q1_testing; Q2_testing; Q3_testing];
end