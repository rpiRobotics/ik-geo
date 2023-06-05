function [T_avg, Q_testing] = sp_5(P_list)   
    setup = subproblem_setups.sp_5;

    N = 10e3;
    Q1_testing = NaN(4, N);
    Q2_testing = NaN(4, N);
    Q3_testing = NaN(4, N);
    
    tic
    for i = 1:N
        S_i = setup.run(P_list(i));
        Q1_testing(1:numel(S_i.theta1), i) = S_i.theta1;
        Q2_testing(1:numel(S_i.theta2), i) = S_i.theta2;
        Q3_testing(1:width(S_i.theta3), i) = S_i.theta3;
    end
    T = toc;
    
    T_avg = T/N;
    Q_testing = [Q1_testing; Q2_testing; Q3_testing];
end