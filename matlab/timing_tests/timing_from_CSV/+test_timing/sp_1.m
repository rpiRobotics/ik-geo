function [T_avg, Q_testing] = sp_1(P_list)   
    setup = subproblem_setups.sp_1;
    
    N = 10e3;
    Q1_testing = NaN(4, N);
    
    tic
    for i = 1:N
        S_i = setup.run(P_list(i));
        Q1_testing(1:width(S_i.theta), i) = S_i.theta;
    end
    T = toc;
    
    T_avg = T/N;
    Q_testing = Q1_testing;
end