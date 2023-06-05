function [T_avg, Q_testing] = IK_3_parallel(P_list)
setup = IK_setups.IK_3_parallel;
N = 10e3;

Q_testing = NaN(6,16, N);

tic
for i = 1:N
    S_i = setup.run(P_list(i));
    Q_testing(:,1:width(S_i.Q),i) = S_i.Q;
end
T = toc;

T_avg = T/N;
end