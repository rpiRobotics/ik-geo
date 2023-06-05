function [T_avg, Q_testing] = IRB_6640(P_list)
setup = hardcoded_IK_setups.IRB_6640;
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