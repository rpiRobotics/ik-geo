function [T_avg, Q_testing] = IK_gen_6_dof(P_list)
setup = IK_setups.IK_gen_6_dof;
%N = 1e3;
N=100;
Q_testing = NaN(6,16, N);

tic
for i = 1:N
    S_i = setup.run(P_list(i));
    Q_i = S_i.Q;
    Q_testing(:,1:min(16,width(S_i.Q)),i) = Q_i(:, 1:min(16, width(Q_i)));
end
T = toc;

T_avg = T/N;
end