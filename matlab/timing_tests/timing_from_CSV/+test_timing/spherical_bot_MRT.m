function [T_avg, Q_testing] = spherical_bot_MRT(P_list)
N = 10e3;

Q_testing = NaN(6,16, N);

tic
for i = 1:N
    Q = robotIK_spherical_bot([P_list(i).R P_list(i).T; 0 0 0 1])'; % Matlab Robotics Toolbox Solver
    Q_testing(:,1:width(Q),i) = Q;
end
T = toc;

T_avg = T/N;
end