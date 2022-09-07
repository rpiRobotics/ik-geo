addpath("implementation\")

% Also make sure to add path to general-robotics-toolbox

%% Subproblem 1

[P, S] = subproblem_setups.sp_1.setup();
S_t.theta = subproblem1_linear(P.p1,P.p2,P.k)
subproblem_setups.sp_1.error(P, S_t)

%% Subproblem 2

[P, S] = subproblem_setups.sp_2.setup()
[S_t.theta1, S_t.theta2] = subproblem2_linear(P.p1,P.p2,P.k1,P.k2)
subproblem_setups.sp_2.error(P,S_t)

%% Subproblem 2 Extension

[P, S] = subproblem_setups.sp_2E.setup()
[S_t.theta1,S_t.theta2] = subproblem2_linear_extended(P.p0,P.p1,P.p2,P.k1,P.k2);
subproblem_setups.sp_2E.error(P, S_t)

%% Subproblem 3

[P, S] = subproblem_setups.sp_3.setup()
S_t.theta = subproblem3_linear(P.p1,P.p2,P.k,P.d)
subproblem_setups.sp_3.error(P, S_t)
%% Subproblem 4

[P, S] = subproblem_setups.sp_4.setup()

S_t.theta = subproblem4_linear(P.h,P.p,P.k,P.d)

subproblem_setups.sp_4.error(P, S_t)

%% Subproblem 5

[P, S] = subproblem_setups.sp_5.setup()

% global H_ap
% global N_ap
% global x1_ap
% H_ap = k2'*(p0 + rot(k1,theta1)*p1)
% k2'*(p2+rot(k3,theta3)*p3)
% 
% N_ap = norm(p0 + rot(k1,theta1)*p1 - H_ap*k2)
% norm(p2+rot(k3,theta3)*p3 - H_ap*k2)
% 
% x1_ap = [sin(theta1); cos(theta1)]

[S_t.theta1, S_t.theta2, S_t.theta3] = ...
    subproblem5_linear(P.p0,P.p1,P.p2,P.p3,P.k1,P.k2,P.k3);

subproblem_setups.sp_5.error(P,S_t)

%% Subproblem 6

[P, S] = subproblem_setups.sp_6.setup()

[S_t.theta1, S_t.theta2] = subproblem6_linear( ...
P.h, P.k1, P.k2, P.p1, P.p2, P.p3, P.p4, P.d1, P.d2)

subproblem_setups.sp_6.error(P,S_t)