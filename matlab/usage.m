help subproblem

%% Subproblem 1

help subproblem.sp_1
[P, S] = subproblem_setups.sp_1.setup();
S_t.theta = subproblem.sp_1(P.p1,P.p2,P.k);
subproblem_setups.sp_1.error(P, S_t)

%% Subproblem 2

help subproblem.sp_2
[P, S] = subproblem_setups.sp_2.setup();
[S_t.theta1, S_t.theta2] = subproblem.sp_2(P.p1,P.p2,P.k1,P.k2);
subproblem_setups.sp_2.error(P,S_t)

%% Subproblem 3

help subproblem.sp_3
[P, S] = subproblem_setups.sp_3.setup();
S_t.theta = subproblem.sp_3(P.p1,P.p2,P.k,P.d);
subproblem_setups.sp_3.error(P, S_t)
%% Subproblem 4

help subproblem.sp_4
[P, S] = subproblem_setups.sp_4.setup();
S_t.theta = subproblem.sp_4(P.h,P.p,P.k,P.d);
subproblem_setups.sp_4.error(P, S_t)

%% Subproblem 5

help subproblem.sp_5
[P, S] = subproblem_setups.sp_5.setup();
[S_t.theta1, S_t.theta2, S_t.theta3] = ...
    subproblem.sp_5(P.p0,P.p1,P.p2,P.p3,P.k1,P.k2,P.k3);
subproblem_setups.sp_5.error(P,S_t)

%% Subproblem 6

help subproblem.sp_6
[P, S] = subproblem_setups.sp_6.setup();
[S_t.theta1, S_t.theta2] = subproblem.sp_6( ...
    P.H, P.K, P.P, P.d1, P.d2);
subproblem_setups.sp_6.error(P,S_t)