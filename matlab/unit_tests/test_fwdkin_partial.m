n = 2;

q = rand_angle([7 1]);
%q = zeros([7 1]);
%q(n)=pi;

kin = hardcoded_IK_setups.IRB_6640.get_kin();


[kin_new, R_6T_new] = fwdkin_partial(kin, q(n), n);

[R, T] = fwdkin(kin, q);

[R_partial, T_partial] = fwdkin(kin_new, q([1:n-1, n+1:6]));
R_6T_partial = R_partial * R_6T_new;

R-R_6T_partial
T-T_partial
