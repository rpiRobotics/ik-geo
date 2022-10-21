function [e, e_R, e_T] = robot_IK_error(P, S)
    e_R = NaN([1 width(S.Q)]);
    e_T = NaN([1 width(S.Q)]);
    for i = 1:width(S.Q)
        [R_t, T_t] = fwdkin(P.kin, S.Q(:,i));
        e_R(i) = norm(R_t - P.R);
        e_T(i) = norm(T_t - P.T);
    end
    e = e_R + e_T;
end