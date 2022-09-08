classdef IK_gen_6_dof
    methods (Static)
        function [P, S] = setup()
            S.q = rand_angle([6 1]);
            P.kin.joint_type = zeros(1,6);

            P.kin.H = rand_normal_vec(7);
            P.kin.P = rand_vec(7);
            P.GC = S.q([2 4 6]) >= 0;

            [P.R, P.T] = fwdkin(P.kin, S.q);
        end

        function S = run(P)
            S.q = IK.IK_gen_6_dof(P.R, P.T, P.GC, P.kin);
        end

        function S = run_mex(P)
            S.q = IK.IK_gen_6_dof_mex(P.R, P.T, P.GC, P.kin);
        end

        function [e, e_R, e_T] = error(P, S)
            [R_t, T_t] = fwdkin(P.kin, S.q);
            e_R = norm(R_t - P.R);
            e_T = norm(T_t - P.T);
            e = e_R + e_T;
        end
    end
end