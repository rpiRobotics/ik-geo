classdef IK_3_parallel
    % h_2 h_3 h_4 parallel

    methods (Static)
        function [P, S] = setup()
            S.q = rand_angle([6 1]);
            P.kin.joint_type = zeros(1,6);

            P.kin.H = rand_normal_vec(7);
            P.kin.H(:,3) = P.kin.H(:,2);
            P.kin.H(:,4) = P.kin.H(:,2);

            P.kin.P = rand_vec(7);
            P.GC = S.q([2 4 6]) >= 0;

            [P.R, P.T] = fwdkin(P.kin, S.q);
        end

        function S = run(P)
            S.q = IK.IK_3_parallel(P.R, P.T, P.GC, P.kin);
        end

        function S = run_mex(P)
            S.q = IK.IK_3_parallel_mex(P.R, P.T, P.GC, P.kin);
        end

        function [e, e_R, e_T] = error(P, S)
            [R_t, T_t] = fwdkin(P.kin, S.q);
            e_R = norm(R_t - P.R);
            e_T = norm(T_t - P.T);
            e = e_R + e_T;
        end
    end
end