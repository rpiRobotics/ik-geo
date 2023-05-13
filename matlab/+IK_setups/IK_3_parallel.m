classdef IK_3_parallel
    % h_2 h_3 h_4 parallel

    methods (Static)
        function [P, S] = setup()
            S.Q = rand_angle([6 1]);
            P.kin.joint_type = zeros(1,6);

            P.kin.H = rand_normal_vec(6);
            P.kin.H(:,3) = P.kin.H(:,2);
            P.kin.H(:,4) = P.kin.H(:,2);

            P.kin.P = rand_vec(7);

            [P.R, P.T] = fwdkin(P.kin, S.Q);
        end

        function S = run(P)
            [S.Q, S.is_LS] = IK.IK_3_parallel(P.R, P.T, P.kin);
        end

        function S = run_mex(P)
            [S.Q, S.is_LS] = IK.IK_3_parallel_mex(P.R, P.T, P.kin);
        end

        function generate_mex()
            P = IK_setups.IK_3_parallel.setup(); %#ok<NASGU> 
            codegen -report +IK/IK_3_parallel.m -args {P.R, P.T, P.kin}
        end

        function [e, e_R, e_T] = error(P, S)
            e_R = NaN([1 width(S.Q)]);
            e_T = NaN([1 width(S.Q)]);
            for i = 1:width(S.Q)
                [R_t, T_t] = fwdkin(P.kin, S.Q(:,i));
                e_R(i) = norm(R_t - P.R);
                e_T(i) = norm(T_t - P.T);
            end
            e = e_R + e_T;
        end
    end
end