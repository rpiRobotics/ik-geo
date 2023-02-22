classdef IK_spherical
    % h_4, h_5, h_6 all intersect

    methods (Static)
        function [P, S] = setup()
            zv = [0;0;0];

            S.Q = rand_angle([6 1]);
            P.kin.joint_type = zeros(1,6);

            P.kin.H = rand_normal_vec(6);
            P.kin.P = [rand_vec rand_vec rand_vec rand_vec zv zv rand_vec];

            [P.R, P.T] = fwdkin(P.kin, S.Q);
        end

        function P = setup_LS()
            zv = [0;0;0];

            P.kin.joint_type = zeros(1,6);

            P.kin.H = rand_normal_vec(6);
            P.kin.P = [rand_vec rand_vec rand_vec rand_vec zv zv rand_vec];

            P.R = rot(rand_normal_vec, rand_angle);
            P.T = rand_vec;
        end

        function S = run(P)
            [S.Q, S.is_LS] = IK.IK_spherical(P.R, P.T, P.kin);
        end

        function S = run_mex(P)
            [S.Q, S.is_LS] = IK.IK_spherical_mex(P.R, P.T, P.kin);
        end

        function generate_mex()
            P = IK_setups.IK_spherical.setup(); %#ok<NASGU> 
            codegen -report +IK/IK_spherical.m -args {P.R, P.T, P.kin}
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