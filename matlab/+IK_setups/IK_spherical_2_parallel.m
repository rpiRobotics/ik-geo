classdef IK_spherical_2_parallel
    % h_2 parallel to h_3
    % h_4, h_5, h_6 all intersect

    methods (Static)
        function [P, S] = setup()
            zv = [0;0;0];

            S.Q = rand_angle([6 1]);
            P.kin.joint_type = zeros(1,6);

            P.kin.H = rand_normal_vec(6);
            P.kin.H(:,3) = P.kin.H(:,2);

            P.kin.P = [rand_vec rand_vec rand_vec rand_vec zv zv rand_vec];

            [P.R, P.T] = fwdkin(P.kin, S.Q);
        end

        function P = setup_LS()
            zv = [0;0;0];

            P.kin.joint_type = zeros(1,6);

            P.kin.H = rand_normal_vec(6);
            P.kin.H(:,3) = P.kin.H(:,2);

            P.kin.H(:,1) = rand_perp_normal_vec(P.kin.H(:,2)); % make cylindrical
            
            P.kin.P = [rand_vec rand_vec rand_vec rand_vec zv zv rand_vec];
            P.kin.P(:,end) = 0; % Set task frame at the wrist

            P.kin.P(:,4) = P.kin.P(:,4) - P.kin.H(:,2) * (P.kin.H(:,2)'*(P.kin.P(:,2)+P.kin.P(:,3)+P.kin.P(:,4)));

            P.kin.H(:,5) = rand_perp_normal_vec(P.kin.H(:,4));
            P.kin.H(:,6) = rand_perp_normal_vec(P.kin.H(:,5)); % h_4 _|_ h_5 _|_ h_6 to achieve any orientation 

            P.R = rot(rand_normal_vec, rand_angle);
            P.T = rand_vec;
        end

        function S = run(P)
            [S.Q, S.is_LS] = IK.IK_spherical_2_parallel(P.R, P.T, P.kin);
        end

        function S = run_mex(P)
            [S.Q, S.is_LS] = IK.IK_spherical_2_parallel_mex(P.R, P.T, P.kin);
        end

        function generate_mex()
            P = IK_setups.IK_spherical_2_parallel.setup(); %#ok<NASGU> 
            codegen -report +IK/IK_spherical_2_parallel.m -args {P.R, P.T, P.kin}
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
