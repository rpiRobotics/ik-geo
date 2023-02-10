classdef IK_3_parallel_2_intersecting
    % h_2 h_3 h_4 parallel
    % h_5 intersects h_6

    methods (Static)
        function [P, S] = setup()
            S.Q = rand_angle([6 1]);
            P.kin.joint_type = zeros(1,6);

            P.kin.H = rand_normal_vec(6);
            P.kin.H(:,3) = P.kin.H(:,2);
            P.kin.H(:,4) = P.kin.H(:,2);

            P.kin.P = rand_vec(7);
            P.kin.P(:,6) = 0;

            [P.R, P.T] = fwdkin(P.kin, S.Q);
        end

        function P = setup_LS()
            P.kin.joint_type = zeros(1,6);

            P.kin.H = rand_normal_vec(6);
            P.kin.H(:,3) = P.kin.H(:,2);
            P.kin.H(:,4) = P.kin.H(:,2);

            P.kin.P = rand_vec(7);
            P.kin.P(:,6) = 0;
            P.kin.P(:,5) = 0; % Must have a spherical wrist

            P.kin.P(:,end) = 0; % Set task frame at the wrist
            % h_2 _|_ p_12+p_23+p_34+p_45
            P.kin.P(:,5) = P.kin.P(:,5) - P.kin.H(:,2) * (P.kin.H(:,2)'*(P.kin.P(:,2)+P.kin.P(:,3)+P.kin.P(:,4)+P.kin.P(:,5)));
            
            P.kin.H(:,1) = rand_perp_normal_vec(P.kin.H(:,2)); % h_1 _|_ h_2

            P.kin.H(:,5) = rand_perp_normal_vec(P.kin.H(:,4));
            P.kin.H(:,6) = rand_perp_normal_vec(P.kin.H(:,5)); % h_4 _|_ h_5 _|_ h_6

            P.R = rot(rand_normal_vec, rand_angle);
            P.T = rand_vec;
        end

        function S = run(P)
            [S.Q, S.is_LS] = IK.IK_3_parallel_2_intersecting(P.R, P.T, P.kin);
        end

        function S = run_mex(P)
            [S.Q, S.is_LS] = IK.IK_3_parallel_2_intersecting_mex(P.R, P.T, P.kin);
        end

        function generate_mex()
            P = IK_setups.IK_3_parallel_2_intersecting.setup(); %#ok<NASGU> 
            codegen -report +IK/IK_3_parallel_2_intersecting.m -args {P.R, P.T, P.kin}
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