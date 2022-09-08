classdef IK_spherical_2_parallel
    % h_2 parallel to h_3
    % h_4, h_5, h_6 all intersect

    methods (Static)
        function [P, S] = setup()
            zv = [0;0;0];

            S.q = rand_angle([6 1]);
            P.kin.joint_type = zeros(1,6);

            P.kin.H = rand_normal_vec(7);
            P.kin.H(:,3) = P.kin.H(:,2);

            P.kin.P = [rand_vec rand_vec rand_vec rand_vec zv zv zv rand_vec];
            P.GC = S.q([2 4 6]) >= 0;

            [P.R, P.T] = fwdkin(P.kin, S.q);
        end

        function P = setup_LS()
            zv = [0;0;0];

            P.kin.joint_type = zeros(1,6);

            P.kin.H = rand_normal_vec(7);
            P.kin.H(:,3) = P.kin.H(:,2);
            
            P.kin.P = [rand_vec rand_vec rand_vec rand_vec zv zv zv rand_vec];
            P.GC = randi(2,[1 3])-1;

            P.R = rot(rand_normal_vec, rand_angle);
            P.T = rand_vec;
        end

        function S = run(P)
            S.q = IK.IK_spherical_2_parallel(P.R, P.T, P.GC, P.kin);
        end

        function S = run_mex(P)
            S.q = IK.IK_spherical_2_parallel_mex(P.R, P.T, P.GC, P.kin);
        end

        function [e, e_R, e_T] = error(P, S)
            [R_t, T_t] = fwdkin(P.kin, S.q);
            e_R = norm(R_t - P.R);
            e_T = norm(T_t - P.T);
            e = e_R + e_T;
        end
    end
end