classdef sp_2
    methods (Static)
        function [P, S] = setup()
            P.p1 = rand_vec;

            P.k1 = rand_normal_vec;
            P.k2 = rand_normal_vec;

            S.theta1 = rand_angle;
            S.theta2 = rand_angle;

            P.p2 = rot(P.k2, -S.theta2)*rot(P.k1,S.theta1)*P.p1;
        end

        function P = setup_LS()
            P.p1 = rand_vec;

            P.k1 = rand_normal_vec;
            P.k2 = rand_normal_vec;

            P.p2 = rand_vec;
        end

        function S = run(P)
            [S.theta1, S.theta2] = subproblem.sp_2(P.p1,P.p2,P.k1,P.k2);
        end

        function e = error(P, S)
            for i = 1:length(S.theta1)
                e_i = norm(rot(P.k2, S.theta2(i))*P.p2 - rot(P.k1,S.theta1(i))*P.p1);
                if i == 1
                    e = e_i;
                else
                    e = e + e_i;
                end
            end
        end
    end
end