classdef sp_1
    methods (Static)
        function [P, S] = setup()
            P.p1 = rand_vec;
            P.k = rand_normal_vec;
            S.theta = rand_angle;

            P.p2 = rot(P.k,S.theta)*P.p1;
        end

        function P = setup_LS()
            P.p1 = rand_vec;
            P.k = rand_normal_vec;

            P.p2 = rand_vec;
        end

        function S = run(P)
            S.theta = subproblem.sp_1(P.p1,P.p2,P.k);
        end

        function e = error(P, S)
            e = norm(P.p2 - rot(P.k,S.theta)*P.p1);
        end
    end
end