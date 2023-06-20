classdef sp_3
    methods (Static)
        function [P, S] = setup()
            P.p1 = rand_vec;
            P.p2 = rand_vec;
            P.k = rand_normal_vec;
            S.theta = rand_angle;
            
            P.d = norm(P.p2-rot(P.k,S.theta)*P.p1);
        end

        function P = setup_LS()
            P.p1 = rand_vec;
            P.p2 = rand_vec;
            P.k = rand_normal_vec;
            
            P.d = rand;
        end

        function S = run(P)
            S.theta = subproblem.sp_3(P.p1,P.p2,P.k,P.d);
        end

        function e = error(P, S)
            for i = 1:length(S.theta)
                e_i = abs(norm(P.p2-rot(P.k,S.theta(i))*P.p1) - P.d);
                if i == 1
                    e = e_i;
                else
                    e = e + e_i;
                end
            end
        end
    end
end