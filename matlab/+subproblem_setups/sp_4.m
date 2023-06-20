classdef sp_4
    methods (Static)
        function [P, S] = setup()
            P.p = rand_vec;
            P.k = rand_normal_vec;
            P.h = rand_normal_vec;
            S.theta = rand_angle;
            
            P.d = P.h'*rot(P.k,S.theta)*P.p;
        end

        function P = setup_LS()
            P.p = rand_vec;
            P.k = rand_normal_vec;
            P.h = rand_normal_vec;
            
            P.d = rand;
        end

        function S = run(P)
            S.theta = subproblem.sp_4(P.h,P.p,P.k,P.d);
        end

        function e = error(P, S)
            for i = 1:length(S.theta)
                e_i = norm(P.h'*rot(P.k,S.theta(i))*P.p - P.d);
                if i == 1
                    e = e_i;
                else
                    e = e + e_i;
                end
            end
        end
    end
end