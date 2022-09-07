classdef sp_6
    methods (Static)
        function [P, S] = setup() % TODO
            P.p1 = rand_vec;
            P.p2 = rand_vec;
            P.p3 = rand_vec;
            P.p4 = rand_vec;
            P.k1 = rand_normal_vec;
            P.k2 = rand_normal_vec;
            P.h = rand_normal_vec;
            
            S.theta1 = rand_angle;
            S.theta2 = rand_angle;
            
            P.d1 =  P.h'* rot(P.k1, S.theta1) * P.p1 ...
                  + P.h'* rot(P.k2, S.theta2) * P.p2;
            P.d2 =  P.h'* rot(P.k1, S.theta1) * P.p3 ...
                  + P.h'* rot(P.k2, S.theta2) * P.p4;
        end

        function S = run(P)
        [S.theta1, S.theta2] = subproblem6_linear( ...
        P.h, P.k1, P.k2, P.p1, P.p2, P.p3, P.p4, P.d1, P.d2);
        end

        function S = run_mex(P)
        [S.theta1, S.theta2] = subproblem6_linear_mex( ...
        P.h, P.k1, P.k2, P.p1, P.p2, P.p3, P.p4, P.d1, P.d2);
        end

        function e = error(P, S)  
            for i = 1:length(S.theta1)
                e_i = norm([P.h'* rot(P.k1, S.theta1(i)) * P.p1 + P.h'* rot(P.k2, S.theta2(i)) * P.p2 - P.d1
                            P.h'* rot(P.k1, S.theta1(i)) * P.p3 + P.h'* rot(P.k2, S.theta2(i)) * P.p4 - P.d2]);
                if i == 1
                    e = e_i;
                else
                    e = e + e_i;
                end
            end
        end
    end
end