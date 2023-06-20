classdef sp_5
    methods (Static)
        function [P, S] = setup()
            P.p1 = rand_vec;
            P.p2 = rand_vec;
            P.p3 = rand_vec;
            
            P.k1 = rand_normal_vec;
            P.k2 = rand_normal_vec;
            P.k3 = rand_normal_vec;
            
            S.theta1 = rand_angle;
            S.theta2 = rand_angle;
            S.theta3 = rand_angle;
            
            P.p0 = -(rot(P.k1,S.theta1)*P.p1 ...
                   -rot(P.k2,S.theta2)*(P.p2+rot(P.k3,S.theta3)*P.p3));
        end

        function S = run(P)
            [S.theta1, S.theta2, S.theta3] = ...
                subproblem.sp_5(P.p0,P.p1,P.p2,P.p3,P.k1,P.k2,P.k3);
        end

        function e = error(P, S)
            e = NaN;
            for i = 1:length(S.theta1)
                e_i = norm(P.p0 + rot(P.k1,S.theta1(i))*P.p1 - rot(P.k2,S.theta2(i))*(P.p2+rot(P.k3,S.theta3(i))*P.p3));
                if i == 1
                    e = e_i;
                else
                    e = e + e_i;
                end
            end
        end
    end
end