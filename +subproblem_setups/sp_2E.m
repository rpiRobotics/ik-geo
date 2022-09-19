classdef sp_2E
    methods (Static)
        function [P, S] = setup()
            P.p0 = rand_vec;
            P.p1 = rand_vec;
            
            P.k1 = rand_normal_vec;
            P.k2 = rand_normal_vec;
            
            S.theta1 = rand_angle;
            S.theta2 = rand_angle;
            
            P.p2 = rot(P.k2, -S.theta2)*(P.p0 + rot(P.k1,S.theta1)*P.p1);
        end

        function S = run(P)
            [S.theta1,S.theta2] = subproblem.sp_2E(P.p0,P.p1,P.p2,P.k1,P.k2);
        end

        function S = run_mex(P)
            [S.theta1,S.theta2] = subproblem.sp_2E_mex(P.p0,P.p1,P.p2,P.k1,P.k2);
        end

        function generate_mex()
            P = subproblem_setups.sp_2E.setup();
            codegen -report +subproblem/sp_2E.m -args {P.p0,P.p1,P.p2,P.k1,P.k2}
        end

        function e = error(P, S)
            e = norm(P.p0 + rot(P.k1,S.theta1)*P.p1 - rot(P.k2, S.theta2)*P.p2);
        end
    end
end