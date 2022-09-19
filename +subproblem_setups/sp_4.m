classdef sp_4
    methods (Static)
        function [P, S] = setup()
            P.p = rand_vec;
            P.k = rand_normal_vec;
            P.h = rand_normal_vec;
            S.theta = rand_angle;
            
            P.d = P.h'*rot(P.k,S.theta)*P.p;
        end

        function [P, S] = setup_LS()
            P.p = rand_vec;
            P.k = rand_normal_vec;
            P.h = rand_normal_vec;
            S.theta = rand_angle;
            
            P.d = rand;
        end

        function S = run(P)
            S.theta = subproblem.sp_4(P.h,P.p,P.k,P.d);
        end

        function S = run_grt(P)
            S.theta = subproblem4(P.h,P.p,P.k,P.d);
        end

        function S = run_mex(P)
            S.theta = subproblem.sp_4_mex(P.h,P.p,P.k,P.d);
        end

        function generate_mex()
            P = subproblem_setups.sp_4.setup();
            codegen -report +subproblem/sp_4.m -args {P.h,P.p,P.k,P.d}
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