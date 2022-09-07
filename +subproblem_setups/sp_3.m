classdef sp_3
    methods (Static)
        function [P, S] = setup()
            P.p1 = rand_vec;
            P.p2 = rand_vec;
            P.k = rand_normal_vec;
            S.theta = rand_angle;
            
            P.d = norm(P.p2-rot(P.k,S.theta)*P.p1);
        end

        function S = run(P)
            S.theta = subproblem3_linear(P.p1,P.p2,P.k,P.d);
        end

        function S = run_grt(P)
            S = fixed_subproblem3(P.p1,P.p2,P.k,P.d); % TODO
        end

        function S = run_mex(P)
            S.theta = subproblem3_linear_mex(P.p1,P.p2,P.k,P.d);
        end

        function e = error(P, S)
            for i = 1:length(S.theta)
                e_i = norm(P.p2-rot(P.k,S.theta(i))*P.p1) - P.d;
                if i == 1
                    e = e_i;
                else
                    e = e + e_i;
                end
            end
        end
    end
end