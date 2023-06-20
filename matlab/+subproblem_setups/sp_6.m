classdef sp_6
    methods (Static)
        function [P, S] = setup()
            P.H = rand_normal_vec(4);
            P.K = rand_normal_vec(4);
            P.P = rand_vec(4);

            
            S.theta1 = rand_angle;
            S.theta2 = rand_angle;
            
            P.d1 =  P.H(:,1)'* rot(P.K(:,1), S.theta1) * P.P(:,1) ...
                  + P.H(:,2)'* rot(P.K(:,2), S.theta2) * P.P(:,2);
            P.d2 =  P.H(:,3)'* rot(P.K(:,3), S.theta1) * P.P(:,3) ...
                  + P.H(:,4)'* rot(P.K(:,4), S.theta2) * P.P(:,4);
        end

        function S = run(P)
        [S.theta1, S.theta2] = subproblem.sp_6(P.H, P.K, P.P, P.d1, P.d2);
        end

        function e = error(P, S)  
            for i = 1:length(S.theta1)
                e_i = norm([P.H(:,1)'* rot(P.K(:,1), S.theta1(i)) * P.P(:,1) ...
                          + P.H(:,2)'* rot(P.K(:,2), S.theta2(i)) * P.P(:,2) - P.d1
                            P.H(:,3)'* rot(P.K(:,3), S.theta1(i)) * P.P(:,3) ...
                          + P.H(:,4)'* rot(P.K(:,4), S.theta2(i)) * P.P(:,4) - P.d2]);
                if i == 1
                    e = e_i;
                else
                    e = e + e_i;
                end
            end
        end
    end
end