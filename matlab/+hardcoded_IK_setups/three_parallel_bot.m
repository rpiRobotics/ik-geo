classdef three_parallel_bot

methods (Static)
    function kin = get_kin()
        zv = [0;0;0];
        ex = [1;0;0];
        ey = [0;1;0];
        ez = [0;0;1];
        
        kin.H = [ez ex ex ex ez ex];
        kin.P = [ez, ey, ey, ey, ey, ey+ex, ex];
        kin.joint_type = zeros([6 1]);
    end

    function [P, S] = setup()
        S.Q = rand_angle([6,1]);
        [P.R, P.T] = fwdkin(hardcoded_IK_setups.three_parallel_bot.get_kin(), S.Q);
    end
    
    function S = run(P)
        [S.Q, S.is_LS] = hardcoded_IK.three_parallel_bot(P.R, P.T);
    end

    function S = run_mex(P)
        [S.Q, S.is_LS] = hardcoded_IK.three_parallel_bot_mex(P.R, P.T);
    end

    function [e, e_R, e_T] = error(P,S)
        P.kin = hardcoded_IK_setups.three_parallel_bot.get_kin();
        [e, e_R, e_T] = robot_IK_error(P, S);
    end
end
end