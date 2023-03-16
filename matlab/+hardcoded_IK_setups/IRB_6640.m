classdef IRB_6640

methods (Static)
    function kin = get_kin()
        zv = [0;0;0];
        ex = [1;0;0];
        ey = [0;1;0];
        ez = [0;0;1];
        kin.H = [ez ey ey ex ey ex];
        kin.P = [zv, 0.32*ex+0.78*ez, 1.075*ez, 1.1425*ex+0.2*ez, zv, zv, 0.2*ex];
        kin.joint_type = zeros([6 1]);
    end

    function [P, S] = setup()
        S.Q = rand_angle([6,1]);
        [P.R, P.T] = fwdkin(hardcoded_IK_setups.IRB_6640.get_kin(), S.Q);
    end

    function S = run(P)
        [S.Q, S.is_LS] = hardcoded_IK.IRB_6640(P.R, P.T);
    end

    function S = run_mex(P)
        [S.Q, S.is_LS] = hardcoded_IK.IRB_6640_mex(P.R, P.T);
    end

    function [e, e_R, e_T] = error(P,S)
        P.kin = hardcoded_IK_setups.IRB_6640.get_kin();
        [e, e_R, e_T] = robot_IK_error(P, S);
    end
end
end