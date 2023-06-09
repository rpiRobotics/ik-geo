classdef ur5

methods (Static)
    function kin = get_kin()
        zv = [0;0;0];
        ex = [1;0;0];
        ey = [0;1;0];
        ez = [0;0;1];
        kin.H = [ez ey ey ey -ez ey];
        kin.P = [0.089159*ez, 0.1358*ey, -0.1197*ey+0.425*ex, 0.3922*ex, 0.093*ey-0.0946*ez, zv, 0.0823*ey];
        kin.joint_type = zeros([6 1]);
    end

    function [P, S] = setup()
        S.Q = rand_angle([6,1]);
        [P.R, P.T] = fwdkin(hardcoded_IK_setups.ur5.get_kin(), S.Q);
    end
    
    function S = run(P)
        [S.Q, S.is_LS] = hardcoded_IK.ur5(P.R, P.T);
    end

    function S = run_mex(P)
        [S.Q, S.is_LS] = hardcoded_IK.ur5_mex(P.R, P.T);
    end

    function [e, e_R, e_T] = error(P,S)
        P.kin = hardcoded_IK_setups.ur5.get_kin();
        [e, e_R, e_T] = robot_IK_error(P, S);
    end
end
end