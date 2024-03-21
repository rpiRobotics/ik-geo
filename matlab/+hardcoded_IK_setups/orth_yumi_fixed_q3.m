classdef orth_yumi_fixed_q3
properties (Constant)
    % q3 = pi/6;
    q3 = sym(pi)/2;
end

methods (Static)
    function kin = get_kin()       
        zv = [0;0;0];
        ex = [1;0;0];
        ey = [0;1;0];
        ez = [0;0;1];
        
        kin.joint_type = zeros([1 7]);
        kin.H = [ez ey ez ey ex ey ex];
        kin.P = [zv 166*ez-30*ex 30*ex 251.5*ez+40.5*ex 40.5*ez 265*ex-27*ez 27*ez zv];
    end

    function [kin_partial, R_6T] = get_kin_partial()
        kin = hardcoded_IK_setups.orth_yumi_fixed_q3.get_kin();
        % Redefine R_6T = eye(3) in full kinematics parameters
        [kin_partial, R_6T] = fwdkin_partial(kin, hardcoded_IK_setups.orth_yumi_fixed_q3.q3, 3);
        kin_partial.H = double(kin_partial.H);
        kin_partial.P = double(kin_partial.P);
    end

    function [P, S] = setup()
        S.Q = rand_angle([7,1]);
        S.Q(3) = hardcoded_IK_setups.orth_yumi_fixed_q3.q3;
        S.Q_partial = S.Q([1:2 4:end]);
        [P.R, P.T] = fwdkin(hardcoded_IK_setups.orth_yumi_fixed_q3.get_kin(), S.Q);
    end
    
    function S = run(P)
        [S.Q, S.is_LS] = hardcoded_IK.orth_yumi_fixed_q3(P.R, P.T);
    end

    function S = run_mex(P)
        [S.Q, S.is_LS] = hardcoded_IK.orth_yumi_fixed_q3_mex(P.R, P.T);
    end

    function [e, e_R, e_T] = error(P,S)
        P.kin = hardcoded_IK_setups.orth_yumi_fixed_q3.get_kin();
        S.Q = [S.Q(1:2,:)
               hardcoded_IK_setups.orth_yumi_fixed_q3.q3*ones([1 width(S.Q)])
               S.Q(3:end,:)];
        [e, e_R, e_T] = robot_IK_error(P, S);
    end
end
end