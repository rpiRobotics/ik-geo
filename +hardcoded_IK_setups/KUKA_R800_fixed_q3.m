classdef KUKA_R800_fixed_q3
properties (Constant)
    q3 = pi/6;
end

methods (Static)
    function kin = get_kin()
        zv = [0;0;0];
        ex = [1;0;0];
        ey = [0;1;0];
        ez = [0;0;1];
        
        kin.H = [ez ey ez -ey ez ey ez];
        kin.P = [(0.15+0.19)*ez zv 0.21*ez 0.19*ez (0.21+0.19)*ez zv zv (0.081+0.045)*ez];
        kin.joint_type = zeros([7 1]);
    end

    function [kin_partial, R_6T] = get_kin_partial()
        kin = hardcoded_IK_setups.KUKA_R800_fixed_q3.get_kin();
        [kin_partial, R_6T] = fwdkin_partial(kin,hardcoded_IK_setups.KUKA_R800_fixed_q3.q3, 3);
    end

    function read_from_robotics_toolbox
        % Print out kinematics parameters from MATLAB's robotics toolbox
        robot = importrobot("iiwa7.urdf");
        for i = 1:length(robot.Bodies)
            disp(robot.Bodies{i}.Name)
            disp(robot.Bodies{i}.Joint.JointAxis)
            disp(robot.Bodies{i}.Joint.JointToParentTransform)
        end
    end

    function verify_kin_params
        % Compare kinematics parameters to MATLAB's robotics toolbox
        
        robot = importrobot("iiwa7.urdf");

        % homeConfiguration(robot)
        q_mrt = randomConfiguration(robot)
        q = [q_mrt.JointPosition]';

        getTransform(robot, q_mrt, "iiwa_link_ee_kuka")

        kin = hardcoded_IK_setups.KUKA_R800_fixed_q3.get_kin();
        [R, T] = fwdkin(kin, q)
    end

    function [P, S] = setup()
        S.Q = rand_angle([7,1]);
        S.Q(3) = hardcoded_IK_setups.KUKA_R800_fixed_q3.q3;
        S.Q_partial = S.Q([1:2 4:end]);
        [P.R, P.T] = fwdkin(hardcoded_IK_setups.KUKA_R800_fixed_q3.get_kin(), S.Q);
    end
    
    function S = run(P)
        [S.Q, S.is_LS] = hardcoded_IK.KUKA_R800_fixed_q3(P.R, P.T);
    end

    function S = run_mex(P)
        [S.Q, S.is_LS] = hardcoded_IK.KUKA_R800_fixed_q3_mex(P.R, P.T);
    end

    function [e, e_R, e_T] = error(P,S)
        P.kin = hardcoded_IK_setups.KUKA_R800_fixed_q3.get_kin();
        if height(S.Q) == 6 % Partial Q
        S.Q = [S.Q(1:2,:)
               hardcoded_IK_setups.KUKA_R800_fixed_q3.q3*ones([1 width(S.Q)])
               S.Q(3:end,:)];
        end
        [e, e_R, e_T] = robot_IK_error(P, S);
    end
end
end