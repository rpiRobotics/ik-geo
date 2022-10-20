classdef yumi_fixed_q3
properties (Constant)
    q3 = pi/6;
end

methods (Static)
    function kin = get_kin()       
        kin.joint_type = zeros([7 1]);
        kin.P = [0.0536    0.0642    0.1578    0.0880    0.1270    0.0354    0.0385    0.0040
                 0.0725    0.0527    0.0406    0.0011   -0.0877   -0.0712   -0.0087   -0.0043
                 0.4149    0.0632    0.0650    0.0143   -0.0700   -0.0670   -0.0030   -0.0038];

        kin.H=  [0.8138    0.1048    0.8138    0.1048    0.5716    0.1048    0.5716
                 0.3420    0.7088    0.3420    0.7088   -0.6170    0.7088   -0.6170
                 0.4698   -0.6976    0.4698   -0.6976   -0.5410   -0.6976   -0.5410];
        kin.H = kin.H ./ vecnorm(kin.H);

        
%         kin.R_6T =  [-0.8138    0.1048    0.5716
%                      -0.3420    0.7088   -0.6170
%                      -0.4698   -0.6976   -0.5410];
%         [U,~,V] = svd(kin.R_6T);
%         kin.R_6T = U*V';
    end

    function [kin_partial, R_6T] = get_kin_partial()
        kin = hardcoded_IK_setups.yumi_fixed_q3.get_kin();
        % Redefine R_6T = eye(3) in full kinematics parameters
        [kin_partial, R_6T] = fwdkin_partial(kin, hardcoded_IK_setups.yumi_fixed_q3.q3, 3);
    end

    function read_from_robotics_toolbox
        % Print out kinematics parameters from MATLAB's robotics toolbox
        robot = importrobot("abbYuMi.urdf");
%         for i = 1:length(robot.Bodies)
%             disp(robot.Bodies{i}.Name)
%             disp(robot.Bodies{i}.Joint.JointAxis)
%             disp(robot.Bodies{i}.Joint.JointToParentTransform)
%         end
         q_mrt = homeConfiguration(robot);
         T_01 = getTransform(robot, q_mrt, "yumi_link_1_l");
         T_02 = getTransform(robot, q_mrt, "yumi_link_2_l");
         T_03 = getTransform(robot, q_mrt, "yumi_link_3_l");
         T_04 = getTransform(robot, q_mrt, "yumi_link_4_l");
         T_05 = getTransform(robot, q_mrt, "yumi_link_5_l");
         T_06 = getTransform(robot, q_mrt, "yumi_link_6_l");
         T_07 = getTransform(robot, q_mrt, "yumi_link_7_l");
         T_0T = getTransform(robot, q_mrt, "gripper_l_base");

         P = [T_01(1:3,end)               T_02(1:3,end)-T_01(1:3,end) ...
              T_03(1:3,end)-T_02(1:3,end) T_04(1:3,end)-T_03(1:3,end) ...
              T_05(1:3,end)-T_04(1:3,end) T_06(1:3,end)-T_05(1:3,end) ...
              T_07(1:3,end)-T_06(1:3,end) T_0T(1:3,end)-T_07(1:3,end)]
         H = [T_01(1:3,3) T_02(1:3,3) T_03(1:3,3) T_04(1:3,3) T_05(1:3,3) T_06(1:3,3) T_07(1:3,3)]
         R_6T = T_0T(1:3, 1:3)
    end

    function verify_kin_params
        % Compare kinematics parameters to MATLAB's robotics toolbox
        
        robot = importrobot("abbYuMi.urdf");

        q_mrt = randomConfiguration(robot);
        %q_mrt = homeConfiguration(robot);
        q = [q_mrt.JointPosition]';
        %q = q([1 2 4 5 6 7 3]); 
        q = q(1:7); % GRT seems to have a bug here, as naming is wrong

        getTransform(robot, q_mrt, "gripper_l_base")

        kin = hardcoded_IK_setups.yumi_fixed_q3.get_kin();
        [R, T] = fwdkin(kin, q);
        disp([R*kin.R_6T T])
    end

    function [P, S] = setup()
        S.Q = rand_angle([7,1]);
        S.Q(3) = hardcoded_IK_setups.yumi_fixed_q3.q3;
        S.Q_partial = S.Q([1:2 4:end]);
        [P.R, P.T] = fwdkin(hardcoded_IK_setups.yumi_fixed_q3.get_kin(), S.Q);
    end
    
    function S = run(P)
        [S.Q, S.is_LS] = hardcoded_IK.yumi_fixed_q3(P.R, P.T);
    end

    function S = run_mex(P)
        [S.Q, S.is_LS] = hardcoded_IK.yumi_fixed_q3_mex(P.R, P.T);
    end

    function [e, e_R, e_T] = error(P,S)
        P.kin = hardcoded_IK_setups.yumi_fixed_q3.get_kin();
        S.Q = [S.Q(1:2,:)
               hardcoded_IK_setups.yumi_fixed_q3.q3*ones([1 width(S.Q)])
               S.Q(3:end,:)];
        [e, e_R, e_T] = robot_IK_error(P, S);
    end
end
end