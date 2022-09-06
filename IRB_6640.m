% Robot inverse kinematics example: ABB IRB-6640
% 6-DOF robot with spherical wrist and 2 parallel axes

% Set Up Inverse Kinematics Problem
% Define kinematic parameters
zv = [0;0;0];
ex = [1;0;0];
ey = [0;1;0];
ez = [0;0;1];

kin.H = [ez ey ey ex ey ex];
kin.P = [zv, 0.32*ex+0.78*ez, 1.075*ez, 1.1425*ex+0.2*ez, 0*ex, 0*ey, 0.2*ex];
kin.joint_type = zeros([6 1]);

% Pick a joint configuration find the associated end effector pose
q_true = rand([6 1])*2*pi - pi
[R_0T, p_0T] = fwdkin(kin, q_true)

%% Solve Inverse Kinematics
Q = [];
% Use subproblem 4 to find up to two solutions for q1

t1 = subproblem4_linear( ...
    kin.H(:,2), ...
    p_0T-R_0T*kin.P(:,7)-kin.P(:,1), ...
    -kin.H(:,1), ...
    kin.H(:,2)'*(kin.P(:,2)+kin.P(:,3)+kin.P(:,4)));

% Use subproblem 3 to find up to two solutions for q3

for q1 = t1
    t3 = subproblem3_linear( ...
        -kin.P(:,4), ...
        kin.P(:,3), ...
        kin.H(:,3), ...
        norm( ...
            rot(-kin.H(:,1), q1)*(-p_0T+R_0T*kin.P(:,7)+kin.P(:,1)) + kin.P(:,2) ...
            ) ...
        );
% Solve for q2 using subproblem 1

    for q3 = t3
        q2 = subproblem1_linear( ...
            -kin.P(:,3) - rot(kin.H(:,3), q3)*kin.P(:,4), ...
            rot(-kin.H(:,1), q1)*(-p_0T+R_0T*kin.P(:,7)+kin.P(:,1)) + kin.P(:,2), ...
            kin.H(:,2));

        R_36 = rot(-kin.H(:,3), q3)*...
               rot(-kin.H(:,2), q2)*...
               rot(-kin.H(:,1), q1)*R_0T; % R_T6 = 0
% Solve for q5 using subproblem 4

        t5 = subproblem4_linear( ...
            kin.H(:,4), ...
            kin.H(:,6), ...
            kin.H(:,5), ...
            kin.H(:,4)'*R_36*kin.H(:,6));
        
% Solve for q4 using subproblem 1

        for q5 = t5  
            q4 = subproblem1_linear( ...
                rot(kin.H(:,5), q5)*kin.H(:,6), ...
                R_36*kin.H(:,6), ...
                kin.H(:,4));
% Solve for q6 using subproblem 1

            q6 = subproblem1_linear( ...
                rot(-kin.H(:,5), q5) * kin.H(:,4), ...
                R_36'*kin.H(:,4), ...
                -kin.H(:,6));
            q_i = [q1 q2 q3 q4 q5 q6]';
            Q = [Q q_i];
        end
    end
end


Q

%% Check that all the solutions are valid
diffs = [];
for q_i = Q
    [R_0T_i, p_0T_i] = fwdkin(kin, q_i);
    diffs = [diffs; [norm(R_0T - R_0T_i) norm(p_0T-p_0T_i)]];
end
diffs
