function [R, p] = fwdkin(kin, theta)
    % FWDKIN
    %
    % [R, p] = fwdkin(kin, theta)
    %
    % purpose: general forward kinematics for serial chain robot
    %           (compatible with matlab-rigid-body-viz toolbox)
    % 
    % input:
    %   kin: kinematics struct with form:
    %       root
    %           -> H          : [h_1 h_2 ... h_n]  
    %                           (3 x n) actuator axes
    %           -> P          : [p_{O,1} p_{1,2} ... p_{n,T}] 
    %                           (3 x n + 1) actuator displacements
    %           -> joint_type : n-vector of joint types
    %                           0 - rotational
    %                           1 - prismatic
    %                           2 - mobile orientation
    %                           3 - mobile translation
    %   
    %   theta: n-vector of actuator state (expecting radians for
    %                                           rotational states)
    % 
    % output:
    %       R = R_{O,T}: 3 x 3 matrix for orientation of end effector with
    %                   respect to base frame
    %       p = p_{O,T}: 3 x 1 vector giving position of end effector with
    %                   respect to base frame in base frame coordinates
    %
        
    p = kin.P(:,1);
    R = eye(3);

    for i = 1:numel(kin.joint_type)
        if (kin.joint_type(i) == 0 || ...       % rotational actuators
                    kin.joint_type(i) == 2)        
            R = R*rot(kin.H(:,i),theta(i));
        elseif (kin.joint_type(i) == 1 || ...   % translational actuators
                    kin.joint_type(i) == 3)    
            p = p + R*kin.H(:,i)*theta(i);
        end
        p = p + R*kin.P(:,i+1);
    end
end