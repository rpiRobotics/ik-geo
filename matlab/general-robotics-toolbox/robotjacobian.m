function J = robotjacobian(kin, theta)
    % ROBOTJACOBIAN
    %
    % J = robotjacobian(kin, theta)
    %
    % purpose: calculate jacobian for serial chain robot relating joint
    %           velocity to end effector spatial velocity
    %           (compatible with matlab-rigid-body-viz toolbox)
    %
    % input:
    %   kin: struct with form:
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
    %   theta: n-vector of actuator position (expecting radians for
    %                                           rotational states)
    % 
    % output:
    %       J = [jw_1 jw_2 ... jw_n] (6 x n) jacobian matrix 
    %           [jv_1 jv_2 ... jv_n]        * in base frame
    %           top three rows reflect the instantaneous change in 
    %           end effector angular velocity while the bottom three rows
    %           are the instantaneous change in linear velocity both with
    %           respect to actuator velocity
    %
    
    p = kin.P(:,1);
    R = eye(3);
    
    J = zeros(6,numel(kin.joint_type));

    hi = zeros(3,numel(kin.joint_type));
    pOi = zeros(3,numel(kin.joint_type)+1);
    pOi(:,1) = p;

    % Compute and store forward kinematics
    for i = 1:numel(kin.joint_type)
        if (kin.joint_type(i) == 0 || ...       % rotational actuators
                    kin.joint_type(i) == 2)        
            R = R*rot(kin.H(:,i),theta(i));
        elseif (kin.joint_type(i) == 1 || ...   % translational actuators
                    kin.joint_type(i) == 3) 
            p = p + R*kin.H(:,i)*theta(i);
        end
        p = p + R*kin.P(:,i+1);
        pOi(:,i+1) = p;
        hi(:,i) = R*kin.H(:,i);
    end

    pOT = pOi(:,end);
    % Compute Jacobian
    i = 1;
    j = 1;
    while i <= numel(kin.joint_type)
        if kin.joint_type(i) == 0               % revolute actuators
            J(:,j) = [hi(:,i); hat(hi(:,i))*(pOT - pOi(:,i))];
        elseif kin.joint_type(i) == 1           % prismatic actuators  
            J(:,j) = [0;0;0; hi(:,i)];
        elseif kin.joint_type(i) == 3           % nonholonomic mobile
            % This is a special case, and is dependent on the next
            % two 'joints' following the format for the unicycle model.  
            % Should consider new format in future release.
            % Linear Velocity
            J(:,j) = [0;0;0;rot(hi(:,i+2),theta(i+2))*hi(:,i)];
            % Angular Velocity
            J(:,j+1) = [hi(:,i+2);hat(hi(:,i+2))*(pOT - pOi(:,i+2))];
            J = J(:,1:end-1);
            i = i + 2;
            j = j + 1;
        end
        i = i + 1;
        j = j + 1;
    end
end
   
   
    
    
