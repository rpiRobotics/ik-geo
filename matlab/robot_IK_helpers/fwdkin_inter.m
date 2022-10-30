function [R, p, p_inter] = fwdkin_inter(kin, theta, inter)
% Like fwdkin, but also provides intermediate positions
    p = kin.P(:,1);
    R = eye(3);
    p_inter = NaN([3 length(inter)]);
    i_inter = 1;

    for i = 1:numel(kin.joint_type)
        if any(i == inter)
            p_inter(:,i_inter) = p;
            i_inter = i_inter +1;
        end

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