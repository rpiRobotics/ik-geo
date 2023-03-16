function R=rot(k,theta)
    % ROT
    %   R = rot(k, theta)
    %   
    %   Creates a 3 x 3 rotation matrix basd on the Euler-Rodrigues formula.
    %   R = I + sin(theta)*hat(k) + (1 - cos(theta))*hat(k)^2
    %
    %   see also HAT
    k = k / norm(k);
    R = eye(3,3)+sin(theta)*hat(k)+(1-cos(theta))*hat(k)*hat(k);
end  