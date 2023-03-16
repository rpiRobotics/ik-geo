function khat = hat(k)
    % HAT
    %   khat = hat(k)
    %
    %   matrix cross-product for a 3 x 3 vector
    %
    %           [  0 -k3  k2]
    %   khat =  [ k3   0 -k1]
    %           [-k2  k1   0]
    %
    khat=[0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];
end
  