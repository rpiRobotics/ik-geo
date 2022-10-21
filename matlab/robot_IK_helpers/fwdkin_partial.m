function [kin_new, R_6T_new] = fwdkin_partial(kin, q_n, n, R_6T)
% Returns a new kinematics parameters struct kin_new
% based on fixing the angle q_k for just one joint n
% Also need to adjust R_6T, which fwdkin() doesn't use at the moment
% Indicies after n get decreased by one in kin_new
% (Only works revolute joint)

if nargin < 4
    R_6T = eye(3);
end

R_n = rot(kin.H(:,n), q_n);

kin_new.joint_type = kin.joint_type(1:end-1); 

kin_new.H = [kin.H(:,1:n-1) R_n*kin.H(:,n+1:end)];
kin_new.P = [kin.P(:,1:n-1) kin.P(:,n)+R_n*kin.P(:,n+1)  R_n*kin.P(:,n+2:end)];

R_6T_new = R_n * R_6T;
end