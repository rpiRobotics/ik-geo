function [theta, is_LS] = sp_1(p1, p2, k)
% subproblem.sp_1  Subproblem 1: Circle and point
%   theta = subproblem.sp_1(p1, p2, k) finds theta such that
%           rot(k, theta)*p1 = p2
%   If there's no solution, theta minimizes the least-squares residual
%           || rot(k, theta)*p1 - p2 ||
%
%   [theta, is_LS] = subproblem.sp_1(p1, p2, k) also produces a flag is_LS
%   is_LS is true if theta is a least-squares solution
%
%   The problem is ill-posed if p1 or p2 is parallel to k
%
%   Inputs:
%       p1: 3x1 vector
%       p2: 3x1 vector
%       k:  3x1 vector with norm(k) = 1
%   Outputs:
%       theta: 1x1 angle (in radians)
%       is_LS: 1x1 boolean

KxP = cross(k, p1);
A = [KxP -cross(k,KxP)];

x = A'*p2;

theta = atan2(x(1),x(2));

if nargout > 1
    % Least squares solution if ||p_1|| != ||p_2|| or k'p_1 != k'p_2
    is_LS = abs(norm(p1) - norm(p2)) > 1e-8 || abs(dot(k,p1) - dot(k,p2)) > 1e-8;
end
end