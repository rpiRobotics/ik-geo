function [theta, is_LS] = sp_3(p1, p2, k, d)
% subproblem.sp_3  Subproblem 3: Circle and sphere
%   theta = subproblem.sp_3(p1, p2, k, d) finds theta such that
%       || rot(k, theta)*p1 - p2 || = d
%   If there's no solution, minimize the least-squares residual
%       | || rot(k, theta)*p1 - p2 || - d |
%
%   If the problem is well-posed, there may be 1 or 2 exact solutions, or 1
%   least-squares solution
%   theta1 and theta2 are column vectors of the solutions
%
%   [theta, is_LS] = subproblem.sp_3(p1, p2, k, d) also produces
%   a flag is_LS, which is true if theta is a least-squares solution
%
%   The problem is ill-posed if (p1, k) or (p2, k) are parallel
%
%   Inputs:
%       p1: 3x1 vector
%       p2: 3x1 vector
%       k:  3x1 vector with norm(k) = 1
%       d:  1x1
%   Outputs:
%       theta: 1xN angle (in radians)
%           (N is the number of solutions)
%       is_LS: 1x1 boolean

[theta, is_LS] = subproblem.sp_4(p2, p1, k, 1/2 * (dot(p1,p1)+dot(p2,p2)-d^2));
end