function [theta1, theta2, is_LS] = sp_2(p1, p2, k1, k2)
% subproblem.sp_2  Subproblem 2: Two circles
%   [theta1, theta2] = subproblem.sp_2(p1, p2, k1, k2) finds theta1, theta2
%   such that
%       rot(k1, theta1)*p1 = rot(k2, theta2)*p2
%   If there's no solution, minimize the least-squares residual
%           || rot(k1, theta1)*p1 - rot(k2, theta2)*p2 ||
%
%   If the problem is well-posed, there may be 1 or 2 solutions
%   (These may be exact or least-squares solutions)
%   theta1 and theta2 are column vectors of the solutions
%
%   [theta1, theta2, is_LS] = subproblem.sp_2(p1, p2, k1, k2) also produces
%   a flag is_LS, which is true if (theta1, theta2) is a least-squares
%   solution
%
%   The problem is ill-posed if (p1, k1), (p2, k2), or (k1, k2) are
%   parallel
%
%   Inputs:
%       p1: 3x1 vector
%       p2: 3x1 vector
%       k1: 3x1 vector with norm(k1) = 1
%       k2: 3x1 vector with norm(k2) = 1
%   Outputs:
%       theta1: 1xN angle vector (in radians)
%       theta2: 1xN angle vector (in radians)
%           (N is the number of solutions)
%       is_LS: 1x1 boolean

% Rescale for least-squares case
p1_nrm = p1/norm(p1);
p2_nrm = p2/norm(p2);

[theta1, t1_is_LS] = subproblem.sp_4(k2, p1_nrm, k1, dot(k2,p2_nrm));
[theta2, t2_is_LS] = subproblem.sp_4(k1, p2_nrm, k2, dot(k1,p1_nrm));

% Make sure solutions correspond by flipping theta2
% Also make sure in the edge case that one angle has one solution and the
% other angle has two solutions that we duplicate the single solution
if numel(theta1)>1 || numel(theta2)>1
    theta1 = [theta1(1) theta1(end)];
    theta2 = [theta2(end) theta2(1)];
end

if nargout > 2
    is_LS = abs(norm(p1) - norm(p2)) > 1e-8 || t1_is_LS || t2_is_LS;
end    

end