function C = uniquetol_manual(A)
% subproblem.uniquetol_manual Reimplementation of uniquetol
%   Reimplementation of uniquetol since it's not supported by MATLAB coder

if isempty(A)
    C = A;
    return
end

A = sort(A);
C = A(1);
last_unique = A(1);
for i = 1:numel(A)
    if ~within_tol(A(i), last_unique)
        C = [C A(i)];
        last_unique = A(i);
    end
end
end

function is_equal = within_tol(a,b)
    is_equal = abs(a - b) < 1e-6; 
end