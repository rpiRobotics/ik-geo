function rts = quartic_roots(poly)
% subproblem.quartic_roots Numeric Quartic Equation Solution
%   Solve for zeros of the f(x) = Ax^4 + Bx^3 + Cx^2 + Dx + E
%   Input is the vector [A B C D E]
%   https://en.wikipedia.org/wiki/Quartic_equation#Summary_of_Ferrari's_method

poly = complex(poly); % Make sure sqrt() can give complex results
A = poly(1); B = poly(2); C = poly(3); D = poly(4); E = poly(5);

if norm(A) < 1e-12 % At most degree 3 (cubic)
    if norm(B) < 1e-12 % At most degree 2 (quadratic)
        if norm(C) < 1e-12 % At most degree 1 (linear)
            % Dx + E = 0
            rts = -E/D;
            return
        end

        % Solve the quadratic
        % Cx^2 + Dx + E = 0
        rts = [-D + sqrt(D^2 -4*C*E)
               -D - sqrt(D^2 -4*C*E)]/(2*C);
        return
    end

    % Solve the cubic
    % Bx^3 + Cx^2 + Dx + E = 0
    rts = roots(poly); % TODO closed-form
    return
end

alpha = -3/8 * B^2 / A^2 + C/A;
beta = B^3/(8*A^3) - B*C/(2*A^2) + D/A;
gamma = -3*B^4/(256*A^4) + C*B^2/(16*A^3) - B*D/(4*A^2) + E/A;

% if beta == 0
if norm(beta) < 1e-12
    rts = -B/(4*A) + [ + sqrt((-alpha + sqrt(alpha^2-4*gamma))/2)
                       - sqrt((-alpha + sqrt(alpha^2-4*gamma))/2)
                       + sqrt((-alpha - sqrt(alpha^2-4*gamma))/2)
                       - sqrt((-alpha - sqrt(alpha^2-4*gamma))/2)];
    return
end

P = -alpha^2/12 - gamma;
Q = -alpha^3/108 + alpha*gamma/3 - beta^2/8;
R = -Q/2 + sqrt(Q^2/4 + P^3/27);
U = R^(1/3);

%if U == 0
if norm(U) < 1e-12
    y = -5/6 * alpha - Q^(1/3);
else
    y = -5/6 * alpha + U - P/(3*U);
end

W = sqrt(alpha + 2*y);

rts = -B/(4*A) + [+W + sqrt(-(3*alpha+2*y+2*beta/W))
                  +W - sqrt(-(3*alpha+2*y+2*beta/W))
                  -W + sqrt(-(3*alpha+2*y-2*beta/W))
                  -W - sqrt(-(3*alpha+2*y-2*beta/W))] /2;
end