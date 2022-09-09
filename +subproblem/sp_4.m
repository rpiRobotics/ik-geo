function [theta, is_LS] = sp_4(h, p, k, d)
% h'* R(k,theta)*p = d

A_11 = cross(k,p);
A_1 = [A_11 -cross(k,A_11)];
A = h'*A_1;

b = d - h'*k*(k'*p);

norm_A_2 = dot(A,A);

x_ls = A_1'*(h*b);

if norm(norm_A_2) > b^2
    xi = sqrt(norm_A_2-b^2);
    A_perp_tilde = [A(2); -A(1)];

    
    sc_1 = x_ls + xi*A_perp_tilde;
    sc_2 = x_ls - xi*A_perp_tilde;

    theta = [atan2(sc_1(1), sc_1(2)) atan2(sc_2(1), sc_2(2))];
    if nargout > 1
        is_LS = false;
    end
else
    theta = atan2(x_ls(1), x_ls(2));
    if nargout > 1
        is_LS = true;
    end
end

end