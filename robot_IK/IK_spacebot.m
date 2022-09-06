function qq = IK_spacebot(R, T, SEW_class, psi, GC, kin)
if isfield(kin, 'RT')
    R = R * kin.RT';
end

qq = NaN([7,1]);

p_W_EE_0 = kin.P(:,end);

d_S_E = norm(sum(kin.P(:,2:4), 2));
d_E_W = norm(sum(kin.P(:,5:7), 2));

% Find wrist position
    W = T - R * p_W_EE_0;

% Find shoulder position
    S = kin.P(:,1);

% Find elbow position
    % Use lengths of links to find point C
    % delta is the angle between SW and SE
    d_S_W = norm(W - S);
    cos_delta = (d_S_E^2 + d_S_W^2 - d_E_W^2) / (2 * d_S_E * d_S_W);

    C = S + (W - S) * d_S_E * cos_delta / d_S_W;

    k_sew = SEW_class.inv_kin(S, W, psi);

    % Find elbow position based on (x_c, y_c) and SEW angle
    d_C_E = d_S_E * sqrt(1-cos_delta^2);
    E = C + d_C_E * k_sew;

% Find q_1 and q_2 using subproblem 2
    h_1 = kin.H(:,1);
    h_2 = kin.H(:,2);
    p_S_E_0 = sum(kin.P(:,2:4), 2);
    p_S_E = E-S;

    [q1, q2] = subproblem2_linear(p_S_E,p_S_E_0, -h_1, h_2);

    if (q2(1) >= 0) == (GC(1) == 1)
        qq([1 2]) = [q1(1) q2(1)];
    else
        qq([1 2]) = [q1(2) q2(2)];
    end  

% Find q_3 and q_4 using subproblem 2
    h_3 = kin.H(:,3);
    h_4 = kin.H(:,4);
    p_E_W_0 = sum(kin.P(:,5:7), 2);
    p_E_W = W - E;
    
    R_2 = rot(h_1, qq(1)) * rot(h_2, qq(2));

    [q3, q4] = subproblem2_linear(R_2'*p_E_W,p_E_W_0, -h_3, h_4);

    if (q4(1) >= 0) == (GC(2) == 1)
        qq([3 4]) = [q3(1) q4(1)];
    else
        qq([3 4]) = [q3(2) q4(2)];
    end

% Find q_5 and q_6 using subproblem 2
    h_5 = kin.H(:,5);
    h_6 = kin.H(:,6);
    p_W_EE = T - W;
    p_W_EE_0 = kin.P(:,end);

    R_4 = R_2*rot(h_3, qq(3)) * rot(h_4, qq(4));
    [q5, q6] = subproblem2_linear(R_4'*p_W_EE, p_W_EE_0, -h_5, h_6);

    if (q6(1) >= 0) == (GC(3) == 1)
        qq([5 6]) = [q5(1) q6(1)];
    else
        qq([5 6]) = [q5(2) q6(2)];
    end

% Find q_7
    h_7 = kin.H(:,7);
    R_6 = R_4 * rot(h_5, qq(5)) * rot(h_6, qq(6));
    axang = rotm2axang(R_6' * R);
    qq(7) = axang(4) * axang(1:3)*h_7;

end