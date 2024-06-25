function kin = dh_to_kin(alpha_vec, a_vec, d_vec)
% Convert a robot define in Denavit-Hartenberg convention to Product of Exponentials

    N = length(alpha_vec);
    kin.joint_type = zeros(1,N);
    kin.H = sym(NaN(3,N));
    kin.P = sym(NaN(3,N+1));

    kin.P(:,1) = [0;0;0];
    kin.H(:,1) = [0;0;1];
    R = eye(3);

    for i = 1:N
        % Translate d_i along z_{i-1}   
        % Move a along x_{i-1}
        kin.P(:,i+1) = d_vec(i)*R(:,3) + a_vec(i)*R(:,1);

        % Rotate by alpha along x_{i-1}
        R = rot(R(:,1), alpha_vec(i))*R;

        if i == N
            kin.RT = rot(R(:,1), alpha_vec(i));
        else
            kin.H(:,i+1) = R(:,3); % Joint axis is z axis
        end
    end

end