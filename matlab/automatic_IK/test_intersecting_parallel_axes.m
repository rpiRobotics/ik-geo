% setup = hardcoded_IK_setups.ur5;
setup = hardcoded_IK_setups.IRB_6640;
% setup = hardcoded_IK_setups.two_parallel_bot;
% setup = hardcoded_IK_setups.three_parallel_bot;
% setup = hardcoded_IK_setups.KUKA_R800_fixed_q3;
% setup = hardcoded_IK_setups.yumi_fixed_q3;
% setup = hardcoded_IK_setups.RRC_fixed_q6;
% setup = hardcoded_IK_setups.spherical_bot;

% setup = IK_setups.IK_gen_6_dof;
% setup = IK_setups.IK_2_intersecting;
% setup = IK_setups.IK_2_parallel;
% setup = IK_setups.IK_spherical;
% setup = IK_setups.IK_spherical_2_intersecting;
% setup = IK_setups.IK_spherical_2_parallel;
% setup = IK_setups.IK_3_parallel;
% setup = IK_setups.IK_3_parallel_2_intersecting;

if ismethod(setup,"get_kin_partial") % hardcoded with fixed q_i
    kin = setup.get_kin_partial;
elseif ismethod(setup,"get_kin") % hardcoded
    kin = setup.get_kin;
else % randomly generated
    P = setup.setup;
    kin = P.kin;
end

clc

N = length(kin.joint_type);

% Make sure kin.P and kin.H are the right sizes
assert(width(kin.P) == N + 1);
assert(width(kin.H) == N);

% TODO: Also include how close the intersction is, or adjustable threshold
% TODO: is_intersecting_nonconsecutive

is_intersecting   = false([1, N-1]); % joints i, i+1 intersect
is_intersecting_nonconsecutive = false([1, N-2]); % joints i, i+2 intersect
is_parallel       = false([1, N-1]); % joints i, i+1 parallel
is_spherical      = false([1, N-2]); % joints i, i+1, i+2 spherical
% (Three parallel joints can be implied from is_parallel)

% Test for 2R and 3R joints
prev_intersection = NaN(3,1);
for i = 1:N-1
    j = i+1;

    p_ij = kin.P(:,j);
    h_i = kin.H(:,i);
    h_j = kin.H(:,j);
    
    ab = pinv([h_i h_j]) * p_ij;
    
    dist_ij = norm(p_ij - [h_i h_j] * ab);
    if dist_ij < 1e-3
        fprintf("Joints %d and %d intersect\n", i, j);
        is_intersecting(i) = true;
        intersection = sum(kin.P(:,1:j),2) + h_j*ab(2);
        if norm(intersection - prev_intersection) < 1e-3
            fprintf("Joints %d %d %d spherical\n", i-1, i, j);
            is_spherical(i-1) = true;
        end
        prev_intersection = intersection;
    else
        prev_intersection = NaN(3,1);
    end
end

fprintf("\n");

% Test for 2R|| (and implicitly 3R||) joints
for i = 1:N-1
    j = i+1;
    h_i = kin.H(:,i);
    h_j = kin.H(:,j);

    if abs(dot(h_i, h_j)) > 1-1e-3
        fprintf("Joints %d and %d parallel\n", i, j);
        is_parallel(i) = true;
    end

end

% Recomend solver based on intersecting and parallel axes

fprintf("\nCompatible solvers:\n")
if N == 6
    rec_solver_6_DOF(is_intersecting, is_parallel, is_spherical)
elseif N == 7
    rec_solver_7_DOF(is_intersecting, is_parallel, is_spherical)
else
    fprintf("Only 6- and 7-DOF solver reccomendations supported\n")
end

function rec_solver_6_DOF(is_intersecting, is_parallel, is_spherical)
    if is_spherical(4) && is_intersecting(1)
        fprintf("IK_spherical_2_intersecting (Closed-Form Quadratic)\n")
    end

    if is_spherical(4) && is_parallel(2)
        fprintf("IK_spherical_2_parallel (Closed-Form Quadratic)\n")
    end
    
    if is_parallel(2) && is_parallel(3) && is_intersecting(5)
        fprintf("IK_3_parallel_2_intersecting (Closed Form Quadratic)\n")
    end
    
    if is_spherical(4)
        fprintf("IK_spherical (Closed-Form Quartic)\n")
    end
    
    if is_parallel(2) && is_parallel(3)
        fprintf("IK_3_parallel (Closed-Form Quartic)\n")
    end
    
    if is_intersecting(5)
        fprintf("IK_2_intersecting (1D Search)\n")
    end

    if is_parallel(2)
        fprintf("IK_2_parallel (1D Search)\n")
    end

    % TODO
    %fprintf("IK_4_6_intersecting (1D Search)\n")
    
    % Always possible
        fprintf("IK_gen_6_dof (2D Search)\n")
end

function rec_solver_7_DOF(is_intersecting, is_parallel, is_spherical)
    % TODO
end