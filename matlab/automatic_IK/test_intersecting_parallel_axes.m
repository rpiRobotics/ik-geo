% setup = hardcoded_IK_setups.ur5;
% setup = hardcoded_IK_setups.IRB_6640;
% setup = hardcoded_IK_setups.two_parallel_bot;
% setup = hardcoded_IK_setups.three_parallel_bot;
% setup = hardcoded_IK_setups.KUKA_R800_fixed_q3;
% setup = hardcoded_IK_setups.yumi_fixed_q3;
setup = hardcoded_IK_setups.RRC_fixed_q6;
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
        intersection = sum(kin.P(:,1:j),2) + h_j*ab(2);
        if norm(intersection - prev_intersection) < 1e-3
            fprintf("Joints %d %d %d spherical\n", i-1, i, j);
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
    end

end
