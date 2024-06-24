% setup = hardcoded_IK_setups.ur5;
% setup = hardcoded_IK_setups.IRB_6640;
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
setup = IK_setups.IK_spherical_2_intersecting;
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

[is_intersecting, is_intersecting_nonconsecutive, is_parallel, is_spherical] = detect_intersecting_parallel_axes(kin);

clc
fprintf("Intersecting Joints: ");
for i=find(is_intersecting)
    fprintf("(%d, %d) ", i, i+1);
end
fprintf("\n");

fprintf("Intersecting Nonconsecutive Joints: ");
for i=find(is_intersecting_nonconsecutive)
    fprintf("(%d, %d) ", i, i+2);
end
fprintf("\n");

fprintf("Parallel Joints: ");
for i=find(is_parallel)
    fprintf("(%d, %d) ", i, i+1);
end
fprintf("\n");

fprintf("Spherical Joints: ");
for i=find(is_spherical)
    fprintf("(%d, %d, %d) ", i, i+1, i+2);
end
fprintf("\n");

% Recommend solver based on intersecting and parallel axes
fprintf("\nCompatible solvers:\n")
if N == 6
    rec_solver_6_DOF(is_intersecting, is_intersecting_nonconsecutive, is_parallel, is_spherical)
elseif N == 7
    rec_solver_7_DOF(is_intersecting, is_intersecting_nonconsecutive, is_parallel, is_spherical)
else
    fprintf("Only 6- and 7-DOF solver recommendations supported\n")
end