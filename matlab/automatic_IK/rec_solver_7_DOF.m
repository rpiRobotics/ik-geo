function rec_solver_7_DOF(is_intersecting, is_intersecting_nonconsecutive, is_parallel, is_spherical)

if is_intersecting(1) && is_intersecting(3) && is_spherical(5)
    fprintf("IK_2R_2R_3R (Closed-Form Quadratic)\n");
end

if is_intersecting(1) && is_spherical(3) && is_intersecting(6)
    fprintf("IK_2R_3R_2R (Closed-Form Quadratic)\n");
end

if is_intersecting(1) && is_parallel(3) && is_parallel(4) && is_intersecting(6)
    fprintf("IK_2R_3Rp_2R (Closed-Form Quadratic)\n");
end

if is_spherical(1) && is_intersecting(5)
    fprintf("IK_3R_R_2R_R (1D Search)\n");
end

if is_spherical(1) && is_spherical(5)
    fprintf("IK_3R_R_3R (Closed-Form Quadratic)\n");
end

if is_intersecting(2) && is_intersecting(4) && is_spherical(6)
    fprintf("IK_R_2R_2R_2R (1D Search)\n");
end

if is_intersecting(2) && is_spherical(5)
    fprintf("IK_R_2R_R_3R_SJ2 (1D Search)\n");
end

if is_spherical(3) && is_intersecting(6)
    fprintf("IK_R_R_3R_2R (Closed-Form Quartic)\n");
end

if is_parallel(3) && is_parallel(4) && is_intersecting(6)
    fprintf("IK_R_R_3Rp_2R (Closed-Form Quartic)\n");
end

% Always possible
    fprintf("IK_gen_7_dof (2D Search)\n");
end