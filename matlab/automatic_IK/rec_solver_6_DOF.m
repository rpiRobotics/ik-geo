function rec_solver_6_DOF(is_intersecting, is_intersecting_nonconsecutive, is_parallel, is_spherical)
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

    if is_intersecting_nonconsecutive(4)
        fprintf("IK_4_6_intersecting (1D Search)\n")
    end

    % Always possible
        fprintf("IK_gen_6_dof (2D Search)\n")
end