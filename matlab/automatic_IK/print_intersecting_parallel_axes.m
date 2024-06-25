function print_intersecting_parallel_axes(is_intersecting, is_intersecting_nonconsecutive, is_parallel, is_spherical)
% Print intersecting / parallel axes to console

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
end