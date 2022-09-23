function v = rand_perp_normal_vec(v_ref)
    v = rand_vec();
    v = cross(v, v_ref);
    v = v / norm(v);
end