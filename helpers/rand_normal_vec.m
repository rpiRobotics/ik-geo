function v = rand_normal_vec()
    v = rand_vec();
    v = v/norm(v);
end