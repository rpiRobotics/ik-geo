function v = rand_normal_vec(N)
    if nargin < 1
        N = 1;
    end
    v = rand_vec(N);
    for i = 1:N
        v(:,i) = v(:,i) / norm(v(:,i));
    end
end