function v = rand_vec(N)
    if nargin < 1
        N = 1;
    end
    v = rand([3 N])*2-1;
end
