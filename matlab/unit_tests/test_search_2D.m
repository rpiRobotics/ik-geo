
[x1_vec, x2_vec, soln_num_vec] = search_2D(@example_error, -pi, pi, -pi, pi, 100, true);

[x1_vec x2_vec soln_num_vec]


for i = 1:length(x1_vec)
    v = example_error(x1_vec(i), x2_vec(i));
    v(soln_num_vec(i))
end
%%
example_error(1,2)

function v = example_error(x1, x2)
    v = [cos(x1)^4 + x2^2
         x1^2 + sin(x2-0.5)^2];
end