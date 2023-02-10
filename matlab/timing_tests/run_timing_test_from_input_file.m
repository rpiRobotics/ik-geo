setup = subproblem_setups.sp_3;

S_test_list = repmat(S_list(1), 0, length(S_list));
tic
for i = 1:length(P_list)
    S_test_list(i) = setup.run(P_list(i));
end
t_total = toc
t_total/N*1e6