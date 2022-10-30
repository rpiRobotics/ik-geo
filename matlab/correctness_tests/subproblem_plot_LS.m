% Plot a graph of error vs theta for each least-squeares subproblem
% Also plot the subproblem solution, which should be at the minimum(s)

%%
plot_1d(subproblem_setups.sp_1())
%%
plot_2d(subproblem_setups.sp_2())
%%
plot_1d(subproblem_setups.sp_3())
%%
plot_1d(subproblem_setups.sp_4())

function plot_1d(setup)
    P = setup.setup_LS();
    S = setup.run(P);
    
    t_test = linspace(-pi, pi, 1000);
    e_test = NaN(size(t_test));
    
    for i = 1:length(t_test)
        S_t.theta = t_test(i);
        e_test(i) = setup.error(P, S_t);
    end
    
    plot(t_test, e_test);
    xline(S.theta);
end

function plot_2d(setup)
    P = setup.setup_LS();
    S = setup.run(P);

    t_test_single = linspace(-pi, pi, 100);
    [t_test_1, t_test_2] = meshgrid(t_test_single, t_test_single);
    e_test = NaN(size(t_test_1));
    for i = 1:numel(t_test_1)
        S_t.theta1 = t_test_1(i);
        S_t.theta2 = t_test_2(i);
        e_test(i) = setup.error(P, S_t);
    end
    
    imagesc(t_test_single, t_test_single, e_test)
    hold on
    ind_min = e_test == min(min(e_test));
    plot(t_test_1(ind_min), t_test_2(ind_min), 'go', 'LineWidth',2)
    plot(S.theta1(1), S.theta2(1), 'rx', 'LineWidth',2)
    if numel(S.theta1) > 1
        plot(S.theta1(2), S.theta2(2), 'rx', 'LineWidth',2)
    end
    hold off
    
    colormap turbo
end