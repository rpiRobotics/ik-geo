function runtimes = run_timing_test(setup_func, run_funcs, N_trials, N_attempts)

runtimes = NaN([N_trials length(run_funcs)]);

for i = 1:N_trials
    if any((i / N_trials) == [0 0.2 0.4 0.6 0.8])
        disp(i/N_trials)
    end

    problem_setup = setup_func();
    
    for i_run_func = 1:length(run_funcs)
        for i_throwout = 1:3
            run_func = run_funcs{i_run_func};
        end
        
        for i_trial = 1:N_trials
            tic
            for i_attempt = 1:N_attempts
                soln = run_func(problem_setup);
            end
            t = toc;
            runtimes(i_trial, i_run_func) = t;
        end
    end
    

end

runtimes = runtimes / N_attempts;

end
