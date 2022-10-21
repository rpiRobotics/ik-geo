function runtimes = run_timing_test(setup_func, run_funcs, N_trials, N_attempts, bypass_timing_check)
assert(mod(N_attempts,10) == 0);
runtimes = NaN([N_trials length(run_funcs)]);

if nargin < 5
    bypass_timing_check = false;
end

disp("Starting " + functions(run_funcs{1}).function)

for i = 1:N_trials
    if bypass_timing_check
        disp(i/N_trials)
    else
        if any((i / N_trials) == [0 0.2 0.4 0.6 0.8])
            disp(i/N_trials)
        end
    end

    problem_setup = setup_func();
    
    for i_run_func = 1:length(run_funcs)
        
        run_func = run_funcs{i_run_func};
        

        for i_throwout = 1:3
            soln = run_func(problem_setup);
        end
        
        for i_trial = 1:N_trials
            tic();
            for i_attempt = 1:(N_attempts/10)
                soln = run_func(problem_setup); % 1
                soln = run_func(problem_setup); % 2
                soln = run_func(problem_setup); % 3
                soln = run_func(problem_setup); % 4
                soln = run_func(problem_setup); % 5
                soln = run_func(problem_setup); % 6
                soln = run_func(problem_setup); % 7
                soln = run_func(problem_setup); % 8
                soln = run_func(problem_setup); % 9
                soln = run_func(problem_setup); % 10
            end
            t = toc();
            if ~bypass_timing_check
                assert(t > 0.001, 'inaccurate result: increase N_attempts for %s', functions(run_func).function);
                assert(t < 0.01, ' inaccurate result: decrease N_attempts for %s', functions(run_func).function');
            end
            runtimes(i_trial, i_run_func) = t;
        end
    end
end

runtimes = runtimes / N_attempts;
end
