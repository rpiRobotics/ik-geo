% Check correctness of subproblem solutions when exact solution may not exist
% (i.e. least squares solutions)

setups = {
    subproblem_setups.sp_1
    subproblem_setups.sp_2
    subproblem_setups.sp_3
    subproblem_setups.sp_4
};

at_mins = NaN(size(setups));

for N_trial = 1:100
for i = 1:length(setups)
    setup = setups{i};
    P = setup.setup_LS();
    S = setup.run(P);
    at_mins(i) = is_at_min(P,S, setup);
    
    if ~at_mins(i)
        error('Not at min')
    end
end
end

at_mins


function at_min = is_at_min(P,S,setup)
DELTA  = 1e-12;

e_0 = setup.error(P,S);

f = fieldnames(S);
for i = 1:length(f)
    for i_ind = 1:length(S.(f{i}))
    for sign = [+1 -1]
        S_test = S;
        S_test.(f{i})(i_ind) = S.(f{i})(i_ind) + sign*DELTA;
        if setup.error(P,S_test) < e_0 - DELTA
            at_min = false;
            return
        end
    end
    end
end

at_min = true;
end

