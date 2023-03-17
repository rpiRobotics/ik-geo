% Check correctness of robot IK solutions when exact solution exists

setups = {
    IK_setups.IK_gen_6_dof
    IK_setups.IK_2_intersecting
    IK_setups.IK_2_parallel
    IK_setups.IK_spherical
    IK_setups.IK_spherical_2_intersecting
    IK_setups.IK_spherical_2_parallel
    IK_setups.IK_3_parallel
    IK_setups.IK_3_parallel_2_intersecting
};

%% Just 1 test
setup = IK_setups.IK_2_parallel;

[P, S_given] = setup.setup();
setup.error(P,S_given) % Make sure S_given is correct

% P = setup.setup_LS();

S = setup.run(P);
%S = setup.run_mex(P);

S.is_LS
    
[e, e_R, e_T] = robot_IK_error(P,S);
e

% S_exact.Q = S.Q(:,~S.is_LS);
% e = setup.error(P, S_exact)
%% Multiple test
N_trials = 100;
min_errors = NaN(length(setups), N_trials);

for i = 1:length(setups)
    disp("Setup "+ i +" / " + length(setups))
    for j = 1:N_trials
        setup = setups{i};
        [P, S_given] = setup.setup();
        
        % S = setup.run_mex(P);
        S = setup.run(P);
        e = robot_IK_error(P,S);
        min_errors(i,j) = min(e);    
    end
end

%% 

mean(min_errors')
max(min_errors')

histogram(min_errors(1,:)); hold on
histogram(min_errors(2,:));
histogram(min_errors(3,:));
histogram(min_errors(4,:));
histogram(min_errors(5,:));
histogram(min_errors(6,:));
histogram(min_errors(7,:)); hold off
set(gca,'XScale','log')