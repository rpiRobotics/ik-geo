function [x_vec, soln_num_vec] = search_1D(fun, x1, x2, N, show_plot)
% Inputs
%   Minimization function (vector valued)
%   Search interval
%   Number of initial samples
%   Plotting on/off
% Outputs
%   Vector of zeros locations
%   Vector of which index of the function has the zero

% Sample the search space
x_sample_vec = linspace(x1, x2, N);
e_1 = fun(x_sample_vec(1)); % Use to find size
e_mat = NaN([length(e_1) N]);
e_mat(:,1) = e_1;
for i = 2:N
    e_mat(:,i) = fun(x_sample_vec(i));
end

% Find zero crossings
% Ignore very large crossings, as this may be caused by angle wrapping
CROSS_THRESH = 0.1;
zero_cross_direction = diff(e_mat<0, 1,2)~=0 & abs(e_mat(:,2:end)) < CROSS_THRESH & abs(e_mat(:,1:end-1)) < CROSS_THRESH;
has_zero_cross = sum(abs(zero_cross_direction));
crossings_left = x_sample_vec(has_zero_cross>0);
crossings_right = x_sample_vec([false has_zero_cross>0]);

crossing_soln_nums = zero_cross_direction(:,has_zero_cross>0);
n_zeros = sum(crossing_soln_nums(:));

% Iterate on each bracket
options = optimset('Display','off', 'TolX', 1e-5);

ind_soln = 1;
x_vec = NaN(1, n_zeros);
soln_num_vec = NaN(1, n_zeros);
for i = 1:length(crossings_left)
    soln_nums = find(crossing_soln_nums(:,i));
    for i_soln_num = 1:length(soln_nums)
        soln_num = soln_nums(i_soln_num);
        x_vec(ind_soln) = fzero(@(x)(select_soln(fun(x),soln_num)), [crossings_left(i) crossings_right(i)], options);
        soln_num_vec(ind_soln) = soln_num;
        ind_soln = ind_soln + 1;
    end
end

% Plot results
if show_plot
    plot(x_sample_vec, e_mat, '.');
    yline(0);
    if ~isempty(x_vec)
        xline(x_vec);
    end
end

end

function x = select_soln(x_arr, soln_num)
    x = x_arr(soln_num);
end