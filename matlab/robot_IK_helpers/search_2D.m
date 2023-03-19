function [x1_vec, x2_vec, soln_num_vec] = search_2D(fun, min1, max1, min2, max2, N, show_plot)
N_max_minima = 100;
MIN_THRESH = 1e-1;

x1_vec = NaN(N_max_minima, 1);
x2_vec = NaN(N_max_minima, 1);
soln_num_vec = NaN(N_max_minima, 1);

% Sample the search space
x1_search_vec = linspace(min1, max1, N);
x2_search_vec = linspace(min2, max2, N);
[x1_mesh, x2_mesh] = meshgrid(x1_search_vec, x2_search_vec);

test_output = fun(min1, min2);
N_branches = length(test_output);

e_mesh = NaN([size(x1_mesh), N_branches]);

for i = 1:N
    for j = 1:N
        x1_ij = x1_mesh(i,j);
        x2_ij = x2_mesh(i,j);
        f_ij = fun(x1_ij, x2_ij);
        e_mesh(i,j,:) = f_ij;
    end
end

% Find approximate minima
lt_thresh_mat = e_mesh < MIN_THRESH;
e_mesh_iter = e_mesh;
e_mesh_iter(~lt_thresh_mat) = NaN;

for i_minimum = 1:N_max_minima
    % Find and save smallest value
    [x_min_i, idx_min_i] = min(e_mesh_iter,[], "all", "omitnan");
    [idx_min_1, idx_min_2, idx_min_3] = ind2sub(size(e_mesh_iter), idx_min_i);
    if isnan(x_min_i) || x_min_i > MIN_THRESH
        break
    end
    x1_vec(i_minimum) = x1_mesh(idx_min_1, idx_min_2);
    x2_vec(i_minimum) = x2_mesh(idx_min_1, idx_min_2);
    soln_num_vec(i_minimum) = idx_min_3;

    % Remove all neighbors below a threshold
    % (But only for that branch)
    [neighbors_idx] = find_blob(idx_min_1, idx_min_2, lt_thresh_mat(:,:,idx_min_3));
    [x1_idx, x2_indx] = ind2sub(size(lt_thresh_mat(:,:,idx_min_3)), neighbors_idx);
    neighbors_idx_3d = sub2ind(size(lt_thresh_mat),x1_idx,x2_indx, idx_min_3*ones(size(x2_indx)));

    lt_thresh_mat(neighbors_idx_3d) = 0;
    e_mesh_iter(neighbors_idx_3d) = NaN;

end
if i_minimum == N_max_minima
    error("Too many minima found!")
end
x1_vec = x1_vec(1:i_minimum-1);
x2_vec = x2_vec(1:i_minimum-1);
soln_num_vec = soln_num_vec(1:i_minimum-1);

opts = optimset("Display", "none");
% Optimize each minimum
for i = 1:length(x1_vec)

    x_12 = fminsearch(@(ip)(select_soln(fun, ip(1), ip(2), soln_num_vec(i))), [x1_vec(i) x2_vec(i)], opts);
    x1_vec(i) = x_12(1);
    x2_vec(i) = x_12(2);
end

% Plot results
if show_plot
    figure; view(3); hold on;
    for i = 1:N_branches
        mesh(x1_mesh, x2_mesh, e_mesh(:,:,i));
    end
    hold off;

    hold on
    plot(x1_vec, x2_vec, 'xr')
    hold off;
end

end

function neighbors_idx = find_blob(x1, x2, bool_mat)
sz = size(bool_mat);
neighbors_idx = [];
inner(x1, x2);

    function inner(starting_x1, starting_x2)
        % Wrap for torus (e.g. angle search)
        starting_x1 = mod(starting_x1-1, sz(1))+1;
        starting_x2 = mod(starting_x2-1, sz(2))+1;

        idx = sub2ind(sz,starting_x1,starting_x2);
        if any(neighbors_idx == idx)
            return % Already seen this node
        end
        if ~bool_mat(idx)
            return % Not part of blob
        end
        neighbors_idx = [ neighbors_idx idx];

        inner(starting_x1+1, starting_x2);
        inner(starting_x1-1, starting_x2);
        inner(starting_x1  , starting_x2+1);
        inner(starting_x1  , starting_x2-1);
    end
end

function x = select_soln(fun, x1, x2, soln_num)
    v = fun(x1, x2);
    x = v(soln_num);
end