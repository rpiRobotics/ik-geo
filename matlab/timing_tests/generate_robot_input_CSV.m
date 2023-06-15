N = 10e3;
% N = 10;
% N=100;

file_location = "../../test_cases/";

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

for i = 1:length(setups)
    setup = setups{i};
    rng default
    [T, P_list, S_list] = get_table(setup, N);
    
    
    class_name = string(class(setup)).split(".");
    
    writetable(T,  file_location + class_name(end) + ".csv");
    save(file_location + class_name(end) + ".mat", "P_list", "S_list");
end

%%
function [T, P_list, S_list] = get_table(setup, N)
    [P, S] = setup.setup;
    P_CSV.H = P.kin.H;
    P_CSV.P = P.kin.P;
    P_CSV.R = P.R;
    P_CSV.T = P.T;
    names = [get_col_names(P_CSV) get_col_names(S)];
    P_list = repmat(P,N,0);
    S_list = repmat(S,N,0);
    
    M = nan(N, length(names));
    for i = 1:N
        [P, S] = setup.setup;
        P_CSV.H = P.kin.H;
        P_CSV.P = P.kin.P;
        P_CSV.R = P.R;
        P_CSV.T = P.T;
        M(i,:) = [get_table_row(P_CSV) get_table_row(S)];
        P_list(i) = P;
        S_list(i) = S;
    
    end
    
    T = array2table(M, 'VariableNames',names);
end

function row = get_table_row(P)
    row = [];
    P_fields = fieldnames(P);
    for i = 1:length(P_fields)
        field = P.(P_fields{i});
        row = [row field(:)'];
    end
end

function names = get_col_names(P)
    names = {};
    P_fields = fieldnames(P);
    for i = 1:length(P_fields)
        size = numel(P.(P_fields{i}));
        if size == 1
            names{end+1} = P_fields{i};
        else
            for k = 1:size
                names{end+1} = strcat(P_fields{i}, '_', num2str(k));
            end
        end
    end

end