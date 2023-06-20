N = 10e3;
%N = 10;

file_location = "../../test_cases/";

setups = {
    subproblem_setups.sp_1
    subproblem_setups.sp_2
    subproblem_setups.sp_3
    subproblem_setups.sp_4
    subproblem_setups.sp_5
    subproblem_setups.sp_6
};

for i = 1:length(setups)
    setup = setups{i};
    rng default
    [T, P_list, S_list] = get_table(setup, N);
    
    class_name = string(class(setup)).split(".");
    
    writetable(T, file_location + class_name(end) + ".csv");
    save(file_location + class_name(end) + ".mat", "P_list", "S_list");
end
%%
function [T, P_list, S_list] = get_table(setup, N)
    [P, S] = setup.setup;
    names = [get_col_names(P) get_col_names(S)];
    P_list = repmat(P,N,0);
    S_list = repmat(S,N,0);
    
    M = nan(N, length(names));
    for i = 1:N
        [P, S] = setup.setup;
        M(i,:) = [get_table_row(P) get_table_row(S)];
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