N = 10e3;
% N = 10;
% N=100;

file_location = "../../test_cases/";

setups = {
    hardcoded_IK_setups.ur5
    hardcoded_IK_setups.IRB_6640
    hardcoded_IK_setups.two_parallel_bot
    hardcoded_IK_setups.three_parallel_bot
    hardcoded_IK_setups.KUKA_R800_fixed_q3
    hardcoded_IK_setups.yumi_fixed_q3
    hardcoded_IK_setups.RRC_fixed_q6
    hardcoded_IK_setups.spherical_bot
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
    P_CSV.R = P.R;
    P_CSV.T = P.T;
    names = [get_col_names(P_CSV) get_col_names(S)];
    P_list = repmat(P,N,0);
    S_list = repmat(S,N,0);
    
    M = nan(N, length(names));
    for i = 1:N
        [P, S] = setup.setup;
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