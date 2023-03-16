%% Read in test cases

setup = subproblem_setups.sp_2;

T = readtable("../../test_cases/sp_2.csv");
N = height(T);
[P, S] = setup.setup();
P_list = repmat(P, N, 0);
S_list = repmat(S, N, 0);

P_template = repmat(P,0,0);
%S_template = repmat(S,0,0);
%P_template = P;
parfor i = 1:N
    P_list(i) = row2struct(T(i,:), P_template);
    %S_list(i) = row2struct(T(i,:), S_template);
    if mod(i,100000 / 100) == 0
        disp(i/N)
    end
end

function template = row2struct(row, template)
    col_names = row.Properties.VariableNames;
    for i = 1:length(col_names)
        col_name = col_names{i};
        split_name = split(col_name,'_');
        if ~isfield(template, split_name(1))
            continue
        end
        if length(split_name) == 1
            template(1).(col_name) = row.(col_name);
        else
            template(1).(split_name{1})(str2double(split_name{2}),1) = row.(col_name);
        end
    
    end
end