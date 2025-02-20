function Q_unique = unique_q_tol(Q, tol)
    if nargin < 2
        tol = 1e-6;
    end
    
    if isempty(Q)
        Q_unique = Q;
        return;
    end
    
    Q_unique = Q(:,1); % Start with the first column
    
    for i = 2:size(Q, 2)
        diffs = wrapToPi(Q_unique - Q(:,i));
        dists = vecnorm(diffs, 2, 1);
        
        if all(dists > tol)
            Q_unique = [Q_unique, Q(:,i)]; % Append if it's unique enough
        end
    end
end