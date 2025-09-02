function Q_unique = unique_q_tol(Q, tol, mode)
    arguments
        Q (:,:) double
        tol (1,1) double = 1e-6
        mode (1,1) string {mustBeMember(mode, ["euclidean", "infinity"])} = "euclidean"
    end

    if isempty(Q)
        Q_unique = Q;
        return;
    end

    Q_unique = Q(:,1); % Start with the first column

    for i = 2:size(Q, 2)
        diffs = wrapToPi(Q_unique - Q(:,i));

        switch mode
            case "euclidean"
                dists = vecnorm(diffs, 2, 1);
                is_unique = all(dists > tol);
            case "infinity"
                max_diffs = max(abs(diffs), [], 1);
                is_unique = all(max_diffs > tol);
        end

        if is_unique
            Q_unique = [Q_unique, Q(:,i)];
        end
    end
end