function [q, index_q, diff_norm] = closest_q(Q, q_prev)
% Finds q = argmin_{q_i in Q} norm(q_i - q_prev)
% Q is a matrix [q_1 q_2 ... q_N]

Q_diff = wrapToPi(Q - q_prev);
norms = vecnorm(Q_diff);
[diff_norm, index_q] = min(norms);
q = Q(:,index_q);
end