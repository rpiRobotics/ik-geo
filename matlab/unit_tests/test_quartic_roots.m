EQN = rand(1, 5);
% EQN = [0 rand(1, 4)];
% EQN = [0 0 rand(1, 3)];
% EQN = [0 0 0 rand(1, 2)];
% EQN = [1 2 3 4 5]
% EQN = [1 -10 35 -50 24] % beta = 0
% EQN = [1 -7 17 -17 6] % repeated root (double)
% EQN = [1 -6 12 -10 3] % Repeated root (triple),  U = 0

rts = subproblem.quartic_roots(EQN)
polyval(EQN, rts)