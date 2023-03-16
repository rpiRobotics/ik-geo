rng default
% null_fun = @null; % 3.199386
null_fun = @qr_null; % 1.746832 

N = 1e5;

tic
for i = 1:N
    poly = rand([2 4]);
    S = null_fun(poly);
end
toc

%%
A = rand([2 4])
A*qr_null(A)

function n = qr_null(A)
    [Q,~] = qr(A');
    n = Q(:,3:4);
end