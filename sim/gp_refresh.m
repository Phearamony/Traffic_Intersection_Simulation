function M = gp_refresh(M)
%GP_REFRESH  Recompute the factors used by gp_predict from M.A and M.b.
m = size(M.A, 1);
M.A = 0.5 * (M.A + M.A');
[LA, p] = chol(M.A, 'lower');
if p > 0
    LA = chol(M.A + 1e-6 * M.sf^2 * eye(m), 'lower');
end
Lm = chol(M.Kmm, 'lower');
M.w    = LA' \ (LA \ M.b);                  % A^-1 b
M.Ainv = LA' \ (LA \ eye(m));
M.Kinv = Lm' \ (Lm \ eye(m));
end
