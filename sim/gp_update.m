function M = gp_update(M, X, y, lambda)
%GP_UPDATE  Add observations to a sparse GP (online learning).
%   M = gp_update(M, X, y)          X k-by-D, y k-by-1 (original units)
%   M = gp_update(M, X, y, lambda)  forgetting factor in (0, 1]: the old
%                                   data are down-weighted by lambda first
%   With DTC the posterior depends on the data only through
%       A = Kmm + sn^-2 sum_i k_m(x_i) k_m(x_i)',   b = sn^-2 sum_i k_m(x_i) y_i
%   so observations are added by a low-rank update of A and b (one rank-one
%   term per observation; the forgetting factor is applied once per call).
if nargin < 4, lambda = 1; end
if isempty(X), return; end
Xn = (X - M.xm) ./ M.xs;
yn = (y(:) - M.ym) / M.ys;
d2 = zeros(size(M.Z, 1), size(Xn, 1));
for d = 1:size(Xn, 2)
    d2 = d2 + ((M.Z(:, d) - Xn(:, d)') / M.ell(d)).^2;
end
Km = M.sf^2 * exp(-0.5 * d2);
M.A = M.Kmm + lambda * (M.A - M.Kmm) + (Km * Km') / M.sn^2;
M.b = lambda * M.b + Km * yn / M.sn^2;
M = gp_refresh(M);
M.n = M.n + size(X, 1);
end
