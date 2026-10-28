function [mu, s2] = gp_predict(M, X)
%GP_PREDICT  Mean and variance of the sparse GP at the rows of X.
%   Returns values in the original target units.
Xn = (X - M.xm) ./ M.xs;
d2 = zeros(size(M.Z, 1), size(Xn, 1));
for d = 1:size(Xn, 2)
    d2 = d2 + ((M.Z(:, d) - Xn(:, d)') / M.ell(d)).^2;
end
Km = M.sf^2 * exp(-0.5 * d2);                       % m-by-n
mu = (Km' * M.w) * M.ys + M.ym;
if nargout > 1
    q = sum(Km .* ((M.Kinv - M.Ainv) * Km), 1)';
    s2 = max(M.sf^2 - q, 0) + M.sn^2;
    s2 = s2 * M.ys^2;
end
end
