function M = gp_fit(X, y, opts)
%GP_FIT  Sparse Gaussian process regression (DTC) with an ARD kernel.
%
%   M = gp_fit(X, y, opts)
%   X  n-by-D inputs, y n-by-1 targets.
%   opts.m       number of inducing points (default 300)
%   opts.n_opt   points used to fit the hyperparameters (default 800)
%   opts.iter    optimizer iterations (default 150)
%   opts.seed    seed for the subsets (default 1)
%
%   Inputs and targets are standardised. Kernel (ARD squared exponential)
%       k(x, x') = sf^2 exp(-1/2 sum_d (x_d - x'_d)^2 / l_d^2)
%   Hyperparameters [log l_1..l_D, log sf, log sn] maximize the exact
%   marginal likelihood on a random subset of n_opt points (analytic
%   gradient, fminunc). The full data then enter through the
%   deterministic training conditional (DTC) on m inducing points Z:
%       A = Kmm + sn^-2 Kmn Knm,      b = sn^-2 Kmn y
%       mean(x) = k_m(x)' A^-1 b
%       var(x)  = k(x,x) - k_m(x)' (Kmm^-1 - A^-1) k_m(x) + sn^2
%   A and b are sums over the data, so gp_update can add one observation
%   at a time (online learning) at O(m^2) cost.

if nargin < 3, opts = struct(); end
if ~isfield(opts, 'm'), opts.m = 300; end
if ~isfield(opts, 'n_opt'), opts.n_opt = 800; end
if ~isfield(opts, 'iter'), opts.iter = 150; end
if ~isfield(opts, 'seed'), opts.seed = 1; end

[n, D] = size(X);
M.xm = mean(X, 1); M.xs = std(X, 0, 1); M.xs(M.xs < 1e-9) = 1;
M.ym = mean(y);    M.ys = std(y);       if M.ys < 1e-9, M.ys = 1; end
Xn = (X - M.xm) ./ M.xs;
yn = (y - M.ym) / M.ys;

% ---- hyperparameters on a subset ----
r = rand_stream(9e5 + opts.seed, 1, n);
[~, perm] = sort(r);
io = perm(1:min(opts.n_opt, n));
h0 = [zeros(1, D), 0, log(0.3)];               % l = 1, sf = 1, sn = 0.3 (standardised)
f = @(h) nlml(h, Xn(io, :), yn(io));
% weak log-normal prior on the length scales (standardised inputs): keeps
% them away from tiny values that only fit noise in the subset

try
    o = optimset('GradObj', 'on', 'MaxIter', opts.iter, 'Display', 'off', 'TolFun', 1e-6, 'TolX', 1e-6);
    h = fminunc(f, h0(:), o);
catch err
    fprintf(2, '[gp_fit] hyperparameter optimization failed (%s); default values used\n', err.message);
    h = h0(:);
end
h = h(:)';
h(end) = max(h(end), log(1e-3));               % keep the noise away from zero
M.ell = exp(h(1:D)); M.sf = exp(h(D+1)); M.sn = exp(h(D+2));
M.D = D;

% ---- inducing points and DTC sums ----
m = min(opts.m, n);
iz = perm(end - m + 1:end);                    % a different random subset
M.Z = Xn(iz, :);
M.Kmm = kern(M.Z, M.Z, M) + 1e-6 * M.sf^2 * eye(m);
M.A = M.Kmm;  M.b = zeros(m, 1);
blk = 4000;
for k0 = 1:blk:n
    k1 = min(n, k0 + blk - 1);
    Kmn = kern(M.Z, Xn(k0:k1, :), M);
    M.A = M.A + (Kmn * Kmn') / M.sn^2;
    M.b = M.b + Kmn * yn(k0:k1) / M.sn^2;
end
M = gp_refresh(M);
M.n = n;
M.nlml = f(h(:));
end

function K = kern(A, B, M)
d2 = zeros(size(A, 1), size(B, 1));
for d = 1:size(A, 2)
    d2 = d2 + ((A(:, d) - B(:, d)') / M.ell(d)).^2;
end
K = M.sf^2 * exp(-0.5 * d2);
end

function [f, g] = nlml(h, X, y)
[n, D] = size(X);
ell = exp(h(1:D)); sf2 = exp(2 * h(D+1)); sn2 = exp(2 * h(D+2));
Dd = cell(1, D); d2 = zeros(n);
for d = 1:D
    Dd{d} = ((X(:, d) - X(:, d)') / ell(d)).^2;
    d2 = d2 + Dd{d};
end
Kf = sf2 * exp(-0.5 * d2);
K = Kf + (sn2 + 1e-8) * eye(n);
[L, p] = chol(K, 'lower');
if p > 0, f = 1e10; g = zeros(size(h)); return; end
alpha = L' \ (L \ y);
f = 0.5 * (y' * alpha) + sum(log(diag(L))) + 0.5 * n * log(2 * pi);
lp = h(1:D); f = f + 0.5 * sum((lp(:) / 1.5).^2);      % prior on log length scales
if nargout > 1
    Ki = L' \ (L \ eye(n));
    W = alpha * alpha' - Ki;
    g = zeros(size(h));
    for d = 1:D
        g(d) = -0.5 * sum(sum(W .* (Kf .* Dd{d}))) + h(d) / 1.5^2;
    end
    g(D+1) = -0.5 * sum(sum(W .* (2 * Kf)));
    g(D+2) = -0.5 * trace(W) * 2 * sn2;
end
end
