function M = gp_calibrate(M, s2, tau, r)
%GP_CALIBRATE  Fit the horizon-dependent noise of gp_sd.
%   M = gp_calibrate(M, s2, tau, r)
%   s2   GP predictive variances, tau predicted times to arrival,
%   r    residuals (actual minus predicted) on calibration data.
%   Maximum likelihood for r ~ N(0, var_latent + (c0 + c1 tau)^2) over
%   c0 > 0, c1 > 0 (fminsearch on log-parameters, so both stay positive).
lat = max(s2(:) - (M.sn * M.ys)^2, 0); tau = max(tau(:), 0); r = r(:);
ok = isfinite(lat) & isfinite(tau) & isfinite(r);
lat = lat(ok); tau = tau(ok); r = r(ok);
f = @(h) nll(h, lat, tau, r);
h = fminsearch(f, [log(0.5), log(0.1)], optimset('Display', 'off', 'MaxIter', 400));
M.cal = exp(h);
end

function J = nll(h, lat, tau, r)
v = lat + (exp(h(1)) + exp(h(2)) * tau).^2;
J = sum(0.5 * log(v) + 0.5 * r.^2 ./ v);
end
