function r = est_metrics(y, mu, sd)
%EST_METRICS  Accuracy of one estimator on a set of samples.
%   r = est_metrics(y, mu, sd)   y true times, mu predictions, sd predictive
%   standard deviations ([] or NaN for estimators without uncertainty).
%   Fields: n, rmse, mae, bias (mu - y), within2 / within5 (% with
%   |error| < 2 s / 5 s), cov90 (% of y inside mu +- 1.645 sd), nll
%   (mean negative log predictive density), fail (% of predictions that
%   are not finite: the IDM forward simulation can fail to reach the line).
y = y(:); mu = mu(:);
fin = isfinite(mu);
r.n = sum(fin);
r.fail = 100 * mean(~fin);
e = mu(fin) - y(fin);
if isempty(e)
    r.rmse = NaN; r.mae = NaN; r.bias = NaN; r.within2 = NaN; r.within5 = NaN;
else
    r.rmse = sqrt(mean(e.^2)); r.mae = mean(abs(e)); r.bias = mean(e);
    r.within2 = 100 * mean(abs(e) < 2); r.within5 = 100 * mean(abs(e) < 5);
end
r.cov90 = NaN; r.nll = NaN;
if nargin > 2 && ~isempty(sd) && any(isfinite(sd(:)))
    s = sd(:); s = s(fin);
    r.cov90 = 100 * mean(abs(e) <= 1.645 * s);
    r.nll = mean(0.5 * log(2 * pi * s.^2) + 0.5 * (e ./ s).^2);
end
end
