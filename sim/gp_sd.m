function sd = gp_sd(M, s2, tau)
%GP_SD  Predictive standard deviation with a horizon-dependent noise.
%   The GP has one noise level sn for every sample, but a vehicle about to
%   cross is much more predictable than one 150 m away. With a calibration
%   M.cal = [c0 c1] (gp_calibrate) the noise part is replaced by
%       sigma_noise(tau) = c0 + c1 tau
%   and the GP's own (latent) uncertainty is kept:
%       sd = sqrt(var_latent + sigma_noise(tau)^2).
%   Without M.cal the GP variance is used as it is.
s2 = s2(:)'; tau = tau(:)';
if isfield(M, 'cal') && numel(M.cal) == 2
    lat = max(s2 - (M.sn * M.ys)^2, 0);
    sd = sqrt(lat + (M.cal(1) + M.cal(2) * max(tau, 0)).^2);
else
    sd = sqrt(s2);
end
end
