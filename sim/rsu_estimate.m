function [F, tau, sd] = rsu_estimate(rsu, W, app, t, G, kind)
%RSU_ESTIMATE  Predicted stop-line arrival times of one approach.
%
%   [F, tau, sd] = rsu_estimate(rsu, W, app, t, G, kind)
%   kind  'idm'     IDM forward simulation (no uncertainty, sd = 0)
%         'phys'    queue-discharge physics only (sd = 0)
%         'gpr'     plain GPR on [d v gap g] (constant mean)
%         'pigpr'   physics-informed GPR: the physics prior is the IDM
%                   forward simulation (queue-discharge physics where it has
%                   no answer); a GPR learns the residual from
%                   [d v gap g n vl nR prior]
%         'pigpr_on' the online-updated copy of the PI-GPR
%   A vehicle whose prior arrival is after the end of the current yellow is
%   left over to the next green; the GPR is not applied to it (it was
%   trained on arrivals within the current green) and its prior is used.
%   F     features (est_features), vehicles closest first
%   tau   predicted time until each vehicle reaches its stop line [s]
%   sd    predictive standard deviation [s]
%   A minimum headway h_min between consecutive predicted arrivals is
%   enforced afterwards for every estimator (as in the conference method).

F = est_features(W, app, t, G, rsu.theta);
k = numel(F.ids);
tau = zeros(1, k); sd = zeros(1, k);
if k == 0, return; end
switch kind
    case 'idm'
        tau = est_idm(W, F, app, t, G);
    case 'phys'
        tau = F.tphys;
    case 'gpr'
        [mu, s2] = gp_predict(rsu.gpr, F.Z4);
        tau = mu(:)'; sd = sqrt(s2(:))';
        [~, ~, tend] = signal_info(t, app, G.C);
        left = t + F.tphys > tend;              % left over (physics decides)
        tau(left) = F.tphys(left);
    case {'pigpr', 'pigpr_on'}
        prior = est_idm(W, F, app, t, G);
        bad = ~isfinite(prior);
        prior(bad) = F.tphys(bad);
        Z = [F.Zb, prior(:)];
        if strcmp(kind, 'pigpr'), Mg = rsu.pigpr; else, Mg = rsu.pigpr_on; end
        [mu, s2] = gp_predict(Mg, Z);
        tau = prior + mu(:)'; sd = sqrt(s2(:))';
        [~, ~, tend] = signal_info(t, app, G.C);
        left = t + prior > tend;
        tau(left) = prior(left);
        F.prior = prior;
    otherwise
        error('rsu_estimate: unknown estimator %s', kind);
end
tau = max(tau, G.C.DT);
for q = 2:k
    if isfinite(tau(q-1)), tau(q) = max(tau(q), tau(q-1) + rsu.h_min); end
end
end
