function rsu = rsu_crossed(rsu, veh, t_stop, cfg)
%RSU_CROSSED  A vehicle crossed its stop line at t_stop: use its snapshots.
%   Every stored snapshot of this vehicle becomes a sample with the true
%   remaining time y = t_stop - t_snap:
%   collect  rsu.samples  [t_snap id app(1=E,2=W) d v gap g n vl nR prior tphys y idm]
%   eval     rsu.evalrows [t_snap id y g idm phys gpr sd pigpr sd pigpr_on sd]
%   online   the PI-GPR copy learns the residual y - prior (gp_update with
%            forgetting factor rsu.lambda), only from arrivals within the
%            green in which the snapshot was taken (the training scope) and
%            only for vehicles the RSU knows and did not command
if isempty(rsu.pending), return; end
m = [rsu.pending.id] == veh.id;
if ~any(m), return; end
P = rsu.pending(m);
rsu.pending = rsu.pending(~m);
y = t_stop - [P.t];
keep = y > 0 & y <= 120;
P = P(keep); y = y(keep);
if isempty(P), return; end
Z = reshape([P.z], 8, [])';
tp = [P.tphys]';
pr = reshape([P.pred], 8, [])';
if rsu.collect
    rsu.samples = [rsu.samples; [[P.t]', repmat(veh.id, numel(P), 1), [P.app]', Z, tp, y(:), pr(:, 1)]];
end
if rsu.eval
    rsu.evalrows = [rsu.evalrows; [[P.t]', repmat(veh.id, numel(P), 1), y(:), [P.g]', pr]];
end
online = (rsu.eval || (rsu.plan && cfg.online && strcmp(cfg.est, 'pigpr'))) && ~isempty(rsu.pigpr_on);
% arrivals the RSU changed itself (shaped, pulled, eco) are not natural
% arrivals: they are not learned from
if online && rsu.plan && (any(rsu.shp.id == veh.id) || any(rsu.pul.id == veh.id) || any(rsu.eco == veh.id))
    online = false;
end
if online
    inph = [P.g]' > 0 & y(:) <= [P.g]' + 1e-6;
    if any(inph)
        rsu.pigpr_on = gp_update(rsu.pigpr_on, Z(inph, :), y(inph)' - Z(inph, 8), rsu.lambda);
        rsu.online_n = rsu.online_n + sum(inph);
    end
    if rsu.plan, rsu.pigpr = rsu.pigpr_on; end
end
end
