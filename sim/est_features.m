function F = est_features(view, app, t, G, theta)
%EST_FEATURES  Inputs of the arrival-time estimators for one approach.
%
%   F = est_features(view, app, t, G, theta)
%   view   the RSU's current picture (rsu_view): ids, s, v, a, p, app, mov
%   theta  physics parameters [t_L, h_s, a_p] (start-up lost time, discharge
%          headway, acceleration of the kinematic part)
%
%   Vehicles of the approach that are still before their stop line, sorted
%   closest first. For each:
%     d       distance to the stop line [m]
%     v       speed [m/s]
%     gap     bumper-to-bumper gap to the vehicle ahead (d for the first)
%     g       signal timing (signal_feature): green left (> 0) or minus the
%             time until green (< 0)
%     n       vehicles ahead that still have to pass (before the stop line,
%             or waiting in the junction)
%     vl      speed of the vehicle ahead (25 if none)
%     nR      right-turners ahead that have not turned yet (a waiting
%             right-turner blocks the single lane)
%     tphys   queue-discharge physics estimate (est_phys)
%   F.Z4 = [d v gap g]                 inputs of the plain GPR
%   F.Zb = [d v gap g n vl nR]         inputs of the PI-GPR (the prior is
%                                      appended by rsu_estimate)

C = G.C; L = C.VEH_L;
F = struct('ids', [], 'd', [], 'v', [], 'gap', [], 'g', [], 'n', [], 'vl', [], 'nR', [], ...
           'tphys', [], 'Z4', zeros(0, 4), 'Zb', zeros(0, 7), 'mov', '', 'prior', []);
if isempty(view.ids), return; end
sStop = C.L_APP - C.STOP;
inApp = view.app == app;
before = inApp & view.s < sStop;
if ~any(before), return; end
blocking = inApp & view.s < sStop + C.STOP & ~view.committed;   % still ahead in the lane
idx = find(before);
[~, o] = sort(view.s(idx), 'descend');
idx = idx(o);
[~, tg0, tend] = signal_info(t, app, C);
g = signal_feature(t, app);
k = numel(idx);
F.ids = view.ids(idx); F.d = sStop - view.s(idx); F.v = view.v(idx);
F.mov = view.mov(idx);
F.gap = zeros(1, k); F.n = zeros(1, k); F.vl = C.V_LIM * ones(1, k); F.g = g * ones(1, k);
F.nR = zeros(1, k);
sb = view.s(blocking); vb = view.v(blocking); rb = view.mov(blocking) == 'R';
for q = 1:k
    si = view.s(idx(q));
    ah = sb > si;
    F.n(q) = sum(ah);
    F.nR(q) = sum(ah & rb);
    if any(ah)
        [sa, ja] = min(sb(ah)); va = vb(ah);
        F.gap(q) = max(sa - L - si, 0);
        F.vl(q) = va(ja);
    else
        F.gap(q) = F.d(q);
    end
end
F.tphys = est_phys(F.d, F.v, F.n, t, tg0, tend, theta, C);
F.Z4 = [F.d(:), F.v(:), F.gap(:), F.g(:)];
F.Zb = [F.d(:), F.v(:), F.gap(:), F.g(:), F.n(:), F.vl(:), F.nR(:)];
end
