function [rsu, msgs] = rsu_plan(rsu, W, a, t, sig, sigT, G, cfg)
%RSU_PLAN  Right-turn slot search for the first right-turner of approach a.
%
%   The conference slot search, with geometric and uncertainty-aware margins.
%
%   Known vehicles of the opposite approach o, in order of arrival, have
%   predicted stop-line times taubar_m and standard deviations sigma_m
%   (rsu_estimate). The turner has a predicted time taubar_t at its
%   waiting point. The turner is inserted before opposing vehicle p,
%   p = 1..M+1, and the coordinated times follow:
%     tau_t   = max(taubar_t, max_{m<p} tau_m + Db_m)
%     tau_m   = max(taubar_m, tau_t + Da_m, tau_{m-1} + h_min),  m >= p
%     tau_f   = max(taubar_f, tau_{f-1} + h_min)   turner's followers
%   with the margins
%     Db_m = dexit_m - dent_t + PET + z sqrt(sigma_t^2 + sigma_m^2)
%     Da_m = tclr_t  - dent_m + PET + z sqrt(sigma_t^2 + sigma_m^2)
%   dent / dexit: time from the stop line (opposing vehicle) or waiting
%   point (turner, starting from rest) until the vehicle front enters /
%   its rear leaves the conflict zone, from the real path geometry;
%   tclr_t: time until the turner's rear has left its zone.
%   Cost J(p) = sum (tau - taubar)^2 over opposing vehicles, turner and its
%   followers; the p with the smallest J is chosen (exhaustive, exact for
%   one turner). Vehicles already past their stop line cannot be delayed.
%   If the turner cannot clear its zone before the yellow ends, there is no
%   slot this green: no message, the turner clears at the end of green.
%
%   Messages (to CAVs only, sent every step while the plan stands):
%     turner                go = t + tau_t (time to leave the waiting point)
%     delayed opposing m    tgt = t + tau_m at its stop line (arrival-time
%                           MPC) and vadv = d_m / tau_m (speed advice)

msgs = struct('id', {}, 'vadv', {}, 'tgt', {}, 'spt', {}, 'go', {});
if ~strcmp(sig.(a), 'green') || isempty(W.ids), return; end
C = G.C; L = C.VEH_L; sStop = C.L_APP - C.STOP;
oppo = struct('E', 'W', 'W', 'E', 'N', 'S', 'S', 'N'); o = oppo.(a);
kp = G.idx.([a 'R']); P = G.P(kp);

% first right-turner of approach a that has not committed
cand = find(W.app == a & W.mov == 'R' & ~W.committed & W.s <= P.s_wait & W.s >= sStop - 120);
if isempty(cand), return; end
[~, j] = max(W.s(cand)); it = cand(j);     % the first one in the lane

rows = G.oppZ{kp};
if isempty(rows), return; end
vt = min(P.vcurve(P.kappa > 0));
z0k = min(rows(:, 2)); z1k = max(rows(:, 3));
dent_t = travel_time(z0k - P.s_wait, 0, 1.5, vt);
tclr_t = travel_time(z1k + L - P.s_wait, 0, 1.5, vt);
z = cfg.unc_z;

% ---- own approach: turner and its followers ----
[Fa, ta, sa] = rsu_estimate(rsu, W, a, t, G, cfg.est);
if W.s(it) < sStop
    q = find(Fa.ids == W.ids(it), 1);
    if isempty(q) || ~isfinite(ta(q)), return; end
    tbar_t = ta(q) + (P.s_wait - sStop) / vt;
    sig_t = sa(q);
    fol = (q+1):numel(Fa.ids);
else
    tbar_t = travel_time(P.s_wait - W.s(it), W.v(it), 1.5, vt);
    sig_t = 0.3;
    fol = 1:numel(Fa.ids);
end
tbar_f = ta(fol); tbar_f = tbar_f(isfinite(tbar_f));

% ---- opposing approach ----
[Fo, to, so] = rsu_estimate(rsu, W, o, t, G, cfg.est);
% vehicles already past their stop line and not yet clear (fixed)
pastIdx = find(W.app == o & W.s >= sStop & W.mov ~= 'R');
fx_t = []; fx_m = ''; fx_v = []; fx_s = [];
for r = pastIdx
    qz = rowFor(rows, G, o, W.mov(r));
    if isempty(qz) || W.s(r) - L > qz(5), continue; end
    fx_t(end+1) = -(W.s(r) - sStop) / max(W.v(r), 1); %#ok<AGROW>
    fx_m(end+1) = W.mov(r); fx_v(end+1) = W.v(r); fx_s(end+1) = W.s(r); %#ok<AGROW>
end
[~, oo] = sort(fx_s, 'descend');
tbar = [fx_t(oo), to];
sgm  = [zeros(1, numel(oo)), so];
movo = [fx_m(oo), Fo.mov];
vo   = [fx_v(oo), Fo.v];
fixed = [true(1, numel(oo)), false(1, numel(to))];
ids_o = [zeros(1, numel(oo)), Fo.ids];
d_o = [zeros(1, numel(oo)), Fo.d];
M = numel(tbar);
use = isfinite(tbar);

Db = zeros(1, M); Da = zeros(1, M); cons = false(1, M);
for m = 1:M
    qz = rowFor(rows, G, o, movo(m));
    if isempty(qz) || ~use(m), continue; end
    cons(m) = true;
    vc = max(vo(m), 6);
    dent = (qz(4) - sStop) / vc;
    dexit = (qz(5) + L - sStop) / vc;
    u = z * sqrt(sig_t^2 + sgm(m)^2);
    Db(m) = dexit - dent_t + rsu.pet + u;
    Da(m) = tclr_t - dent + rsu.pet + u;
end

best = inf; bp = 0;
for p = 1:M+1
    tt = tbar_t;
    for m = 1:min(p-1, M)
        if cons(m), tt = max(tt, tbar(m) + Db(m)); end
    end
    tau = tbar; ok = true; prev = -inf;
    for m = p:M
        if ~use(m), continue; end
        lo = max(tbar(m), prev + rsu.h_min);
        if cons(m), lo = max(lo, tt + Da(m)); end
        if fixed(m) && lo > tbar(m) + 0.01, ok = false; break; end
        tau(m) = lo; prev = lo;
    end
    if ~ok, continue; end
    tf = tbar_f; pv = tt;
    for f = 1:numel(tf), tf(f) = max(tf(f), pv + rsu.h_min); pv = tf(f); end
    J = sum((tau(use) - tbar(use)).^2) + (tt - tbar_t)^2 + sum((tf - tbar_f).^2);
    if J < best, best = J; bp = p; btau = tau; btt = tt; end
end
if bp == 0, return; end
rsu.plans = rsu.plans + 1;
tend = sigT.(a)(2);
if t + btt + tclr_t > tend
    rsu.noslot = rsu.noslot + 1;
    return;                                   % no slot in this green
end

vadv = NaN; if btt > 0.5, vadv = (P.s_wait - W.s(it)) / btt; end
msgs(end+1) = struct('id', W.ids(it), 'vadv', vadv, 'tgt', t + btt, 'spt', P.s_wait, 'go', t + btt);
for m = 1:M
    if fixed(m) || ~use(m) || movo(m) == 'R', continue; end   % never message the other turner
    if btau(m) - tbar(m) >= 0.5
        tg = t + btau(m);
        if ~strcmp(cfg.msg, 'time'), tg = NaN; end     % speed advice only: no binding time
        msgs(end+1) = struct('id', ids_o(m), 'vadv', d_o(m) / max(btau(m), 0.5), ...
                             'tgt', tg, 'spt', sStop, 'go', NaN); %#ok<AGROW>
    end
end
end

function r = rowFor(rows, G, o, mov)
% conflict row of the turner with an opposing vehicle of movement mov
r = [];
if mov == 'R', return; end
q = G.idx.([o mov]);
k = find(rows(:, 1) == q, 1);
if ~isempty(k), r = rows(k, :); end
end
