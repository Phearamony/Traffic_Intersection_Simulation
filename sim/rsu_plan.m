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
%   point (turner: rolling at curve speed when its arrival-time MPC can
%   time the slot, else from rest) until the vehicle front enters /
%   its rear leaves the conflict zone with vehicle m, from the real path
%   geometry; tclr_t: time until the turner's rear has left that zone.
%   Gap shaping (Road Traffic Act art. 37: the turner yields; it only uses a
%   gap that exists when it turns, checked on board). The RSU may make a gap
%   by asking oncoming CAVs to arrive a little later, within limits:
%     lo_m <= tau_m <= hi_m,   hi_m = nat_m + min(dmax, gentle_m)
%   nat_m: natural arrival (prediction when first shaped); gentle_m: the
%   delay reached by slowing to no less than v_frac of its speed, with a
%   deceleration of at most a_shape, over half of its distance d_m; no new
%   command once d_m < d_cmd or the vehicle is queued (v < 5 m/s). Human
%   drivers and vehicles past the stop line cannot be asked (hi = lo), but a
%   vehicle behind a slowed one follows it (tau_m >= tau_{m-1} + h_min). A
%   given target is never taken back (lo_m >= that target).
%   Commanded vehicles (arrival-time MPC configurations): the turner leaves
%   at its go time and a held oncoming CAV cannot reach its stop line before
%   its target, so in their margins the prediction sigma is replaced by the
%   tracking accuracy s_ctl = 0.3 s. An oncoming CAV may also be held at its
%   natural time (no delay) only to remove that uncertainty; a vehicle
%   behind a held one cannot be early either.
%   Cost J(p) = w_onc sum (tau_m - nat_m)^2 + (tau_t - taubar_t)^2
%               + sum over the turner's followers (tau_f - taubar_f)^2.
%   Rules (cfg.slot):
%     'opt'     the feasible p with the smallest J (exhaustive, exact for one turner)
%     'strict'  no shaping (hi = lo): existing gaps only
%     'greedy'  the feasible p with the earliest turner time (ignores the
%               cost to oncoming traffic)
%     'fcfs'    first come, first served: the first feasible p at or after
%               the turner's natural position in the arrival order
%   The turner's own best time taubar_t is its unobstructed arrival, so it
%   is never asked to wait longer than its slot needs; the arrival-time MPC
%   lets it roll into the slot instead of stopping.
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
% speed of the turner at its waiting point: an arrival-time MPC turner that
% is still moving rolls into its slot at curve speed; otherwise from rest
v_t0 = 0;
if strcmp(cfg.msg, 'time') && strcmp(cfg.ctrl, 'mpc') && W.v(it) > 1 && W.s(it) < P.s_wait - 1
    v_t0 = vt;
end

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

% Vehicles the RSU commands with an arrival time (arrival-time MPC) are
% not uncertain in the risky direction: the turner leaves at its go time,
% and a held oncoming CAV cannot reach its stop line before its target.
% Their prediction sigma is replaced by the tracking accuracy s_ctl.
ctl = strcmp(cfg.msg, 'time') && strcmp(cfg.ctrl, 'mpc');
s_ctl = 0.3;
sig_te = sig_t; if ctl, sig_te = s_ctl; end
Db = zeros(1, M); Da = zeros(1, M); DaC = zeros(1, M); cons = false(1, M);
for m = 1:M
    qz = rowFor(rows, G, o, movo(m));
    if isempty(qz) || ~use(m), continue; end
    cons(m) = true;
    vc = max(vo(m), 6);
    dent = (qz(4) - sStop) / vc;
    dexit = (qz(5) + L - sStop) / vc;
    % the turner's entry / clearing time of THIS conflict zone (not of the
    % whole turn, which also includes the merge with oncoming left-turners)
    dent_m = travel_time(qz(2) - P.s_wait, v_t0, 1.5, vt);
    tclr_m = travel_time(qz(3) + L - P.s_wait, v_t0, 1.5, vt);
    u = z * sqrt(sig_te^2 + sgm(m)^2);
    Db(m) = dexit - dent_m + rsu.pet + u;
    Da(m) = tclr_m - dent + rsu.pet + u;
    DaC(m) = tclr_m - dent + rsu.pet + z * sqrt(sig_te^2 + s_ctl^2);   % if held
end

% ---- shaping limits of every oncoming vehicle ----
% lo   earliest arrival: the prediction, or the arrival time it has already
%      been given (a shaping command is kept, never taken back)
% nat  natural arrival: the prediction when it was first shaped
% hi   latest arrival the RSU may ask for (CAVs only, see header)
lo = tbar; nat = tbar; hi = tbar;
[~, ix] = ismember(Fo.ids, W.ids);
cav_o = [false(1, numel(oo)), W.cav(ix)];
for m = 1:M
    if fixed(m) || ~use(m), continue; end
    k = find(rsu.shp.id == ids_o(m), 1);
    if ~isempty(k)
        lo(m) = max(tbar(m), rsu.shp.tgt(k) - t);
        nat(m) = rsu.shp.nat(k) - t;
    end
    hi(m) = lo(m);
    if ~cav_o(m) || strcmp(cfg.slot, 'strict') || movo(m) == 'R', continue; end
    if d_o(m) < rsu.d_cmd || vo(m) < 5, continue; end        % too close or queued
    % slow down by at most 30 %, decelerating at most a_shape over half the distance
    r = 1 - sqrt(max(0, 1 - rsu.a_shape * d_o(m) / vo(m)^2));
    r = min(r, 1 - rsu.v_frac);
    cap = min(rsu.dmax, 0.5 * d_o(m) / vo(m) * (1 / (1 - r) - 1));
    hi(m) = max(lo(m), nat(m) + cap);
end
for m = 2:M                                    % keep the order in the lane
    if use(m) && use(m-1) && ~fixed(m), lo(m) = max(lo(m), lo(m-1) + rsu.h_min); end
end
hi = max(hi, lo);
% which oncoming vehicles may be held (at least until their natural time)
holdable = ctl & cav_o & ~fixed & use & movo ~= 'R' & ~strcmp(cfg.slot, 'strict');

% ---- slot search: the turner before oncoming vehicle p, p = 1..M+1 ----
best = inf; bp = 0; btt = inf; cand = zeros(0, 3);
for p = 1:M+1
    tt = tbar_t;
    for m = 1:min(p-1, M)
        if cons(m), tt = max(tt, lo(m) + Db(m)); end
    end
    tau = lo; ok = true; prev = -inf; prevHeld = false; held = false(1, M);
    for m = 1:M
        if ~use(m), continue; end
        if m < p, prev = tau(m); continue; end
        pushed = max(lo(m), prev + rsu.h_min);        % car following behind a slowed car
        if ~cons(m) || lo(m) >= tt + Da(m) - 0.01
            tau(m) = pushed;                          % no conflict, or the gap is there anyway
        elseif prevHeld && pushed >= tt + DaC(m) - 0.01
            tau(m) = pushed;                          % behind a held vehicle: cannot be early
        elseif holdable(m) && max(lo(m), tt + DaC(m)) <= hi(m) + 0.01
            tau(m) = max([lo(m), tt + DaC(m), pushed]);
            held(m) = true;                           % commanded: held until tau
        elseif pushed >= tt + Da(m) - 0.01
            tau(m) = pushed;
        else
            ok = false; break;                        % would need more than allowed
        end
        if fixed(m) && tau(m) > lo(m) + 0.01, ok = false; break; end
        prevHeld = held(m) || (prevHeld && tau(m) <= prev + rsu.h_min + 0.01);
        prev = tau(m);
    end
    if ~ok, continue; end
    tf = tbar_f; pv = tt;
    for f = 1:numel(tf), tf(f) = max(tf(f), pv + rsu.h_min); pv = tf(f); end
    J = rsu.w_onc * sum((tau(use) - nat(use)).^2) + (tt - tbar_t)^2 + sum((tf - tbar_f).^2);
    cand(end+1, :) = [p, tt, J]; %#ok<AGROW>
    keep = false;
    switch cfg.slot
        case {'opt', 'strict'}, keep = J < best;
        case 'greedy',          keep = tt < btt - 1e-9 || (abs(tt - btt) < 1e-9 && J < best);
        case 'fcfs'
            p0 = 1 + sum(nat(use) < tbar_t);          % arrival order
            keep = p >= p0 && bp == 0;                % first feasible slot from there
    end
    if keep, best = J; bp = p; btau = tau; btt = tt; bheld = held; end
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
    if fixed(m) || ~use(m) || ~cav_o(m) || movo(m) == 'R', continue; end   % never the other turner
    k = find(rsu.shp.id == ids_o(m), 1);
    held = ~isempty(k) && t < rsu.shp.tgt(k);         % keep repeating a given target
    % a command only within the vehicle's own limit; a vehicle pushed further
    % by the slowed vehicle ahead of it just follows it (no command)
    btau(m) = min(btau(m), hi(m));
    if bheld(m) || btau(m) - lo(m) >= 0.3 || held
        if isempty(k)
            rsu.shp.id(end+1) = ids_o(m); rsu.shp.nat(end+1) = t + nat(m);
            rsu.shp.tgt(end+1) = t + btau(m); rsu.shp.t0(end+1) = t;
        else
            rsu.shp.tgt(k) = max(rsu.shp.tgt(k), min(t + btau(m), rsu.shp.nat(k) + rsu.dmax));
        end
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
