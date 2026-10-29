function [rsu, msgs] = rsu_plan(rsu, W, t, sig, sigT, G, cfg)
%RSU_PLAN  Joint right-turn slot planning for both approaches.
%
%   The RSU plans the waiting right-turners of both east-west approaches
%   together (up to K = 2 per approach, the first ones in each lane).
%
%   Vehicles and times
%     Every vehicle the RSU knows has one planned time tau: the stop-line
%     time for straight and left-turning vehicles, the time it leaves the
%     waiting point for a planned right-turner. lo is its earliest time (the
%     prediction of rsu_estimate, or a target it has already been given),
%     nat its natural time (the prediction when it was first shaped).
%     Vehicles in one lane keep their order: tau_i >= tau_{i-1} + h_min, so a
%     vehicle behind a waiting turner cannot pass it (one lane per direction).
%
%   Slots
%     Each planned turner k of approach a gets a slot p_k in the stream of
%     the opposite approach o: it goes after the first p_k - 1 conflicting
%     vehicles (straight or left-turning; the opposing right-turner does not
%     conflict, left-hand traffic) and before the others:
%       tau_k >= tau_m + Db_km               m before the slot
%       tau_m >= tau_k + Da_km  (or DaC_km)  m after the slot
%     Db_km = dexit_m - dent_km + PET + z sqrt(sigma_k^2 + sigma_m^2)
%     Da_km = tclr_km - dent_m  + PET + z sqrt(sigma_k^2 + sigma_m^2)
%     dent / dexit: time from the stop line (oncoming) or waiting point
%     (turner) until the front enters / the rear leaves THIS conflict zone,
%     from the path geometry. A turner commanded with an arrival time
%     (arrival-time MPC) leaves at its go time, and an oncoming CAV held by
%     a target cannot be early: in their margins sigma is replaced by the
%     tracking accuracy s_ctl = 0.3 s (DaC). A held vehicle also makes the
%     vehicle behind it unable to be early.
%     Two turners of the same lane take their slots in lane order.
%     All constraints only push times later, so the times of a slot
%     assignment are the smallest fixed point (iterated until no change).
%
%   Gap shaping (Road Traffic Act art. 37: the turner yields; its on-board
%   check only lets it turn into a gap that exists)
%     An oncoming CAV may be asked to arrive later within
%       hi_m = nat_m + min(dmax, gentle_m)
%     gentle_m: the delay of slowing to no less than v_frac of its speed,
%     decelerating at most a_shape, over half of its distance d_m; no new
%     command once d_m < d_cmd or the vehicle is queued (v < 5 m/s). It may
%     also be held at its natural time (no delay). Human drivers and
%     vehicles past the stop line cannot be asked; a vehicle behind a slowed
%     one just follows it. A given target is never taken back.
%     An assignment is feasible when every vehicle that is later than lo is
%     a turner, is pushed by the vehicle ahead, or is a CAV within hi.
%
%   Cost and rules (cfg.slot)
%     J = sum over all known vehicles of both approaches (tau - nat)^2
%     'opt'     the feasible joint assignment with the smallest J
%               (exhaustive over the slots each turner could use alone)
%     'strict'  as 'opt' without shaping or holding (existing gaps only)
%     'greedy'  turners in order of arrival, each takes its earliest
%               feasible slot given those already planned
%     'fcfs'    turners in order of arrival, each takes the first feasible
%               slot at or after its own position in the arrival order
%
%   Messages (to CAVs only, every step while the plan stands)
%     turner      go = t + tau_k (leave the waiting point), unless it cannot
%                 clear its zone before the yellow ends (no slot this green)
%     oncoming    tgt = t + tau_m at its stop line (arrival-time MPC) and
%                 vadv = d_m / tau_m (speed advice), within hi_m

msgs = struct('id', {}, 'vadv', {}, 'tgt', {}, 'spt', {}, 'go', {});
if isempty(W.ids), return; end
C = G.C; L = C.VEH_L; sStop = C.L_APP - C.STOP; h = rsu.h_min;
apps = 'EW'; z = cfg.unc_z;
ctl = strcmp(cfg.msg, 'time') && strcmp(cfg.ctrl, 'mpc');
strict = strcmp(cfg.slot, 'strict');
s_ctl = 0.3; K = 2;

% ---------------- 1. lanes ----------------
Ln = cell(1, 2);
for ai = 1:2
    a = apps(ai); o = apps(3 - ai);
    kp = G.idx.([a 'R']); P = G.P(kp); vt = min(P.vcurve(P.kappa > 0));
    rowsO = G.oppZ{G.idx.([o 'R'])};          % zones of the OPPOSITE turner with this lane
    [F, tp, sd] = rsu_estimate(rsu, W, a, t, G, cfg.est);
    % vehicles past the stop line: not yet clear (non-R) or waiting turners
    past = find(W.app == a & W.s >= sStop);
    e = struct('id', [], 'mov', '', 'cav', false(1, 0), 'fixed', false(1, 0), 'd', [], ...
               'v', [], 's', [], 'lo', [], 'sd', [], 'turn', false(1, 0));
    for r = past
        if W.mov(r) == 'R'
            if W.committed(r) || W.s(r) > P.s_wait + 0.05, continue; end
            lo = travel_time(P.s_wait - W.s(r), W.v(r), 1.5, vt);
            e = addE(e, W.ids(r), 'R', W.cav(r), false, 0, W.v(r), W.s(r), lo, 0.3, true);
        else
            qz = rowFor(rowsO, G, a, W.mov(r));
            if isempty(qz) || W.s(r) - L > qz(5), continue; end
            lo = -(W.s(r) - sStop) / max(W.v(r), 1);
            e = addE(e, W.ids(r), W.mov(r), W.cav(r), true, 0, W.v(r), W.s(r), lo, 0, false);
        end
    end
    [~, o1] = sort(e.s, 'descend'); e = pick(e, o1);
    [~, ix] = ismember(F.ids, W.ids);
    for q = 1:numel(F.ids)
        if ~isfinite(tp(q)), continue; end
        isT = F.mov(q) == 'R' && ~W.committed(ix(q));
        lo = tp(q); if isT, lo = tp(q) + (P.s_wait - sStop) / vt; end
        e = addE(e, F.ids(q), F.mov(q), W.cav(ix(q)), false, F.d(q), F.v(q), W.s(ix(q)), lo, sd(q), isT);
    end
    n = numel(e.id);
    e.nat = e.lo; e.hi = e.lo; e.ps = zeros(1, n);
    for i = 1:n
        if e.fixed(i) || e.turn(i), continue; end
        k = find(rsu.shp.id == e.id(i), 1);
        if ~isempty(k)
            e.lo(i) = max(e.lo(i), rsu.shp.tgt(k) - t);
            e.nat(i) = rsu.shp.nat(k) - t;
            e.ps(i) = k;
        end
    end
    for i = 2:n                                    % lane order
        if ~e.fixed(i), e.lo(i) = max(e.lo(i), e.lo(i-1) + h); end
    end
    % natural time: behind the vehicles ahead (lane order), or the time
    % stored when it was first shaped
    e.nat(e.ps == 0) = e.lo(e.ps == 0);
    e.nat = min(e.nat, e.lo);
    e.hi = e.lo;
    for i = 1:n
        if e.fixed(i) || e.turn(i) || e.mov(i) == 'R' || ~e.cav(i) || strict, continue; end
        if e.d(i) < rsu.d_cmd || e.v(i) < 5, continue; end      % too close or queued
        r = 1 - sqrt(max(0, 1 - rsu.a_shape * e.d(i) / e.v(i)^2));
        r = min(r, 1 - rsu.v_frac);
        cap = min(rsu.dmax, 0.5 * e.d(i) / e.v(i) * (1 / (1 - r) - 1));
        e.hi(i) = max(e.lo(i), e.nat(i) + cap);
    end
    e.hold = ctl & e.cav & ~e.fixed & e.mov ~= 'R' & ~strict;   % may be held (arrival-time MPC)
    e.cmd = e.hold | e.hi > e.lo + 1e-6;                         % may be commanded at all
    e.onc = e.mov ~= 'R';                          % can conflict with the other turner
    Ln{ai} = e;
end

% ---------------- 2. planned turners and their margins ----------------
T = struct('ai', {}, 'i', {}, 'id', {}, 'cidx', {}, 'Db', {}, 'Da', {}, 'DaC', {}, ...
           'tclr', {}, 'tend', {}, 'v_t0', {}, 'sp', {});
for ai = 1:2
    a = apps(ai); o = apps(3 - ai); bi = 3 - ai;
    if ~strcmp(sig.(a), 'green'), continue; end
    kp = G.idx.([a 'R']); P = G.P(kp); vt = min(P.vcurve(P.kappa > 0));
    rows = G.oppZ{kp};
    if isempty(rows), continue; end
    e = Ln{ai}; f = Ln{bi};
    cand = find(e.turn & (e.s >= sStop - 120));
    cand = cand(1:min(K, numel(cand)));
    for i = cand
        sig_k = e.sd(i); if ctl, sig_k = s_ctl; end
        v_t0 = 0;
        if ctl && e.v(i) > 1 && e.s(i) < P.s_wait - 1, v_t0 = vt; end
        nb = numel(f.id);
        Db = nan(1, nb); Da = nan(1, nb); DaC = nan(1, nb);
        for m = 1:nb
            qz = rowFor(rows, G, o, f.mov(m));
            if isempty(qz), continue; end
            vc = max(f.v(m), 6);
            dent = (qz(4) - sStop) / vc;
            dexit = (qz(5) + L - sStop) / vc;
            dent_k = travel_time(qz(2) - P.s_wait, v_t0, 1.5, vt);
            tclr_k = travel_time(qz(3) + L - P.s_wait, v_t0, 1.5, vt);
            u = z * sqrt(sig_k^2 + f.sd(m)^2);
            Db(m) = dexit - dent_k + rsu.pet + u;
            Da(m) = tclr_k - dent + rsu.pet + u;
            DaC(m) = tclr_k - dent + rsu.pet + z * sqrt(sig_k^2 + s_ctl^2);
        end
        T(end+1) = struct('ai', ai, 'i', i, 'id', e.id(i), 'cidx', find(isfinite(Db)), ...
            'Db', Db, 'Da', Da, 'DaC', DaC, ...
            'tclr', travel_time(max(rows(:, 3)) + L - P.s_wait, v_t0, 1.5, vt), ...
            'tend', sigT.(a)(2) - t, 'v_t0', v_t0, 'sp', P.s_wait); %#ok<AGROW>
    end
end
nT = numel(T);
if nT == 0, return; end

% ---------------- 3. slot search ----------------
% slots each turner could use on its own (a joint assignment only adds delays)
S = cell(1, nT);
for k = 1:nT
    sl = zeros(1, nT);
    for p = 1:numel(T(k).cidx) + 1
        sl(k) = p;
        if evalSlots(Ln, T, sl, h), S{k} = [S{k}, p]; end
    end
    if isempty(S{k}), S{k} = numel(T(k).cidx) + 1; end
end
nat_k = arrayfun(@(x) Ln{x.ai}.nat(x.i), T);
best = []; bestJ = inf;
switch cfg.slot
    case {'opt', 'strict'}
        sizes = cellfun(@numel, S);
        idx = ones(1, nT);
        while true
            sl = arrayfun(@(k) S{k}(idx(k)), 1:nT);
            if laneOrderOk(T, sl)
                [ok, tau, held, J] = evalSlots(Ln, T, sl, h);
                if ok && J < bestJ, bestJ = J; best = struct('sl', sl, 'tau', {tau}, 'held', {held}); end
            end
            k = 1;
            while k <= nT && idx(k) == sizes(k), idx(k) = 1; k = k + 1; end
            if k > nT, break; end
            idx(k) = idx(k) + 1;
        end
    case {'greedy', 'fcfs'}
        sl = greedySlots(Ln, T, S, nat_k, h, cfg);
        [ok, tau, held, J] = evalSlots(Ln, T, sl, h);
        if ok, best = struct('sl', sl, 'tau', {tau}, 'held', {held}); bestJ = J; end
end
if isempty(best), return; end
rsu.plans = rsu.plans + 1;
% diagnostics: how often the joint optimizer chooses differently from the
% sequential greedy rule (same limits)
if strcmp(cfg.slot, 'opt')
    rsu.diag(1) = rsu.diag(1) + 1;                       % plans
    rsu.diag(2) = rsu.diag(2) + (nT >= 2);               % plans with two or more turners
    rsu.diag(3) = rsu.diag(3) + any(cellfun(@numel, S) > 1);   % a real choice
    cg = cfg; cg.slot = 'greedy';
    slg = greedySlots(Ln, T, S, nat_k, h, cg);
    rsu.diag(4) = rsu.diag(4) + any(slg ~= best.sl);     % different choice
end

% ---------------- 4. messages ----------------
for k = 1:nT
    e = Ln{T(k).ai}; i = T(k).i;
    tk = best.tau{T(k).ai}(i);
    if tk + T(k).tclr > T(k).tend
        rsu.noslot = rsu.noslot + 1;              % no slot in this green
        continue;
    end
    vadv = NaN; if tk > 0.5, vadv = (T(k).sp - e.s(i)) / tk; end
    msgs(end+1) = struct('id', T(k).id, 'vadv', vadv, 'tgt', t + tk, 'spt', T(k).sp, 'go', t + tk); %#ok<AGROW>
end
for ai = 1:2
    e = Ln{ai}; tau = best.tau{ai}; held = best.held{ai};
    for i = 1:numel(e.id)
        if ~e.cmd(i), continue; end
        k = e.ps(i);
        keep = k > 0 && t < rsu.shp.tgt(k);       % keep repeating a given target
        tm = min(tau(i), e.hi(i));
        if ~(held(i) || keep), continue; end
        if k == 0
            rsu.shp.id(end+1) = e.id(i); rsu.shp.nat(end+1) = t + e.nat(i);
            rsu.shp.tgt(end+1) = t + tm; rsu.shp.t0(end+1) = t;
        else
            rsu.shp.tgt(k) = max(rsu.shp.tgt(k), min(t + tm, rsu.shp.nat(k) + rsu.dmax));
            tm = max(tm, rsu.shp.tgt(k) - t);
        end
        tg = t + tm;
        if ~strcmp(cfg.msg, 'time'), tg = NaN; end   % speed advice only: no binding time
        msgs(end+1) = struct('id', e.id(i), 'vadv', e.d(i) / max(tm, 0.5), ...
                             'tgt', tg, 'spt', sStop, 'go', NaN); %#ok<AGROW>
    end
end
end

% ======================================================================
function sl = greedySlots(Ln, T, S, nat_k, h, cfg)
% turners in order of natural arrival; greedy: each its earliest feasible
% slot; fcfs: the first feasible slot at or after its arrival-order position
nT = numel(T);
[~, ord] = sort(nat_k);
sl = zeros(1, nT);
for k = ord
    opts = S{k};
    if strcmp(cfg.slot, 'fcfs')
        f = Ln{3 - T(k).ai};
        p0 = 1 + sum(f.nat(T(k).cidx) < nat_k(k));
        later = opts(opts >= p0);
        if ~isempty(later), opts = [later, fliplr(opts(opts < p0))]; else, opts = fliplr(opts); end
    end
    bt = inf; bp = opts(end);
    for p = opts
        sl(k) = p;
        if ~laneOrderOk(T, sl), continue; end
        [ok, tau] = evalSlots(Ln, T, sl, h);
        if ~ok, continue; end
        tk = tau{T(k).ai}(T(k).i);
        if strcmp(cfg.slot, 'fcfs'), bp = p; break; end
        if tk < bt - 1e-9, bt = tk; bp = p; end
    end
    sl(k) = bp;
end
end

function [ok, tau, held, J] = evalSlots(Ln, T, sl, h)
% times of a slot assignment (sl(k) = 0: turner k not planned)
tau = {Ln{1}.lo, Ln{2}.lo};
held = {false(size(Ln{1}.lo)), false(size(Ln{2}.lo))};
for it = 1:30
    changed = false;
    for ai = 1:2
        e = Ln{ai};
        for i = 2:numel(e.id)
            if e.fixed(i), continue; end
            nt = tau{ai}(i - 1) + h;
            if tau{ai}(i) < nt - 1e-9, tau{ai}(i) = nt; changed = true; end
        end
    end
    for k = 1:numel(T)
        p = sl(k);
        if p == 0, continue; end
        ai = T(k).ai; bi = 3 - ai; f = Ln{bi};
        tk = tau{ai}(T(k).i);
        c = T(k).cidx;
        for j = 1:min(p - 1, numel(c))
            tk = max(tk, tau{bi}(c(j)) + T(k).Db(c(j)));
        end
        if tk > tau{ai}(T(k).i) + 1e-9, tau{ai}(T(k).i) = tk; changed = true; end
        for j = p:numel(c)
            m = c(j);
            behindHeld = m > 1 && held{bi}(m - 1) && f.hold(m - 1) && tau{bi}(m) <= tau{bi}(m - 1) + h + 1e-6;
            if f.hold(m) || behindHeld, mg = T(k).DaC(m); else, mg = T(k).Da(m); end
            req = tk + mg;
            if tau{bi}(m) < req - 1e-9
                tau{bi}(m) = req; changed = true;
                if f.cmd(m), held{bi}(m) = true; end
            elseif f.hold(m) && f.lo(m) < tk + T(k).Da(m) - 1e-9
                held{bi}(m) = true;           % only safe because it is held
            end
        end
    end
    if ~changed, break; end
end
ok = ~changed;
J = 0;
for ai = 1:2
    e = Ln{ai}; x = tau{ai};
    for i = 1:numel(e.id)
        if x(i) <= e.lo(i) + 0.01, continue; end
        if e.fixed(i), ok = false; return; end
        if e.turn(i), continue; end
        if i > 1 && x(i) <= x(i - 1) + h + 0.01, continue; end     % pushed by the one ahead
        if e.cmd(i) && x(i) <= e.hi(i) + 0.01, continue; end     % commanded within its limit
        ok = false; return;
    end
    m = ~e.fixed;
    if strcmp(getenv('ABL_OBJ'), 'lin')               % diagnostic: linear total delay
        J = J + sum(x(m) - e.nat(m));
    else
        J = J + sum((x(m) - e.nat(m)).^2);
    end
end
end

function ok = laneOrderOk(T, sl)
% two turners of the same lane take their slots in lane order
ok = true;
for k = 1:numel(T)
    for q = 1:numel(T)
        if T(k).ai == T(q).ai && T(k).i < T(q).i && sl(k) > 0 && sl(q) > 0 && sl(q) < sl(k)
            ok = false; return;
        end
    end
end
end

function e = addE(e, id, mov, cav, fixed, d, v, s, lo, sd, turn)
e.id(end+1) = id; e.mov(end+1) = mov; e.cav(end+1) = cav; e.fixed(end+1) = fixed;
e.d(end+1) = d; e.v(end+1) = v; e.s(end+1) = s; e.lo(end+1) = lo; e.sd(end+1) = sd;
e.turn(end+1) = turn;
end

function e = pick(e, k)
f = fieldnames(e);
for j = 1:numel(f), e.(f{j}) = e.(f{j})(k); end
end

function r = rowFor(rows, G, o, mov)
% conflict row of a turner with a vehicle of approach o and movement mov
r = [];
if mov == 'R' || isempty(rows), return; end
q = G.idx.([o mov]);
k = find(rows(:, 1) == q, 1);
if ~isempty(k), r = rows(k, :); end
end
