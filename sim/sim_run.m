function R = sim_run(cfgName, S, opts)
%SIM_RUN  One simulation run of one configuration (the single engine).
%
%   R = sim_run(cfgName, S)          S from abl_settings()
%   R = sim_run(cfgName, S, opts)    opts.live (animation), opts.collect
%                                    (estimator training snapshots),
%                                    opts.est_eval (log all estimators)
%
%   Every configuration uses this loop; sim_config only switches the
%   controller, the coordination, the estimator and the RSU message.
%
%   Step order (dt = 0.5 s, t = KK*dt as in the earlier scripts)
%     1  signal states
%     2  scheduled arrivals join the entry queue of their approach
%     3  the first vehicle of each entry queue enters if the entry is free
%     4  vehicle ahead of every vehicle (find_leaders)
%     5  V2X uplink, RSU planning and messages, downlink (rsu_* functions)
%     6  right-turn decisions and stop requirements
%     7  acceleration of every vehicle (IDM or MPC)
%     8  integration, no-overlap guard, fuel, idle, jerk, crossings, exits
%     9  log (and live animation)
%
%   R.veh   one row per vehicle that arrived (entered or still waiting)
%   R.log   [t id s v a path isCAV] for every vehicle and step
%   R.G, R.cfg, R.S, R.count (guards, QP solves/failures, ...), R.est

if nargin < 3, opts = struct(); end
if ~isfield(opts, 'live'), opts.live = false; end
if ~isfield(opts, 'collect'), opts.collect = false; end
if ~isfield(opts, 'est_eval'), opts.est_eval = false; end
if ~isfield(opts, 'frames'), opts.frames = []; end        % times to save a frame
if ~isfield(opts, 'frame_dir'), opts.frame_dir = '.'; end
if ~isfield(opts, 'view'), opts.view = 60; end
if ~isfield(opts, 'debug_turn'), opts.debug_turn = []; end   % [t0 t1]: print gap checks

cfg = sim_config(cfgName);
persistent Gc
if isempty(Gc), Gc = geom_build(); end
G = Gc; C = G.C; dt = C.DT;
KKmax = S.KKmax;

% ---------------- traffic ----------------
rng(S.seed);
sched = make_traffic_schedule(S, KKmax, dt, [0, 120, 300], [0.30, 0.45, 0.60], [0.25, 0.40, 0.55]);
apps = {'E', 'W'};                         % east-west corridor (as in the paper runs)
noise = rand_stream(3e5 + S.seed, 1, KKmax * 120, 'randn');  ni = 0;
chanU = rand_stream(1e6 + S.seed, 1, KKmax * 240);           ci = 0;

% ---------------- state ----------------
V = vehTemplate(); V(1) = [];
DONE = V;
Q = struct('E', V, 'W', V);                % entry queues
nextId = struct('E', 3000, 'W', 4000);
count = struct('guard', 0, 'stopguard', 0, 'qp', 0, 'qpfail', 0, 'blocked', 0, ...
               'msg_sent', 0, 'msg_lost', 0, 'guard_log', zeros(0, 4));

% one straight vehicle per approach at t = 0 (as in the earlier scripts)
for a = 1:numel(apps)
    ap = apps{a};
    v0 = vehTemplate();
    v0.id = nextId.(ap); nextId.(ap) = nextId.(ap) + 1;
    v0.app = ap; v0.mov = 'S'; v0.p = G.idx.([ap 'S']);
    v0.v = 20; v0.isCAV = ~strcmp(cfg.coord, 'none') && S.pen > 0;
    v0.t_arr = 0; v0.t_spawn = 0;
    V(end+1) = v0; %#ok<AGROW>
end

% RSU
rsu = [];
if strcmp(cfg.coord, 'rsu') || opts.est_eval || opts.collect
    rsu = rsu_init(cfg, G, S, opts);
end
chan = struct('lat', max(S.lat, 0), 'loss', min(max(S.loss, 0), 1), 'sense', S.sense);
down = struct('id', {}, 'm', {}, 't_del', {});   % messages in transit

LOG = zeros(KKmax * 80, 7); nlog = 0;
hfig = [];

for KK = 1:KKmax
    t = KK * dt;
    sig = struct(); sigT = struct();
    for a = 1:numel(apps)
        [st, tg0, tend] = signal_info(t, apps{a}, C);
        sig.(apps{a}) = st; sigT.(apps{a}) = [tg0, tend];
    end

    % ---- 2. arrivals ----
    for a = 1:numel(apps)
        ap = apps{a};
        if ~sched.([ap '_intent'])(KK), continue; end
        nv = vehTemplate();
        nv.id = nextId.(ap); nextId.(ap) = nextId.(ap) + 1;
        nv.app = ap;
        if sched.([ap '_TurnLeft'])(KK), nv.mov = 'L';
        elseif sched.([ap '_TurnRight'])(KK), nv.mov = 'R';
        else, nv.mov = 'S'; end
        nv.p = G.idx.([ap nv.mov]);
        nv.Vd = sched.([ap '_Vd'])(KK);  nv.T = sched.([ap '_Th'])(KK);
        nv.amax = sched.([ap '_A'])(KK); nv.bc = sched.([ap '_B'])(KK);
        nv.s0 = sched.([ap '_S0'])(KK);  nv.hm = sched.([ap '_HM'])(KK);
        nv.isCAV = ~strcmp(cfg.coord, 'none') && sched.([ap '_U'])(KK) < S.pen;
        nv.t_arr = t;
        Q.(ap)(end+1) = nv;
    end

    % ---- 3. entry ----
    for a = 1:numel(apps)
        ap = apps{a};
        if isempty(Q.(ap)), continue; end
        inLane = arrayfun(@(x) x.app == ap, V);
        sm = min([V(inLane).s, inf]);
        if sm >= C.SPAWN_GAP
            nv = Q.(ap)(1); Q.(ap)(1) = [];
            nv.t_spawn = t;
            % enter at the desired speed, or the speed of a close vehicle ahead
            if isfinite(sm) && sm < 60
                k = find(inLane & [V.s] == sm, 1);
                nv.v = min(nv.Vd, V(k).v);
            else
                nv.v = nv.Vd;
            end
            if nv.t_spawn > nv.t_arr + 1e-9, count.blocked = count.blocked + 1; end
            % idling in the queue before the entry costs fuel too
            nv.fuel = fuel_rate(0, 0) * (nv.t_spawn - nv.t_arr);
            V(end+1) = nv; %#ok<AGROW>
        end
    end
    n = numel(V);
    if n == 0
        continue;
    end

    % ---- 4. leaders ----
    [gap, vl, ~, al] = find_leaders(V, G);

    % ---- 5. V2X and RSU ----
    if ~isempty(rsu)
        % uplink: the RSU keeps the last state it received from each vehicle
        for i = 1:n
            if ~V(i).isCAV && ~strcmp(chan.sense, 'all') && ~opts.est_eval && ~opts.collect
                continue;                          % human driver: no V2X
            end
            ci = ci + 1;
            if chan.loss > 0 && chanU(ci) < chan.loss, continue; end
            rsu = rsu_receive(rsu, V(i), t, chan.lat, G);
        end
        rsu = rsu_forget(rsu, V, DONE, t);
        [rsu, msgs] = rsu_step(rsu, t, sig, sigT, G, cfg);
        % downlink with loss and latency
        for m = 1:numel(msgs)
            count.msg_sent = count.msg_sent + 1;
            ci = ci + 1;
            if chan.loss > 0 && chanU(ci) < chan.loss
                count.msg_lost = count.msg_lost + 1; continue;
            end
            down(end+1) = struct('id', msgs(m).id, 'm', msgs(m), ...
                                 't_del', t + ceil(chan.lat / dt - 1e-9) * dt); %#ok<AGROW>
        end
        if ~isempty(down)
            dl = [down.t_del] <= t + 1e-9;
            ids = [V.id];
            for d = find(dl)
                i = find(ids == down(d).id, 1);
                if ~isempty(i) && V(i).isCAV
                    V(i).msg = down(d).m; V(i).msg_rx = t;
                end
            end
            down = down(~dl);
        end
    end

    % ---- 6. decisions and stop points ----
    for i = 1:n, V(i).t_now = t; end
    sStop = inf(1, n);
    for i = 1:n
        P = G.P(V(i).p);
        if V(i).s < P.s_stop + 0.05            % (a vehicle held exactly at the line counts)
            st = sig.(V(i).app);
            if strcmp(st, 'red') || (strcmp(st, 'yellow') && ...
                    (P.s_stop - V(i).s) >= V(i).v^2 / (2 * 3.0))
                sStop(i) = P.s_stop;
            end
        end
        if V(i).mov == 'R' && ~V(i).committed && V(i).s >= P.s_stop - 40
            if V(i).isCAV
                [ok, why] = turn_gap_ok(i, V, G, sig, 'cav', 1.5);
            else
                [ok, why] = turn_gap_ok(i, V, G, sig, 'human', 0.7 + V(i).hm);
            end
            if ~isempty(opts.debug_turn) && t >= opts.debug_turn(1) && t <= opts.debug_turn(2) && ~isempty(why)
                fprintf('t %.1f turner %d s %.1f: blocked by %d t_arr %.2f < %.2f (s %.1f v %.1f a %.2f)\n', ...
                        t, V(i).id, V(i).s, why);
            end
            V(i).go = ok;
            if ok && V(i).s >= P.s_wait - 0.3, V(i).committed = true; end
            if ~V(i).committed && V(i).s > P.s_wait + 0.05
                V(i).committed = true;             % could not stop in time
            end
            if ~V(i).committed && ~ok
                sStop(i) = min(sStop(i), P.s_wait);
            end
        end
    end

    % yielding: a straight or left-turning vehicle that has not reached its
    % conflict zone with an opposing right-turner already turning (committed
    % and not yet clear) stops before the zone if it can do so comfortably
    % (defensive driving; priority is not given away, a collision is avoided)
    isR = [V.mov] == 'R';
    if any(isR)
        for i = find(~isR)
            rowsI = oppTurnRows(G, V(i).p);
            if isempty(rowsI), continue; end
            for r = 1:size(rowsI, 1)
                kR = rowsI(r, 1); z0i = rowsI(r, 2); z1k = rowsI(r, 5);
                if V(i).s >= z0i, continue; end
                busy = false;
                for u = find(isR & [V.p] == kR)
                    if V(u).committed && V(u).s - C.VEH_L < z1k && V(u).s > G.P(kR).s_wait - 0.5
                        busy = true; break;
                    end
                end
                if busy && (z0i - V(i).s) >= V(i).v^2 / (2 * 4.0)
                    sStop(i) = min(sStop(i), z0i - 0.5);
                end
            end
        end
    end

    % ---- 7. control ----
    acc = zeros(1, n);
    for i = 1:n
        P = G.P(V(i).p);
        gs = sStop(i) - V(i).s;                   % distance to the stop point
        g  = gap(i); vlead = vl(i);
        if gs < g, g = gs; vlead = 0; end
        useMPC = strcmp(cfg.ctrl, 'mpc') && V(i).isCAV;
        if useMPC
            [ai, V(i), info] = ctrl_mpc(V(i), gap(i), vl(i), sStop(i), G, cfg, t, sigT.(V(i).app), sig.(V(i).app), al(i));
            count.qp = count.qp + info.solves; count.qpfail = count.qpfail + info.fail;
        else
            v0 = V(i).Vd;
            if V(i).isCAV && strcmp(cfg.coord, 'rsu') && fresh(V(i), t) && ...
                    isfield(V(i).msg, 'vadv') && isfinite(V(i).msg.vadv)
                dz = P.s_stop - V(i).s;
                if dz > 0 && dz <= 80
                    v0 = min(v0, max(V(i).msg.vadv, 0.5 * V(i).Vd));
                end
            end
            v0 = min(v0, P.vcurve(min(numel(P.s), floor(V(i).s / C.DS) + 1)));
            prm = struct('a', V(i).amax, 'b', V(i).bc, 's0', V(i).s0, 'T', V(i).T);
            if V(i).isCAV
                prm = struct('a', 1.5, 'b', 2.5, 's0', 2.0, 'T', V(i).T);
            end
            ai = ctrl_idm(V(i).v, v0, g, vlead, prm);
            ai = min(ai, curve_cap(P, V(i).s, V(i).v, prm.b));
            if ~V(i).isCAV
                k1 = mod(ni, numel(noise)) + 1; k2 = mod(ni + 1, numel(noise)) + 1;
                ai = ai * (1 + noise(k1) / 30) + 0.05 * noise(k2);
                ni = ni + 2;
            end
        end
        acc(i) = ai;
    end

    % ---- 8. integration ----
    s_old = [V.s]; v_old = [V.v];
    v_new = max(v_old + acc * dt, 0);
    ds = zeros(1, n);
    for i = 1:n
        if v_old(i) + acc(i) * dt < 0 && acc(i) < 0
            ds(i) = v_old(i)^2 / (2 * -acc(i));
        else
            ds(i) = 0.5 * (v_old(i) + v_new(i)) * dt;
        end
    end
    s_new = s_old + ds;
    % no-overlap guard (same for every configuration) and stop guard
    for rep = 1:2
        [~, ~, lead] = find_leadersAt(V, G, s_new);
        for i = 1:n
            j = lead(i);
            if j > 0
                wi = posOnShared(V, G, i, j, s_new);
                if wi.gap < 0.3
                    s_new(i) = max(s_old(i), s_new(i) - (0.3 - wi.gap));
                    v_new(i) = min(v_new(i), v_new(j));
                    count.guard = count.guard + 1;
                    count.guard_log(end+1, :) = [t, V(i).id, V(j).id, wi.gap];
                end
            end
        end
    end
    for i = 1:n
        if s_new(i) > sStop(i) && s_old(i) <= sStop(i) + 1e-6
            s_new(i) = max(s_old(i), sStop(i) - 0.05); v_new(i) = 0;
            count.stopguard = count.stopguard + 1;
        end
    end
    a_act = (v_new - v_old) / dt;

    keep = true(1, n);
    for i = 1:n
        P = G.P(V(i).p);
        V(i).s = s_new(i); V(i).v = v_new(i);
        V(i).jerk2 = V(i).jerk2 + ((a_act(i) - V(i).a) / dt)^2;
        V(i).a = a_act(i); V(i).nstep = V(i).nstep + 1;
        V(i).fuel = V(i).fuel + fuel_rate(v_new(i), a_act(i)) * dt;
        V(i).dist = V(i).dist + ds(i);
        if v_new(i) < 0.5
            if ~V(i).stopped, V(i).nstops = V(i).nstops + 1; end
            V(i).stopped = true;
            if V(i).mov == 'R' && V(i).s >= P.s_stop && ~V(i).committed
                V(i).idle_gap = V(i).idle_gap + dt;
            elseif V(i).s < P.s_stop && ~strcmp(sig.(V(i).app), 'green')
                V(i).idle_red = V(i).idle_red + dt;
            else
                V(i).idle_other = V(i).idle_other + dt;
            end
        elseif v_new(i) > 1.0
            V(i).stopped = false;
        end
        if isnan(V(i).t_stop) && s_old(i) < P.s_stop && s_new(i) >= P.s_stop
            V(i).t_stop = t - dt + dt * (P.s_stop - s_old(i)) / max(s_new(i) - s_old(i), 1e-9);
            if ~isempty(rsu), rsu = rsu_crossed(rsu, V(i), V(i).t_stop, cfg); end
        end
        if s_new(i) >= P.len
            V(i).t_exit = t - dt + dt * (P.len - s_old(i)) / max(s_new(i) - s_old(i), 1e-9);
            V(i).exited = true;
            keep(i) = false;
        end
    end

    % ---- 9. log ----
    if nlog + n > size(LOG, 1), LOG = [LOG; zeros(size(LOG))]; end %#ok<AGROW>
    LOG(nlog+1:nlog+n, :) = [t * ones(n, 1), [V.id]', [V.s]', [V.v]', [V.a]', [V.p]', [V.isCAV]'];
    nlog = nlog + n;
    if opts.live
        hfig = draw_intersection(V, G, t, sig, rsu, hfig, opts.view);
    end
    if any(abs(opts.frames - t) < 1e-9)
        hfig = draw_intersection(V, G, t, sig, rsu, hfig, opts.view);
        print(hfig, fullfile(opts.frame_dir, sprintf('frame_%s_%05.1f.png', cfg.name, t)), '-dpng', '-r90');
    end

    DONE = [DONE, V(~keep)]; %#ok<AGROW>
    V = V(keep);
end

% vehicles still in the model or still waiting at the entry
VQ = [Q.E, Q.W];
for k = 1:numel(VQ)
    VQ(k).entered = false;
    VQ(k).fuel = fuel_rate(0, 0) * (KKmax * dt - VQ(k).t_arr);
end
for k = 1:numel(V), V(k).entered = true; end
for k = 1:numel(DONE), DONE(k).entered = true; end
R.veh = [DONE, V, VQ];
R.log = LOG(1:nlog, :);
R.G = G; R.cfg = cfg; R.S = S; R.count = count; R.t_end = KKmax * dt;
R.est = [];
if ~isempty(rsu), R.est = rsu_results(rsu); end
end

% ======================================================================
function v = vehTemplate()
v = struct('id', 0, 'app', 'E', 'mov', 'S', 'p', 1, 's', 0, 'v', 0, 'a', 0, ...
    'Vd', 25, 'T', 1.3, 'amax', 1.5, 'bc', 2.5, 's0', 2.0, 'hm', 2.0, 'isCAV', false, ...
    't_arr', 0, 't_spawn', NaN, 't_stop', NaN, 't_exit', NaN, 'exited', false, 'entered', true, ...
    'fuel', 0, 'dist', 0, 'idle_gap', 0, 'idle_red', 0, 'idle_other', 0, 'jerk2', 0, 'nstep', 0, ...
    'nstops', 0, 'stopped', false, 'committed', false, 'go', false, ...
    'msg', struct(), 'msg_rx', -inf, 'U', [], 'planS', [], 't_now', 0);
end

function rows = oppTurnRows(G, p)
% conflict rows of path p with the opposing right turn:
% [turn path, z0 on p, z1 on p, z0 on turn, z1 on turn]
persistent cache
if isempty(cache), cache = cell(1, numel(G.P)); end
if ~isempty(cache{p}) || G.P(p).mov == 'R'
    rows = cache{p}; return;
end
rows = zeros(0, 5);
for k = 1:numel(G.P)
    if G.P(k).mov ~= 'R', continue; end
    R = G.oppZ{k};
    for r = 1:size(R, 1)
        if R(r, 1) == p, rows(end+1, :) = [k, R(r, 4), R(r, 5), R(r, 2), R(r, 3)]; end %#ok<AGROW>
    end
end
if isempty(rows), rows = zeros(0, 5); end
cache{p} = rows;
end

function f = fresh(veh, t)
f = (t - veh.msg_rx) <= 1.0 + 1e-9 && ~isempty(fieldnames(veh.msg));
end

function [gap, vl, lead] = find_leadersAt(V, G, s_new)
for i = 1:numel(V), V(i).s = s_new(i); end
[gap, vl, lead] = find_leaders(V, G);
end

function o = posOnShared(V, G, i, j, s_new)
% gap between i and its leader j at the new positions
L = G.C.VEH_L;
Pi = G.P(V(i).p); Pj = G.P(V(j).p);
if V(i).app == V(j).app && s_new(i) <= Pi.s_div && s_new(j) - L <= min(Pi.s_div, Pj.s_div)
    o.gap = s_new(j) - L - s_new(i);
elseif ~isnan(G.MZ(V(i).p, V(j).p)) && (s_new(i) < G.MZ(V(i).p, V(j).p) + 8 || ...
        s_new(j) < G.MZ(V(j).p, V(i).p) + 8)
    if s_new(i) < G.MZ(V(i).p, V(j).p)
        o.gap = inf;          % not yet in the merge zone: no physical overlap
    else
        o.gap = (s_new(j) - G.MZ(V(j).p, V(i).p)) - L - (s_new(i) - G.MZ(V(i).p, V(j).p));
    end
else
    wi = s_new(i) - Pi.s_out + Pi.w_out;
    wj = s_new(j) - Pj.s_out + Pj.w_out;
    o.gap = wj - L - wi;
end
end
