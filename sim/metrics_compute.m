function M = metrics_compute(R)
%METRICS_COMPUTE  Results of one run (one row of the results CSV).
%
%   Vehicle sets
%     arrived     every vehicle the schedule sent (also those still waiting
%                 at the entry at the end)
%     entered     those that entered the road
%     exited      those that left the model: averages of fuel, idle time,
%                 delay and stops are over these complete trips
%     rt          right-turners that exited;  str  straight vehicles that exited
%
%   Fields
%     n_rt_intent, n_rt_done, rt_completion   right-turners arrived / exited
%     n_exited, n_unfinished, n_waiting       exited / inside / still at the entry
%     fuel_rt_mL, fuel_str_mL                 mean fuel per complete trip [mL]
%     fuel_L100km                             fuel of every vehicle (also idling
%                                             in the entry queue) per distance
%                                             driven [L/100 km]
%     gap_rt_s, red_rt_s, red_str_s           mean idle time [s] (stopped, v < 0.5)
%     pet_min_s, pet_mean_s                   post-encroachment time of right turns
%     n_conflict                              right turns with PET < 1.5 s
%     n_collision                             right turns whose occupancy of a
%                                             conflict zone overlapped another
%                                             vehicle's (PET = 0)
%     delay_rt_s, delay_str_s, delay_all_s    trip time from ARRIVAL (entry
%                                             queue included) minus free-flow
%                                             time on the path at the
%                                             vehicle's desired speed
%     n_blocked, entry_wait_s                 arrivals that had to wait at the
%                                             entry (also those still waiting at
%                                             the end); mean wait of entered vehicles
%     jerk_rms                                RMS jerk over all entered vehicles [m/s^3]
%     stops_per_veh                           stops per complete trip
%     qp_solves, qp_fail_pct, guard           MPC solves / share without a
%                                             solution; no-overlap corrections
%     n_shaped, shape_mean_s, shape_max_s     oncoming CAVs the RSU asked to (by 0.3 s or more)
%                                             arrive later; extra delay asked
%                                             (last target minus natural arrival)
%     shape_decel                             strongest deceleration of a shaped
%                                             vehicle from its first command to
%                                             its stop line [m/s2, positive]
%
%   PET: for every right turn and every opposing vehicle sharing a conflict
%   zone with it, each vehicle occupies the zone from the moment its front
%   enters until its rear leaves (interpolated from the log). PET is the
%   time between one vehicle leaving and the other entering; 0 if the
%   occupancies overlap. Each turn gets the smallest PET over the opposing
%   vehicles (capped at 30 s).

G = R.G; C = G.C; L = C.VEH_L;
v = R.veh;
ent = [v.entered]; ex = [v.exited];
mv = [v.mov];
rt = ex & mv == 'R'; st = ex & mv == 'S';
sm = @(x) mean(x(isfinite(x)));

M.n_rt_intent = sum(mv == 'R');
M.n_rt_done = sum(rt);
M.rt_completion = M.n_rt_done / max(M.n_rt_intent, 1);
if M.n_rt_intent == 0, M.rt_completion = NaN; end
M.n_exited = sum(ex);
M.n_unfinished = sum(ent & ~ex);
M.n_waiting = sum(~ent);
M.fuel_rt_mL = sm([v(rt).fuel]);
M.fuel_str_mL = sm([v(st).fuel]);
M.fuel_L100km = 100 * sum([v.fuel]) / max(sum([v(ent).dist]), 1);
M.gap_rt_s = sm([v(rt).idle_gap]);
M.red_rt_s = sm([v(rt).idle_red]);
M.red_str_s = sm([v(st).idle_red]);

% ---- delay ----
tff = nan(1, numel(v));
for k = find(ex)
    P = G.P(v(k).p);
    tff(k) = sum(C.DS ./ min(v(k).Vd, P.vcurve));
end
dl = [v.t_exit] - [v.t_arr] - tff;
M.delay_rt_s = sm(dl(rt));
M.delay_str_s = sm(dl(st));
M.delay_all_s = sm(dl(ex));
M.n_blocked = R.count.blocked + sum(~ent);      % also those still waiting at the end
M.entry_wait_s = sm([v(ent).t_spawn] - [v(ent).t_arr]);
M.jerk_rms = sqrt(sum([v(ent).jerk2]) / max(sum([v(ent).nstep]), 1));
M.stops_per_veh = sm([v(ex).nstops]);
M.qp_solves = R.count.qp;
M.qp_fail_pct = NaN;
if R.count.qp > 0, M.qp_fail_pct = 100 * R.count.qpfail / R.count.qp; end
M.guard = R.count.guard;

% ---- PET of right turns ----
LOG = R.log;
pet = []; M.pet_detail = zeros(0, 7);   % [turner id, opposing id, gap, a0, a1, b0, b1]
ncol = 0;
if ~isempty(LOG)
    ids = LOG(:, 2);
    occ = containers.Map('KeyType', 'double', 'ValueType', 'any');
    turners = find(mv == 'R' & ent);
    for u = turners
        k = v(u).p;
        rows = G.oppZ{k};
        if isempty(rows), continue; end
        [tu, su] = trace(LOG, ids, v(u).id);
        best = inf; hit = false;
        for r = 1:size(rows, 1)
            [a0, a1] = occupancy(tu, su, rows(r, 2), rows(r, 3), L);
            if isnan(a0), continue; end
            hit = true;
            q = rows(r, 1);
            for j = find([v.p] == q & ent)
                key = v(j).id * 100 + r;
                if isKey(occ, key)
                    b = occ(key);
                else
                    [tj, sj] = trace(LOG, ids, v(j).id);
                    [b0, b1] = occupancy(tj, sj, rows(r, 4), rows(r, 5), L);
                    b = [b0, b1]; occ(key) = b;
                end
                if isnan(b(1)), continue; end
                if b(1) < a1 && a0 < b(2)
                    g = 0;                              % overlapping occupancy
                elseif b(1) >= a1
                    g = b(1) - a1;
                else
                    g = a0 - b(2);
                end
                best = min(best, g);
                if g < 3, M.pet_detail(end+1, :) = [v(u).id, v(j).id, g, a0, a1, b(1), b(2)]; end
            end
        end
        if hit
            pet(end+1) = min(best, 30); %#ok<AGROW>
            if best == 0, ncol = ncol + 1; end
        end
    end
end
M.pet_min_s = NaN; M.pet_mean_s = NaN; M.n_conflict = 0; M.n_collision = ncol;
if ~isempty(pet)
    M.pet_min_s = min(pet); M.pet_mean_s = mean(pet); M.n_conflict = sum(pet < 1.5);
end

% ---- gap shaping by the RSU ----
M.n_shaped = 0; M.shape_mean_s = NaN; M.shape_max_s = NaN; M.shape_decel = NaN;
if isfield(R, 'est') && isfield(R.est, 'shp') && ~isempty(R.est.shp.id)
    sh = R.est.shp;
    dly = sh.tgt - sh.nat;
    k = dly >= 0.3;                              % asked to arrive later (not only held)
    sh.id = sh.id(k); sh.t0 = sh.t0(k); dly = dly(k);
    M.n_shaped = numel(sh.id);
    if M.n_shaped > 0, M.shape_mean_s = mean(dly); M.shape_max_s = max(dly); end
    dec = 0;
    sStop = G.C.L_APP - G.C.STOP;
    for k = 1:numel(sh.id)
        m = LOG(:, 2) == sh.id(k) & LOG(:, 1) >= sh.t0(k) & LOG(:, 3) < sStop;
        if any(m), dec = max(dec, -min(LOG(m, 5))); end
    end
    M.shape_decel = dec;
end
end

function [t, s] = trace(LOG, ids, id)
m = ids == id;
t = LOG(m, 1); s = LOG(m, 3);
end

function [t0, t1] = occupancy(t, s, z0, z1, L)
% first time the front reaches z0 and the rear passes z1 (NaN if never)
t0 = cross(t, s, z0);
t1 = cross(t, s - L, z1);
if isnan(t0), t1 = NaN; return; end
if isnan(t1), t1 = t(end); end                % still inside at the end
end

function tc = cross(t, s, z)
tc = NaN;
k = find(s >= z, 1);
if isempty(k), return; end
if k == 1, tc = t(1); return; end
tc = t(k-1) + (t(k) - t(k-1)) * (z - s(k-1)) / max(s(k) - s(k-1), 1e-9);
end
