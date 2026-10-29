function sched = make_traffic_schedule(S, KKmax, dt, DemandStages, lambdaNS_axis, lambdaEW_axis)
%MAKE_TRAFFIC_SCHEDULE  Pre-drawn arrivals shared by every configuration.
%
%   sched = make_traffic_schedule(S, KKmax, dt, DemandStages, lambdaNS_axis, lambdaEW_axis)
%   S comes from abl_settings(). The global random stream must already be
%   seeded (rng(S.seed)); with the default settings the draws are exactly
%   those of the original main_human_driving.m.
%
%   Per step KK and approach d in {N,S,E,W}:
%     d_intent     an arrival is drawn (Bernoulli 1 - exp(-lambda*dt))
%     d_TurnLeft   P = 0.25
%     d_TurnRight  P = S.rt given not left (default 0.15, so 11.25% of arrivals)
%     d_Vd, d_Th   desired speed [m/s] and time headway [s], see DRIVERS
%     d_U          uniform number from a separate stream; a vehicle is a CAV
%                  when d_U < S.pen, so lower penetration rates use a subset
%                  of the CAVs of higher rates (nested fleets)
%
%   S.sched
%     'staged'     light / medium / heavy stages (DemandStages, lambda*_axis)
%     'const'      constant east-west axis flow S.demand for the whole run
%     'scenarioA'  fixed illustrative scenario, see SCENARIOS below
%     'scenarioB'  fixed illustrative scenario with two turners and a mutual turn
%   S.asym         share of the east-west flow on the westbound approach
%
%   DRIVERS (desired speed U[a,b] m/s, time headway U[c,d] s; IDM a, b, s0;
%   human gap margin hm)
%   (urban road, speed limit 50 km/h; CAVs never exceed the limit)
%     nominal      Vd U[11.1,14.4] (40-52 km/h), Th U[1.2,2.4], a U[1.0,2.0],
%                  b U[1.5,3.0], s0 U[1.5,2.5], hm U[1.5,3.0]   (training distribution)
%     aggressive   Vd U[12.5,15.3] (45-55 km/h), Th U[0.8,1.6], a U[1.5,2.5],
%                  b U[2.5,3.5], s0 U[1.0,2.0], hm U[0.8,2.0]
%     cautious     Vd U[9.7,12.5] (35-45 km/h), Th U[1.8,3.0], a U[0.7,1.4],
%                  b U[1.2,2.2], s0 U[2.0,3.0], hm U[2.5,4.0]
%   The extra driver parameters come from their own stream (seed
%   7e5 + seed), so arrivals, turns, Vd and Th are unchanged.
%
%   SCENARIOS (fixed before any result was seen; times in s from the start).
%   The east-west signal is green from 60 s to 114.5 s. Every vehicle has
%   Vd = 13 m/s (47 km/h) and Th = 1.5 s. Only the east-west corridor is used.
%   Every main script also places one straight vehicle on each approach at
%   t = 0 (default Vd and Th); in the scenarios these two wait at the red
%   and lead their queues when the green starts.
%     scenarioA  "gap too short": westbound right-turner at 44 s with two
%                straight followers at 46.5 and 49 s; eastbound through
%                platoon at 46, 48.5, 51, 53.5 s (2.5 s headways, shorter
%                than the turner needs), then a gap, then 66 and 68.5 s.
%     scenarioB  as A with a second westbound right-turner at 46.5 s (the
%                follower at 49 s stays straight), a longer opposing platoon
%                (46, 48.5, 51, 53.5, 56 s, then 70, 72.5 s) and an eastbound
%                right-turner at 58 s, which creates a mutual right turn.

dirs = {'N','S','E','W'};
sched = struct();
sched.KKmax = KKmax;
sched.dt = dt;
sched.DemandStages = DemandStages;
sched.lambdaNS_axis = lambdaNS_axis;
sched.lambdaEW_axis = lambdaEW_axis;
sched.mode = S.sched;  sched.demand = S.demand;  sched.rt = S.rt;
sched.asym = S.asym;   sched.driver = S.driver;  sched.seed = S.seed;

for di = 1:4
    d = dirs{di};
    sched.([d '_intent'])    = false(1, KKmax);
    sched.([d '_TurnLeft'])  = false(1, KKmax);
    sched.([d '_TurnRight']) = false(1, KKmax);
    sched.([d '_Vd'])        = zeros(1, KKmax);
    sched.([d '_Th'])        = zeros(1, KKmax);
    sched.([d '_U'])         = zeros(1, KKmax);
    sched.([d '_A'])         = 1.5 * ones(1, KKmax);
    sched.([d '_B'])         = 2.5 * ones(1, KKmax);
    sched.([d '_S0'])        = 2.0 * ones(1, KKmax);
    sched.([d '_HM'])        = 2.0 * ones(1, KKmax);
end

% ---------------- fixed scenarios ----------------
if startsWith(S.sched, 'scenario')
    switch lower(S.sched)          % abl_settings lower-cases ABL_SCHED
        case 'scenarioa'
            W_rt = 44;          W_st = [46.5 49];
            E_st = [46 48.5 51 53.5 66 68.5];  E_rt = [];
        case 'scenariob'
            W_rt = [44 46.5];   W_st = 49;
            E_st = [46 48.5 51 53.5 56 70 72.5];  E_rt = 58;
        otherwise
            error('make_traffic_schedule: unknown scenario %s', S.sched);
    end
    sched = addArrivals(sched, 'W', W_rt, true,  dt, KKmax);
    sched = addArrivals(sched, 'W', W_st, false, dt, KKmax);
    sched = addArrivals(sched, 'E', E_st, false, dt, KKmax);
    sched = addArrivals(sched, 'E', E_rt, true,  dt, KKmax);
    return;
end

% ---------------- random traffic ----------------
switch S.driver
    case 'nominal',    vd = [11.1 3.3]; th = [1.2 1.2];    % 40-52 km/h
    case 'aggressive', vd = [12.5 2.8]; th = [0.8 0.8];    % 45-55 km/h
    case 'cautious',   vd = [9.7 2.8];  th = [1.8 1.2];    % 35-45 km/h
    otherwise, error('make_traffic_schedule: unknown driver type %s', S.driver);
end

for kk = 1:KKmax
    tsec = kk * dt;
    if strcmp(S.sched, 'const')
        lamEW = S.demand;  lamNS = lambdaNS_axis(2);
    else
        if tsec < DemandStages(2), k = 1; elseif tsec < DemandStages(3), k = 2; else, k = 3; end
        lamEW = lambdaEW_axis(k);  lamNS = lambdaNS_axis(k);
    end
    lams = [lamNS/2, lamNS/2, (1 - S.asym) * lamEW, S.asym * lamEW];   % N S E W
    for di = 1:4
        d = dirs{di};
        p = 1 - exp(-lams(di) * dt);
        sched.([d '_intent'])(kk) = rand < p;
        tl = rand < 0.25;
        tr = false;
        if ~tl, tr = rand < S.rt; end
        sched.([d '_TurnLeft'])(kk)  = tl;
        sched.([d '_TurnRight'])(kk) = tr;
        sched.([d '_Vd'])(kk) = vd(1) + vd(2) * rand;
        sched.([d '_Th'])(kk) = th(1) + th(2) * rand;
    end
end

% CAV draws from their own stream, so they do not change the traffic above
U = rand_stream(5e5 + S.seed, 4, KKmax);
for di = 1:4
    sched.([dirs{di} '_U']) = U(di, :);
end

% individual driver parameters, own stream as well
switch S.driver
    case 'nominal',    ra = [1.0 2.0]; rb = [1.5 3.0]; rs0 = [1.5 2.5]; rhm = [1.5 3.0];
    case 'aggressive', ra = [1.5 2.5]; rb = [2.5 3.5]; rs0 = [1.0 2.0]; rhm = [0.8 2.0];
    case 'cautious',   ra = [0.7 1.4]; rb = [1.2 2.2]; rs0 = [2.0 3.0]; rhm = [2.5 4.0];
end
D = rand_stream(7e5 + S.seed, 16, KKmax);
for di = 1:4
    d = dirs{di};
    sched.([d '_A'])  = ra(1)  + diff(ra)  * D(4*di-3, :);
    sched.([d '_B'])  = rb(1)  + diff(rb)  * D(4*di-2, :);
    sched.([d '_S0']) = rs0(1) + diff(rs0) * D(4*di-1, :);
    sched.([d '_HM']) = rhm(1) + diff(rhm) * D(4*di,   :);
end
end

function sched = addArrivals(sched, d, times, isRight, dt, KKmax)
for t = times
    kk = round(t / dt);
    if kk < 1 || kk > KKmax, continue; end
    sched.([d '_intent'])(kk)    = true;
    sched.([d '_TurnLeft'])(kk)  = false;
    sched.([d '_TurnRight'])(kk) = isRight;
    sched.([d '_Vd'])(kk)        = 13;
    sched.([d '_Th'])(kk)        = 1.5;
    sched.([d '_U'])(kk)         = 0;      % every vehicle is a CAV
end
end
