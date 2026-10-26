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
%   DRIVERS (desired speed U[a,b] m/s, time headway U[c,d] s)
%     nominal      Vd U[23,29], Th U[1.2,2.4]   (GPR training distribution)
%     aggressive   Vd U[26,32], Th U[0.8,1.6]
%     cautious     Vd U[18,24], Th U[1.8,3.0]
%
%   SCENARIOS (fixed before any result was seen; times in s from the start).
%   The east-west signal is green from 60 s to 114.5 s. Every vehicle has
%   Vd = 25 m/s and Th = 1.5 s. Only the east-west corridor is used.
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
end

% ---------------- fixed scenarios ----------------
if startsWith(S.sched, 'scenario')
    switch S.sched
        case 'scenarioA'
            W_rt = 44;          W_st = [46.5 49];
            E_st = [46 48.5 51 53.5 66 68.5];  E_rt = [];
        case 'scenarioB'
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
    case 'nominal',    vd = [23 6]; th = [1.2 1.2];
    case 'aggressive', vd = [26 6]; th = [0.8 0.8];
    case 'cautious',   vd = [18 6]; th = [1.8 1.2];
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
rsU = RandStream('mt19937ar', 'Seed', 5e5 + S.seed);
for di = 1:4
    sched.([dirs{di} '_U']) = rand(rsU, 1, KKmax);
end
end

function sched = addArrivals(sched, d, times, isRight, dt, KKmax)
for t = times
    kk = round(t / dt);
    if kk < 1 || kk > KKmax, continue; end
    sched.([d '_intent'])(kk)    = true;
    sched.([d '_TurnLeft'])(kk)  = false;
    sched.([d '_TurnRight'])(kk) = isRight;
    sched.([d '_Vd'])(kk)        = 25;
    sched.([d '_Th'])(kk)        = 1.5;
    sched.([d '_U'])(kk)         = 0;      % every vehicle is a CAV
end
end
