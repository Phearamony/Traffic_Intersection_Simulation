function S = abl_settings()
%ABL_SETTINGS  Run settings shared by every run (sim/sim_main.m).
%
%   Values come from environment variables so that the runners
%   (run_ablation, run_sweeps, run_scenarios, make_figures, watch_main) can
%   loop over seeds and settings. When a variable is not set, the default
%   below is used. abl_env_reset() clears all of them.
%
%   --- run control ---
%   ABL_SEED        random seed of traffic and driver noise          (default 1)
%   ABL_KKMAX       number of 0.5 s steps per run                    (default 720 = 360 s)
%   ABL_PLOT        1 = save the per-run figures (trajectories, fuel) (default 1)
%   ABL_LIVE        1 = live intersection animation while running    (default 1)
%   ABL_NOLOG       1 = do not append rows to the results CSVs       (default 0)
%   ABL_OUTDIR      folder for figures and saved runs   (default output/single_runs)
%   ABL_CSV         results CSV       (default output/single_runs/single_runs.csv)
%   ABL_EST_CSV     estimator CSV  (default output/single_runs/estimator_runs.csv)
%                   (the runners set these to their own folder, see out_folder.m)
%   ABL_STUDY, ABL_POINT   labels written into every results row     (default '')
%   ABL_SAVELOG     1 = also save the run (log, vehicles) to ABL_OUTDIR (default 0)
%
%   --- traffic (read by make_traffic_schedule) ---
%   ABL_SCHED       'staged' (light/medium/heavy, default), 'const',
%                   'scenarioA', 'scenarioB'
%   ABL_DEMAND      constant east-west axis flow [veh/s] for 'const'   (default 0.40)
%   ABL_RT          P(right turn | not left turn)                       (default 0.15)
%   ABL_ASYM        share of east-west flow on the westbound approach   (default 0.5)
%   ABL_DRIVER      'nominal' (default), 'aggressive', 'cautious'
%
%   --- V2X and fleet ---
%   ABL_LAT         one-way V2X latency [s]; the RSU extrapolates a received
%                   state at its speed, and downlink messages arrive after the
%                   latency rounded up to whole 0.5 s steps          (default 0)
%   ABL_LOSS        packet loss probability per message and direction   (default 0)
%   ABL_PEN         CAV penetration rate, 0 to 1                        (default 1)
%   ABL_DMAX        largest extra delay the RSU may ask of one oncoming
%                   CAV to make a gap [s]                               (default 2)
%   ABL_SENSE       what the RSU knows about human drivers in mixed traffic:
%                   'v2x' (default) = nothing, human cars have no V2X and are
%                   invisible to the RSU; 'all' = RSU also senses them
%                   (roadside camera or radar), but cannot instruct them

S.seed     = readNum('ABL_SEED',  1);
S.KKmax    = readNum('ABL_KKMAX', 720);
S.plot     = logical(readNum('ABL_PLOT', 1));
S.live     = logical(readNum('ABL_LIVE', 1));
S.nolog    = logical(readNum('ABL_NOLOG', 0));
S.outdir   = readStr('ABL_OUTDIR', out_folder('single', ''));
S.csv      = readStr('ABL_CSV', out_folder('single', 'single_runs.csv'));
S.estcsv   = readStr('ABL_EST_CSV', out_folder('single', 'estimator_runs.csv'));
S.study    = readStr('ABL_STUDY', '');
S.point    = readStr('ABL_POINT', '');
S.savelog  = logical(readNum('ABL_SAVELOG', 0));

S.sched    = lower(readStr('ABL_SCHED', 'staged'));
S.demand   = readNum('ABL_DEMAND', 0.40);
S.rt       = readNum('ABL_RT', 0.15);
S.asym     = readNum('ABL_ASYM', 0.5);
S.driver   = lower(readStr('ABL_DRIVER', 'nominal'));

S.lat      = readNum('ABL_LAT', 0);
S.loss     = readNum('ABL_LOSS', 0);
S.pen      = readNum('ABL_PEN', 1);
S.sense    = lower(readStr('ABL_SENSE', 'v2x'));
S.dmax     = readNum('ABL_DMAX', 2);
S.estfile  = '';
end

function v = readNum(name, default)
s = getenv(name);
if isempty(s)
    v = default;
else
    v = str2double(s);
    if isnan(v), v = default; end
end
end

function v = readStr(name, default)
v = getenv(name);
if isempty(v), v = default; end
end
