function S = abl_settings()
%ABL_SETTINGS  Run settings shared by every main_*.m script.
%
%   Values come from environment variables so that the runners
%   (run_ablation, run_sweeps, run_scenarios, make_figures) can loop over
%   seeds and settings even though each main script starts with "clear".
%   When a variable is not set, the default below is used, so running a
%   main script by hand (F5) behaves as before.
%   abl_env_reset() clears all of them.
%
%   --- run control ---
%   ABL_SEED        random seed for traffic schedule and driver noise   (default 1)
%   ABL_KKMAX       number of 0.5 s steps per run                       (default 720 = 360 s)
%   ABL_PLOT        1 = save the final figures (trajectories, fuel)     (default 1)
%   ABL_LIVE        1 = live intersection animation while running       (default 1)
%                   (independent of ABL_PLOT; watch_main uses LIVE=1, PLOT=0)
%   ABL_NOLOG       1 = do not append rows to the results CSVs          (default 0)
%   ABL_BATCH       1 = called from a runner (skip clear/close)         (default 0)
%   ABL_SCHED_ONLY  1 = main_human_driving only writes the schedule     (default 0)
%   ABL_OUTDIR      folder for figures                                  (default output/)
%   ABL_CSV         results CSV          (default output/ablation_runs.csv)
%   ABL_EST_CSV     estimator CSV        (default output/estimator_runs.csv)
%   ABL_STUDY, ABL_POINT   labels written into every results row        (default '')
%   ABL_SAVELOG     1 = also save CarLog of each run to ABL_OUTDIR      (default 0)
%   ABL_SCHED_PATH  traffic schedule file written by main_human_driving and
%                   read by every other main    (default simplot/traffic_schedule.mat)
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
%   ABL_VREC        1 = CAVs follow the RSU recommended speed           (default 1)
%   ABL_LAT         one-way V2X latency [s]                             (default 0)
%   ABL_LOSS        packet loss probability per message and direction   (default 0)
%   ABL_PEN         CAV penetration rate, 0 to 1                        (default 1)
%   ABL_SENSE       what the RSU knows about human drivers in mixed traffic:
%                   'v2x' (default) = nothing, human cars have no V2X and are
%                   invisible to the RSU; 'all' = RSU also senses them
%                   (roadside camera or radar), but still cannot instruct them

root = fileparts(fileparts(mfilename('fullpath')));

S.seed     = readNum('ABL_SEED',  1);
S.KKmax    = readNum('ABL_KKMAX', 720);
S.plot     = logical(readNum('ABL_PLOT', 1));
S.live     = logical(readNum('ABL_LIVE', 1));
S.nolog    = logical(readNum('ABL_NOLOG', 0));
S.batch    = logical(readNum('ABL_BATCH', 0));
S.schedonly= logical(readNum('ABL_SCHED_ONLY', 0));
S.outdir   = readStr('ABL_OUTDIR', fullfile(root, 'output'));
S.csv      = readStr('ABL_CSV', fullfile(root, 'output', 'ablation_runs.csv'));
S.estcsv   = readStr('ABL_EST_CSV', fullfile(root, 'output', 'estimator_runs.csv'));
S.study    = readStr('ABL_STUDY', '');
S.point    = readStr('ABL_POINT', '');
S.savelog  = logical(readNum('ABL_SAVELOG', 0));
S.schedpath= readStr('ABL_SCHED_PATH', fullfile(root, 'simplot', 'traffic_schedule.mat'));

S.sched    = lower(readStr('ABL_SCHED', 'staged'));
S.demand   = readNum('ABL_DEMAND', 0.40);
S.rt       = readNum('ABL_RT', 0.15);
S.asym     = readNum('ABL_ASYM', 0.5);
S.driver   = lower(readStr('ABL_DRIVER', 'nominal'));

S.vrec     = logical(readNum('ABL_VREC', 1));
S.lat      = readNum('ABL_LAT', 0);
S.loss     = readNum('ABL_LOSS', 0);
S.pen      = readNum('ABL_PEN', 1);
S.sense    = lower(readStr('ABL_SENSE', 'v2x'));

if ~exist(S.outdir, 'dir'), mkdir(S.outdir); end
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
