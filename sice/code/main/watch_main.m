function watch_main(name, seed, varargin)
%WATCH_MAIN  Watch one configuration as a live animation, with nothing saved.
%
%   watch_main('C3')                     % proposed system, seed 101
%   watch_main('C1', 105)                % human driving, seed 105
%   watch_main('main_v2v_mpc', 101)      % a script name works too
%   watch_main('C3', 1, 'ABL_SCHED', 'scenarioB')       % illustrative scenario B
%   watch_main('C3', 201, 'ABL_SCHED', 'const', 'ABL_DEMAND', '0.6')
%   watch_main('C3', 201, 'ABL_SCHED', 'const', 'ABL_PEN', '0.5')   % mixed traffic
%   watch_main('C2', 101, 'ABL_KKMAX', '240')           % first 120 s only
%
%   name   C1, V2V_IDM, V2V_MPC, C2, C2G, C3I, C3, or HUMAN (= main_human_driving),
%          or the script name (main_v2v_mpc_rsu_gpr, ...)
%   seed   traffic seed (default 101, the first ablation seed). The traffic
%          is the same as in run_ablation / run_sweeps for that seed and those
%          settings, so what you watch is exactly one of the logged runs.
%   extra  any ABL_* setting as name/value pairs (see abl_settings.m)
%
%   What this does NOT touch
%     - no row in output/ablation_runs.csv or any other results CSV (ABL_NOLOG=1)
%     - no figure saved in output/ (ABL_PLOT=0)
%     - simplot/traffic_schedule.mat is not overwritten: the schedule for
%       this seed is written to a temporary folder (ABL_SCHED_PATH)
%     - the GPR model is only read
%   Your ABL_* environment is restored when the function ends, also after
%   an error or Ctrl+C.
%
%   The animation window stays open at the end. Close it before the next call.
%   If you edited Car.m or RSU.m, type  clear classes  once before calling.

if nargin < 1 || isempty(name), name = 'C3'; end
if nargin < 2 || isempty(seed), seed = 101; end
if mod(numel(varargin), 2) ~= 0
    error('watch_main: extra settings must be name/value pairs, e.g. ''ABL_PEN'', ''0.5''');
end

root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(root, 'class')));
addpath(genpath(fullfile(root, 'function')));
addpath(genpath(fullfile(root, 'simplot')));

% ---- which script ----
if startsWith(name, 'main_')
    script = name;
elseif strcmpi(name, 'HUMAN')
    script = 'main_human_driving';
else
    script = config_map(name);
end
allowed = {'main_human_driving', 'main_human_driving_plot', 'main_v2v_idm', 'main_v2v_mpc', ...
           'main_v2v_idm_rsu', 'main_v2v_idm_rsu_gpr', 'main_v2v_mpc_rsu', 'main_v2v_mpc_rsu_gpr'};
if ~ismember(script, allowed)
    error('watch_main: %s has no animation (allowed: %s)', script, strjoin(allowed, ', '));
end

% ---- save the current ABL_* environment and restore it at the end ----
names = {'ABL_SEED','ABL_KKMAX','ABL_PLOT','ABL_LIVE','ABL_NOLOG','ABL_BATCH', ...
         'ABL_SCHED_ONLY','ABL_OUTDIR','ABL_CSV','ABL_EST_CSV','ABL_STUDY','ABL_POINT', ...
         'ABL_SAVELOG','ABL_SCHED','ABL_DEMAND','ABL_RT','ABL_ASYM','ABL_DRIVER', ...
         'ABL_VREC','ABL_LAT','ABL_LOSS','ABL_PEN','ABL_SENSE','ABL_SCHED_PATH'};
old = cellfun(@getenv, names, 'UniformOutput', false);
cleanupObj = onCleanup(@() restoreEnv(names, old)); %#ok<NASGU>

% ---- sandbox: everything that could be written goes to a temp folder ----
tmp = fullfile(tempdir, 'traffic_sim_watch');
if ~exist(tmp, 'dir'), mkdir(tmp); end

abl_env_reset();
setenv('ABL_BATCH', '1');
setenv('ABL_SEED', num2str(seed));
setenv('ABL_PLOT', '0');
setenv('ABL_NOLOG', '1');
setenv('ABL_SAVELOG', '0');
setenv('ABL_OUTDIR', tmp);
setenv('ABL_CSV', fullfile(tmp, 'runs.csv'));
setenv('ABL_EST_CSV', fullfile(tmp, 'est.csv'));
setenv('ABL_SCHED_PATH', fullfile(tmp, 'traffic_schedule.mat'));
for i = 1:2:numel(varargin)
    v = varargin{i+1};
    if isnumeric(v) || islogical(v), v = num2str(v); end
    setenv(varargin{i}, v);
end

close all;

% ---- 1. traffic schedule for this seed (no animation, nothing in output/) ----
if ~strcmp(script, 'main_human_driving')
    setenv('ABL_LIVE', '0');
    setenv('ABL_SCHED_ONLY', '1');
    runIsolated(fullfile(root, 'main', 'main_human_driving.m'));
    setenv('ABL_SCHED_ONLY', '0');
end

% ---- 2. the configuration, animated ----
setenv('ABL_LIVE', '1');
fprintf('\n=== watching %s | seed %d | nothing is saved ===\n', script, seed);
runIsolated(fullfile(root, 'main', [script '.m']));
fprintf('=== %s finished (no results written) ===\n', script);
end

function runIsolated(script_path)
run(script_path);
end

function restoreEnv(names, old)
for i = 1:numel(names)
    setenv(names{i}, old{i});
end
end
