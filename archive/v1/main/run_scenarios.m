function run_scenarios(names)
%RUN_SCENARIOS  Illustrative scenarios for the paper (no randomness in traffic).
%
%   run_scenarios                       % scenarioA and scenarioB
%   run_scenarios('scenarioA')
%
%   The scenarios are fixed in make_traffic_schedule.m (written before any
%   result was seen): one green phase, a westbound right-turner facing an
%   eastbound platoon whose gaps are too short (A), and the same with a
%   second turner and a mutual right turn (B). Each runs 360 steps (180 s).
%
%   Configurations: C1, C2, C2G, C3I, C3, each with figures on.
%   Output in output/scenarios/<name>/ with the usual figure names
%   (EW_human.png, EW_idm_rsu.png, EW_mpc_rsu_gpr.png, fuel_*.png, ...),
%   the CarLog of every run (carlog_<label>.mat), and
%     scenario_overview.png   position and speed of every east-west
%                             vehicle over time for C1, C2 and C3
%   Metrics of all scenario runs: output/scenarios/scenario_runs.csv

if nargin < 1 || isempty(names), names = {'scenarioA', 'scenarioB'}; end
if ischar(names) || isstring(names), names = cellstr(names); end

root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(root, 'class')));
addpath(genpath(fullfile(root, 'function')));
addpath(genpath(fullfile(root, 'simplot')));
base = fullfile(root, 'output', 'scenarios');
if ~exist(base, 'dir'), mkdir(base); end

oldVis = get(groot, 'DefaultFigureVisible');
cleanupObj = onCleanup(@() finish(oldVis)); %#ok<NASGU>
set(groot, 'DefaultFigureVisible', 'off');

scripts = {'main_human_driving', ...        % schedule + C1 figures
           'main_human_driving_plot', ...   % C1 metrics and CarLog
           'main_v2v_idm_rsu', 'main_v2v_idm_rsu_gpr', 'main_v2v_mpc_rsu', 'main_v2v_mpc_rsu_gpr'};

for n = 1:numel(names)
    sc = names{n};
    outdir = fullfile(base, sc);
    if ~exist(outdir, 'dir'), mkdir(outdir); end
    abl_env_reset();
    setenv('ABL_BATCH', '1');  setenv('ABL_PLOT', '1');  setenv('ABL_LIVE', '0');
    setenv('ABL_NOLOG', '0');  setenv('ABL_SEED', '1');  setenv('ABL_KKMAX', '360');
    setenv('ABL_SCHED', sc);   setenv('ABL_OUTDIR', outdir);  setenv('ABL_SAVELOG', '1');
    setenv('ABL_CSV', fullfile(base, 'scenario_runs.csv'));
    setenv('ABL_STUDY', sc);   setenv('ABL_POINT', '');
    for k = 1:numel(scripts)
        fprintf('\n##### %s | %s #####\n', sc, scripts{k});
        try
            runIsolated(fullfile(root, 'main', [scripts{k} '.m']));
        catch err
            fprintf(2, 'FAILED %s %s: %s\n', sc, scripts{k}, err.message);
        end
        close all force;
    end
    try
        plot_scenario(outdir);
    catch err
        fprintf(2, 'plot_scenario failed for %s: %s\n', sc, err.message);
    end
end
end

function runIsolated(script_path)
run(script_path);
end

function finish(oldVis)
abl_env_reset();
set(groot, 'DefaultFigureVisible', oldVis);
end
