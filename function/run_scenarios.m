function run_scenarios(names, cfgs)
%RUN_SCENARIOS  Illustrative scenarios for the paper (fixed traffic).
%
%   run_scenarios                       % scenarioA and scenarioB
%   run_scenarios('scenarioA')
%
%   The scenarios are fixed in make_traffic_schedule.m: one green phase, a
%   westbound right-turner facing an eastbound platoon whose gaps are too
%   short (A), and the same with a second turner and a mutual right turn
%   (B). Each runs 360 steps (180 s). Configurations: C1, C2, C3, C4.
%   Output in output/4_scenarios/<name>/: the usual per-run figures,
%   carlog_<label>.mat of every run, scenario_overview.png (position and
%   speed of every east-west vehicle over time, one column per
%   configuration) and scenario_frames.png (snapshots of the junction for
%   C1, C3 and C4). Metrics: output/4_scenarios/scenario_runs.csv
if nargin < 1 || isempty(names), names = {'scenarioA', 'scenarioB'}; end
if ischar(names), names = {names}; end
if nargin < 2 || isempty(cfgs), cfgs = {'C1', 'C2', 'C3', 'C4'}; end
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'sim')); addpath(fullfile(root, 'function'));
base = out_folder('scenarios');
oldVis = get(groot, 'DefaultFigureVisible');
c = onCleanup(@() finish(oldVis)); %#ok<NASGU>
set(groot, 'DefaultFigureVisible', 'off');
for n = 1:numel(names)
    sc = names{n};
    outdir = fullfile(base, sc);
    if ~exist(outdir, 'dir'), mkdir(outdir); end
    abl_env_reset();
    setenv('ABL_PLOT', '1'); setenv('ABL_LIVE', '0'); setenv('ABL_NOLOG', '0');
    setenv('ABL_SEED', '1'); setenv('ABL_KKMAX', '360');
    setenv('ABL_SCHED', sc); setenv('ABL_OUTDIR', outdir); setenv('ABL_SAVELOG', '1');
    setenv('ABL_CSV', fullfile(base, 'scenario_runs.csv'));
    setenv('ABL_STUDY', sc); setenv('ABL_POINT', '');
    S = abl_settings();
    for k = 1:numel(cfgs)
        try
            sim_main(cfgs{k}, S);
        catch err
            fprintf(2, 'FAILED %s %s: %s\n', sc, cfgs{k}, err.message);
        end
        close all force;
    end
    try
        plot_scenario(outdir, cfgs);
        plot_scenario_frames(outdir, {'C1', 'C3', 'C4'});
    catch err
        fprintf(2, 'scenario figures failed for %s: %s\n', sc, err.message);
    end
end
end

function finish(oldVis)
abl_env_reset();
set(groot, 'DefaultFigureVisible', oldVis);
end
