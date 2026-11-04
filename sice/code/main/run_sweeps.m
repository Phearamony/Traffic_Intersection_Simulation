function run_sweeps(studies, seeds)
%RUN_SWEEPS  Stress tests: where does each system stop working?
%
%   run_sweeps                          % every study, seeds 201:210
%   run_sweeps('demand')                % one study
%   run_sweeps({'latency','loss'})      % several
%   run_sweeps([], 201:205)             % fewer seeds (quick check)
%
%   Studies (function/sweep_definitions.m): demand, rtshare, latency, loss,
%   penetration, driver, asym. For every point and seed the traffic
%   schedule is generated once (main_human_driving with ABL_SCHED_ONLY) and
%   shared by the configurations of that point.
%
%   Seeds 201-210 are disjoint from the GPR training/test seeds (1-20) and
%   from the main ablation (101-120).
%
%   RESUMABLE: rows already in output/sweeps/sweep_runs.csv (and
%   sweep_estimator.csv) are skipped, so you can stop MATLAB (Ctrl+C) and
%   call run_sweeps again; it continues where it stopped. Delete those
%   files to start over.
%
%   Output in output/sweeps/
%     sweep_runs.csv        one row per (study, point, seed, configuration)
%     sweep_estimator.csv   estimator accuracy per (study, point, seed)
%     sweep_errors.log      runs that failed
%   then plot_sweeps() writes the figures and summary tables.

if nargin < 2 || isempty(seeds), seeds = 201:210; end
D = sweep_definitions();
if nargin < 1 || isempty(studies), studies = fieldnames(D)'; end
if ischar(studies) || isstring(studies), studies = cellstr(studies); end

root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(root, 'class')));
addpath(genpath(fullfile(root, 'function')));
addpath(genpath(fullfile(root, 'simplot')));
out = fullfile(root, 'output', 'sweeps');
if ~exist(out, 'dir'), mkdir(out); end
runs_csv = fullfile(out, 'sweep_runs.csv');
est_csv  = fullfile(out, 'sweep_estimator.csv');
log_path = fullfile(out, 'sweep_errors.log');

oldVis = get(groot, 'DefaultFigureVisible');
cleanupObj = onCleanup(@() finish(oldVis)); %#ok<NASGU>
set(groot, 'DefaultFigureVisible', 'off');

doneRuns = readKeys(runs_csv, true);
doneEst  = readKeys(est_csv, false);

% count the work for the progress line
total = 0;
for st = studies
    d = D.(st{1}); nv = numel(d.values);
    total = total + numel(seeds) * (nv * (numel(d.cfgs) + d.est) + numel(d.ref));
end
k_run = 0; t_all = tic;

for st = studies
    name = st{1};
    if ~isfield(D, name), error('run_sweeps: unknown study %s', name); end
    d = D.(name);
    pts = d.values; if ~iscell(pts), pts = num2cell(pts); end
    points = [pts, {'ref'}];
    for ip = 1:numel(points)
        pv = points{ip};
        isRef = ischar(pv) && strcmp(pv, 'ref');
        if isRef
            cfgs = d.ref; doEst = false;
            if isempty(cfgs), continue; end
            ptxt = 'ref';
        else
            cfgs = d.cfgs; doEst = d.est;
            if ischar(pv), ptxt = pv; else, ptxt = sprintf('%g', pv); end
        end
        for s = seeds
            todo = {};
            for c = 1:numel(cfgs)
                [~, lab] = config_map(cfgs{c});
                if ~isKey(doneRuns, key(name, ptxt, s, lab)), todo{end+1} = cfgs{c}; end %#ok<AGROW>
            end
            needEst = doEst && ~isKey(doneEst, key(name, ptxt, s, ''));
            k_run = k_run + (numel(cfgs) - numel(todo)) + (doEst && ~needEst);
            if isempty(todo) && ~needEst, continue; end

            setPoint(d, name, pv, ptxt, isRef, s, runs_csv, est_csv);

            % 1. traffic schedule for this point and seed
            setenv('ABL_SCHED_ONLY', '1');
            ok = runOne(fullfile(root, 'main', 'main_human_driving.m'), log_path, name, ptxt, s, 'schedule');
            setenv('ABL_SCHED_ONLY', '');
            close all force;
            if ~ok, continue; end

            % 2. configurations
            for c = 1:numel(todo)
                k_run = k_run + 1;
                scr = config_map(todo{c});
                fprintf('\n##### [%d/%d] %s = %s | seed %d | %s  (%.1f h elapsed) #####\n', ...
                        k_run, total, name, ptxt, s, todo{c}, toc(t_all) / 3600);
                runOne(fullfile(root, 'main', [scr '.m']), log_path, name, ptxt, s, todo{c});
                close all force;
            end
            % 3. estimator comparison
            if needEst
                k_run = k_run + 1;
                fprintf('\n##### [%d/%d] %s = %s | seed %d | estimator #####\n', k_run, total, name, ptxt, s);
                runOne(fullfile(root, 'main', 'main_estimator_compare.m'), log_path, name, ptxt, s, 'estimator');
                close all force;
            end
        end
    end
end
fprintf('\nSweeps finished in %.1f h.\n', toc(t_all) / 3600);
abl_env_reset();
plot_sweeps(out);
end

% ---------------------------------------------------------------------
function setPoint(d, name, pv, ptxt, isRef, s, runs_csv, est_csv)
abl_env_reset();
setenv('ABL_BATCH', '1');  setenv('ABL_PLOT', '0');  setenv('ABL_LIVE', '0');
setenv('ABL_NOLOG', '0');  setenv('ABL_KKMAX', '720');
setenv('ABL_SEED', num2str(s));
setenv('ABL_CSV', runs_csv);  setenv('ABL_EST_CSV', est_csv);
setenv('ABL_STUDY', name);    setenv('ABL_POINT', ptxt);
for b = 1:size(d.base, 1)
    setenv(d.base{b, 1}, d.base{b, 2});
end
if ~isRef
    if ischar(pv), setenv(d.param, pv); else, setenv(d.param, sprintf('%.10g', pv)); end
end
end

function ok = runOne(script_path, log_path, name, ptxt, s, what)
ok = true;
try
    runIsolated(script_path);
catch err
    ok = false;
    fprintf(2, 'FAILED %s=%s seed %d %s: %s\n', name, ptxt, s, what, err.message);
    fid = fopen(log_path, 'a');
    if fid > 0
        fprintf(fid, '%s %s=%s seed %d %s: %s\n', datestr(now), name, ptxt, s, what, getReport(err, 'basic'));
        fclose(fid);
    end
end
end

function runIsolated(script_path)
run(script_path);
end

function k = key(study, point, seed, label)
k = sprintf('%s|%s|%d|%s', study, point, seed, label);
end

function M = readKeys(csv_path, withLabel)
M = containers.Map('KeyType', 'char', 'ValueType', 'logical');
if ~exist(csv_path, 'file'), return; end
opts = detectImportOptions(csv_path, 'Delimiter', ',');
if ~ismember('study', opts.VariableNames), return; end
txt = intersect({'study', 'point', 'label'}, opts.VariableNames);
opts = setvartype(opts, txt, 'string');
T = readtable(csv_path, opts);
if isempty(T), return; end
pt = string(T.point);
for i = 1:height(T)
    if withLabel
        M(key(char(T.study(i)), char(pt(i)), T.seed(i), char(T.label(i)))) = true;
    else
        M(key(char(T.study(i)), char(pt(i)), T.seed(i), '')) = true;
    end
end
end

function finish(oldVis)
abl_env_reset();
set(groot, 'DefaultFigureVisible', oldVis);
end
