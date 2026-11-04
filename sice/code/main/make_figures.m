function make_figures(seed, kkmax)
%MAKE_FIGURES  Every simulation figure of the journal paper, in one call.
%
%   make_figures            % picks a representative seed from ablation_runs.csv
%   make_figures(107)       % or use a seed you choose
%   make_figures(107, 720)  % and a horizon (default 720 steps = 360 s)
%
%   Run AFTER run_ablation, because the summary figures need its CSV.
%
%   What it does
%     1. Re-runs one seed of every configuration with the final figures on
%        (live animation off). These runs do NOT add rows to the results
%        CSVs (ABL_NOLOG = 1), so the statistics are not changed.
%     2. Runs the estimator comparison for the same seed with its figures.
%     3. Draws the multi-seed summary and paired-difference figures.
%     All figures keep their usual names and are written to output/.
%
%   Representative seed: the seed whose C3 fuel per right-turner is
%   closest to the median over all seeds, so the trajectory figures show a
%   typical run, not the best one.
%
%   The GPR diagnostic figure (gpr_model_diagnostics.png) comes from
%   main_human_driving_gpr_train.m and is copied if it exists.

root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(root, 'class')));
addpath(genpath(fullfile(root, 'function')));
addpath(genpath(fullfile(root, 'simplot')));
out = fullfile(root, 'output');
runs_csv = fullfile(out, 'ablation_runs.csv');

if nargin < 2 || isempty(kkmax), kkmax = 720; end
if nargin < 1 || isempty(seed)
    if ~exist(runs_csv, 'file')
        error('make_figures: %s not found. Run run_ablation first, or pass a seed.', runs_csv);
    end
    T = read_runs(runs_csv);
    C3 = T(T.label == "C3_mpc_rsu_gpr" & isfinite(T.fuel_rt_mL), :);
    if isempty(C3), error('make_figures: no C3 rows in %s.', runs_csv); end
    [~, k] = min(abs(C3.fuel_rt_mL - median(C3.fuel_rt_mL)));
    seed = C3.seed(k);
    fprintf('Representative seed: %d (C3 fuel per right-turner %.2f mL, median %.2f mL)\n', ...
            seed, C3.fuel_rt_mL(k), median(C3.fuel_rt_mL));
end

scripts = {'main_human_driving', ...        % schedule for this seed + C1 figures
           'main_v2v_idm', ...              % V2V-IDM
           'main_v2v_mpc', ...              % V2V-MPC
           'main_v2v_idm_rsu', ...          % C2
           'main_v2v_idm_rsu_gpr', ...      % C2G
           'main_v2v_mpc_rsu', ...          % C3I
           'main_v2v_mpc_rsu_gpr', ...      % C3
           'main_estimator_compare'};       % estimator scatter + residuals

names = {'ABL_BATCH','ABL_PLOT','ABL_LIVE','ABL_NOLOG','ABL_KKMAX','ABL_SEED'};
old = cellfun(@getenv, names, 'UniformOutput', false);
oldVis = get(groot, 'DefaultFigureVisible');
cleanupObj = onCleanup(@() restore(names, old, oldVis)); %#ok<NASGU>

abl_env_reset();
setenv('ABL_BATCH', '1');
setenv('ABL_PLOT',  '1');
setenv('ABL_LIVE',  '0');
setenv('ABL_NOLOG', '1');
setenv('ABL_KKMAX', num2str(kkmax));
setenv('ABL_SEED',  num2str(seed));
set(groot, 'DefaultFigureVisible', 'off');

for k = 1:numel(scripts)
    fprintf('\n##### figures | seed %d | %s #####\n', seed, scripts{k});
    t1 = tic;
    try
        runIsolated(fullfile(root, 'main', [scripts{k} '.m']));
    catch err
        fprintf(2, 'FAILED %s: %s\n', scripts{k}, err.message);
    end
    close all force;
    fprintf('##### done in %.0f s #####\n', toc(t1));
end

if exist(runs_csv, 'file')
    plot_ablation_summary(runs_csv, out);
else
    warning('make_figures: no %s, summary figures skipped.', runs_csv);
end

% ---- list what was written (original file names, all in output/) ----
files = {'gpr_model_diagnostics.png', 'estimator_comparison_scatter.png', 'estimator_comparison_residuals.png', ...
         'EW_human.png', 'EW_idm.png', 'EW_mpc.png', 'EW_idm_rsu.png', 'EW_idm_rsu_gpr.png', ...
         'EW_mpc_rsu.png', 'EW_mpc_rsu_gpr.png', ...
         'fuel_human.png', 'fuel_idm.png', 'fuel_mpc.png', 'fuel_idm_rsu.png', 'fuel_idm_rsu_gpr.png', ...
         'fuel_mpc_rsu.png', 'fuel_mpc_rsu_gpr.png', ...
         'ablation_summary.png', 'ablation_paired.png'};
n_ok = 0;
for i = 1:numel(files)
    if exist(fullfile(out, files{i}), 'file')
        n_ok = n_ok + 1;
    else
        fprintf(2, 'missing: %s\n', files{i});
    end
end
fprintf('\n%d of %d figures in %s (trajectory and fuel figures from seed %d)\n', n_ok, numel(files), out, seed);
end

function runIsolated(script_path)
run(script_path);
end

function restore(names, old, oldVis)
for i = 1:numel(names), setenv(names{i}, old{i}); end
set(groot, 'DefaultFigureVisible', oldVis);
end
