function make_figures(seed, kkmax, cfgs)
%MAKE_FIGURES  Every per-run figure of the paper, plus the multi-seed summary.
%
%   make_figures            % representative seed from 2_main_study/ablation_runs.csv
%   make_figures(107)       % a seed you choose
%   make_figures(107, 720)  % and a horizon (default 720 steps = 360 s)
%
%   Run AFTER run_ablation (the summary figures need its CSV).
%   Everything goes to output/3_figures/.
%   1. Re-runs one seed of every configuration with figures on and logging
%      off (no rows are added to the results CSVs):
%        EW_<fig>.png, EW_<fig>_ac.png, fuel_<fig>.png
%   2. Estimator comparison for the same seed:
%        estimator_comparison_scatter.png, estimator_comparison_residuals.png
%   3. ablation_summary.png and ablation_paired.png (all seeds)
%   Representative seed: the one whose C4 fuel per right-turner is closest
%   to the median over seeds, so the trajectory figures show a typical run.
%   gpr_model_diagnostics.png comes from train_estimators.

root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'sim')); addpath(fullfile(root, 'function'));
out = out_folder('figures');
runs_csv = out_folder('study', 'ablation_runs.csv');
if nargin < 2 || isempty(kkmax), kkmax = 720; end
if nargin < 3 || isempty(cfgs)
    cfgs = {'C1', 'V2V_MPC', 'C2', 'C3', 'C4'};
end
if nargin < 1 || isempty(seed)
    if ~exist(runs_csv, 'file')
        error('make_figures: %s not found. Run main_all.m (main study) first, or pass a seed.', runs_csv);
    end
    T = read_runs(runs_csv);
    P = T(T.label == "C4_proposed" & isfinite(T.fuel_rt_mL), :);
    if isempty(P), error('make_figures: no C4 rows in %s.', runs_csv); end
    [~, k] = min(abs(P.fuel_rt_mL - median(P.fuel_rt_mL)));
    seed = P.seed(k);
    fprintf('Representative seed: %d\n', seed);
end

names = {'ABL_PLOT', 'ABL_LIVE', 'ABL_NOLOG', 'ABL_KKMAX', 'ABL_SEED', 'ABL_OUTDIR'};
old = cellfun(@getenv, names, 'UniformOutput', false);
oldVis = get(groot, 'DefaultFigureVisible');
c = onCleanup(@() restore(names, old, oldVis)); %#ok<NASGU>
abl_env_reset();
setenv('ABL_PLOT', '1'); setenv('ABL_LIVE', '0'); setenv('ABL_NOLOG', '1');
setenv('ABL_KKMAX', num2str(kkmax)); setenv('ABL_SEED', num2str(seed)); setenv('ABL_OUTDIR', out);
set(groot, 'DefaultFigureVisible', 'off');
S = abl_settings();
for k = 1:numel(cfgs)
    try
        sim_main(cfgs{k}, S);
    catch err
        fprintf(2, 'FAILED %s: %s\n', cfgs{k}, err.message);
    end
    close all force;
end
try
    S2 = S; S2.estcsv = fullfile(tempdir, 'est_tmp.csv');
    est_eval_run(S2);
catch err
    fprintf(2, 'FAILED estimator comparison: %s\n', err.message);
end
if exist(runs_csv, 'file')
    plot_ablation_summary(runs_csv, out);
end
fprintf('\nFigures written to %s (per-run figures from seed %d)\n', out, seed);
end

function restore(names, old, oldVis)
for i = 1:numel(names), setenv(names{i}, old{i}); end
set(groot, 'DefaultFigureVisible', oldVis);
end
