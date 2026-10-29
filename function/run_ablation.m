function run_ablation(seeds, kkmax, with_estimator, cfgs)
%RUN_ABLATION  Multi-seed ablation of every configuration.
%
%   run_ablation                       % seeds 101:120, 720 steps (360 s)
%   run_ablation(101:105)              % quick check with 5 seeds
%   run_ablation(101:120, 720, true)   % also the estimator comparison per seed
%   run_ablation(101:105, 720, false, {'C1', 'C4'})   % only some configurations
%
%   Configurations (sim/sim_config.m):
%     C1 human drivers | V2V_MPC (CAVs, no RSU) | C2 the method of [5]
%     (IDM, IDM estimator, 4 s gap, once per green, turners one after
%     another) | C3 the CISE method (as C2 with GPR and QP-MPC) |
%     C4 proposed | C4 with one part changed: C4_NOUNC (no uncertainty
%     margins), C4_NOONLINE (no online learning), C4_SPEED (speed advice),
%     C4_STRICT (existing gaps only), C4_SEQ (turners one after another,
%     as [5]), C4_NOADV (no pulling forward), C4_ONCE (planned once per green)
%   Also available: V2V_IDM, C2G, C3I (the factorial of the CISE paper).
%   All share the traffic of each seed. Output in output/2_main_study/:
%     ablation_runs.csv     one row per (seed, configuration), every metric
%     estimator_runs.csv    estimator accuracy per seed (with_estimator)
%     ablation_stats.csv, ablation_pairs.csv   from summarize_ablation()
%     ablation_errors.log   runs that failed (a failed run does not stop the study)
%   Seeds start at 101: the estimators are trained on seeds 1 to 20.
%   Train first (main/main_train.m). main_all.m with FRESH = true deletes
%   the old study before a new one.

if nargin < 1 || isempty(seeds), seeds = 101:120; end
if nargin < 2 || isempty(kkmax), kkmax = 720; end
if nargin < 3 || isempty(with_estimator), with_estimator = false; end
if nargin < 4 || isempty(cfgs)
    cfgs = {'C1', 'V2V_MPC', 'C2', 'C3', 'C4', 'C4_NOUNC', 'C4_NOONLINE', 'C4_SPEED', ...
            'C4_STRICT', 'C4_SEQ', 'C4_NOADV', 'C4_ONCE'};
end
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'sim')); addpath(fullfile(root, 'function'));
out = out_folder('study');
log_path = fullfile(out, 'ablation_errors.log');

names = {'ABL_SEED', 'ABL_KKMAX', 'ABL_PLOT', 'ABL_LIVE', 'ABL_NOLOG', 'ABL_STUDY', 'ABL_POINT', ...
         'ABL_OUTDIR', 'ABL_CSV', 'ABL_EST_CSV'};
old = cellfun(@getenv, names, 'UniformOutput', false);
cleanupObj = onCleanup(@() restoreEnv(names, old)); %#ok<NASGU>
abl_env_reset();
setenv('ABL_PLOT', '0'); setenv('ABL_LIVE', '0'); setenv('ABL_NOLOG', '0');
setenv('ABL_KKMAX', num2str(kkmax));
setenv('ABL_OUTDIR', out); setenv('ABL_CSV', fullfile(out, 'ablation_runs.csv'));
setenv('ABL_EST_CSV', fullfile(out, 'estimator_runs.csv'));
setenv('ABL_STUDY', 'ablation'); setenv('ABL_POINT', '-');   % rows of this study are marked

t_all = tic;
for s = seeds
    setenv('ABL_SEED', num2str(s));
    S = abl_settings();
    for k = 1:numel(cfgs)
        try
            sim_main(cfgs{k}, S);
        catch err
            logErr(log_path, sprintf('seed %d %s', s, cfgs{k}), err);
        end
    end
    if with_estimator
        try
            est_eval_run(S);
        catch err
            logErr(log_path, sprintf('seed %d estimator', s), err);
        end
    end
    fprintf('##### seed %d done (%.1f min so far) #####\n', s, toc(t_all) / 60);
end
fprintf('\nAll runs finished in %.1f min.\n', toc(t_all) / 60);
summarize_ablation(fullfile(out, 'ablation_runs.csv'));
end

function logErr(p, what, err)
fprintf(2, 'FAILED %s: %s\n', what, err.message);
fid = fopen(p, 'a');
if fid > 0
    fprintf(fid, '%s %s: %s\n', datestr(now), what, getReport(err, 'extended', 'hyperlinks', 'off'));
    fclose(fid);
end
end

function restoreEnv(names, old)
for i = 1:numel(names), setenv(names{i}, old{i}); end
end
