function run_ablation(seeds, kkmax, with_estimator)
%RUN_ABLATION  Multi-seed factorial ablation for the journal paper.
%
%   run_ablation                 % seeds 101:120, 720 steps (360 s), no estimator study
%   run_ablation(101:105)        % quick check with 5 seeds
%   run_ablation(101:120, 720, true)   % also run main_estimator_compare per seed
%
%   For every seed:
%     1. main_human_driving.m      generates traffic_schedule.mat for this seed
%     2. every configuration runs on that same schedule
%   Each configuration appends one row to output/ablation_runs.csv.
%   At the end summarize_ablation() writes ablation_stats.csv and
%   ablation_pairs.csv (means, 95% CIs, paired Wilcoxon and t-tests).
%
%   Seeds start at 101 so that the evaluation traffic never repeats the
%   traffic used to train the GPR model (main_human_driving_gpr_train.m
%   uses seeds 1 to 20).
%
%   Train the GPR model first:  run main_human_driving_gpr_train.m once.
%   Delete output/ablation_runs.csv before a fresh study.

if nargin < 1 || isempty(seeds), seeds = 101:120; end
if nargin < 2 || isempty(kkmax), kkmax = 720; end
if nargin < 3, with_estimator = false; end

root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(root, 'class')));
addpath(genpath(fullfile(root, 'function')));
addpath(genpath(fullfile(root, 'simplot')));

scripts = {'main_human_driving', ...        % schedule for this seed
           'main_human_driving_plot', ...   % C1
           'main_v2v_idm', ...              % V2V_IDM
           'main_v2v_mpc', ...              % V2V_MPC
           'main_v2v_idm_rsu', ...          % C2
           'main_v2v_idm_rsu_gpr', ...      % C2G
           'main_v2v_mpc_rsu', ...          % C3I
           'main_v2v_mpc_rsu_gpr'};         % C3
if with_estimator
    scripts{end+1} = 'main_estimator_compare';
end

old = struct('b', getenv('ABL_BATCH'), 'p', getenv('ABL_PLOT'), ...
             'k', getenv('ABL_KKMAX'), 's', getenv('ABL_SEED'), ...
             'l', getenv('ABL_LIVE'), 'n', getenv('ABL_NOLOG'));
cleanupObj = onCleanup(@() restoreEnv(old)); %#ok<NASGU>

setenv('ABL_BATCH', '1');
setenv('ABL_PLOT', '0');
setenv('ABL_LIVE', '0');
setenv('ABL_NOLOG', '0');
setenv('ABL_KKMAX', num2str(kkmax));

log_path = fullfile(root, 'output', 'ablation_errors.log');
t_all = tic;
for s = seeds
    setenv('ABL_SEED', num2str(s));
    for k = 1:numel(scripts)
        fprintf('\n##### seed %d  |  %s #####\n', s, scripts{k});
        t1 = tic;
        try
            runIsolated(fullfile(root, 'main', [scripts{k} '.m']));
        catch err
            fprintf(2, 'FAILED seed %d %s: %s\n', s, scripts{k}, err.message);
            fid = fopen(log_path, 'a');
            if fid > 0
                fprintf(fid, '%s seed %d %s: %s\n', datestr(now), s, scripts{k}, ...
                        getReport(err, 'basic'));
                fclose(fid);
            end
        end
        close all force;
        fprintf('##### done in %.0f s #####\n', toc(t1));
    end
end
fprintf('\nAll runs finished in %.1f min.\n', toc(t_all) / 60);

summarize_ablation(fullfile(root, 'output', 'ablation_runs.csv'));
end

function runIsolated(script_path)
% Runs a script in this function's own workspace, so the script's
% variables (and any "clear" it contains) cannot touch run_ablation's loop.
run(script_path);
end

function restoreEnv(old)
setenv('ABL_BATCH', old.b);
setenv('ABL_PLOT',  old.p);
setenv('ABL_KKMAX', old.k);
setenv('ABL_SEED',  old.s);
setenv('ABL_LIVE',  old.l);
setenv('ABL_NOLOG', old.n);
end
