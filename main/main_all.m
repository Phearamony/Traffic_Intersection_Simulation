%% MAIN_ALL  Every experiment of the journal paper, unattended (press F5).
%
% Steps (each can be switched off below):
%   1. train the estimators (GPR, PI-GPR, queue physics) on seeds 1-20
%   2. main study: 13 configurations x seeds 101-120 + estimator comparison
%   3. figures of a representative seed and of the whole study
%   4. illustrative scenarios A and B
%   5. stress tests (demand, right-turn share, latency, loss, penetration,
%      driver type, asymmetry, shaping limit), seeds 201-210, and their figures
% A failed step is logged and the next one still runs. Results go to
% output/, one numbered folder per step (see output/README.md):
%   1_training  2_main_study  3_figures  4_scenarios  5_stress_tests
% The console is saved to output/main_all_log.txt.
% Rough time in MATLAB: about a day for everything; steps 1-3 alone take a
% few hours.

clear; close all;

% ---------------- settings ----------------
FRESH      = true;           % true: delete the old results of the steps switched on below
DO_TRAIN   = true;           % 1. estimators
DO_STUDY   = true;           % 2. main study
DO_FIGURES = true;           % 3. figures
DO_SCEN    = true;           % 4. scenarios
DO_SWEEPS  = true;           % 5. stress tests
TRAIN_SEEDS = 1:20;          % last 4 are the held-out test seeds
STUDY_SEEDS = 101:120;
SWEEP_SEEDS = 201:210;
KKMAX       = 720;           % steps per run (720 x 0.5 s = 360 s)
CONFIGS     = {};            % {} = the default 13 (see function/run_ablation.m)
% ------------------------------------------

root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'sim')); addpath(fullfile(root, 'function'));
out = out_folder('');
abl_env_reset();

if FRESH
    % results of the steps that will run again (the trained model is kept
    % unless DO_TRAIN), and files of the older flat layout of output/
    kinds = {'training', 'study', 'figures', 'scenarios', 'sweeps'};
    on = [DO_TRAIN, DO_STUDY, DO_FIGURES, DO_SCEN, DO_SWEEPS];
    for k = find(on)
        d = out_folder(kinds{k}, '');
        if exist(d, 'dir'), rmdir(d, 's'); end
    end
    old = dir(out);
    for k = 1:numel(old)
        nm = old(k).name;
        if any(strcmp(nm, {'.', '..', 'README.md'})) || any(strcmp(nm, {'1_training', '2_main_study', ...
                '3_figures', '4_scenarios', '5_stress_tests', 'single_runs'}))
            continue;
        end
        if old(k).isdir, rmdir(fullfile(out, nm), 's'); else, delete(fullfile(out, nm)); end
    end
end

diary(fullfile(out, 'main_all_log.txt'));
t0 = tic;
steps = {
    DO_TRAIN,   'estimator training', @() train_estimators(TRAIN_SEEDS, 4, KKMAX)
    DO_STUDY,   'main study',         @() run_ablation(STUDY_SEEDS, KKMAX, true, CONFIGS)
    DO_FIGURES, 'figures',            @() make_figures([], KKMAX)
    DO_SCEN,    'scenarios',          @() run_scenarios()
    DO_SWEEPS,  'stress tests',       @() run_sweeps([], SWEEP_SEEDS)
    DO_SWEEPS,  'sweep figures',      @() plot_sweeps()
};
for k = 1:size(steps, 1)
    if ~steps{k, 1}, continue; end
    fprintf('\n==================== %s (%.1f h so far) ====================\n', steps{k, 2}, toc(t0) / 3600);
    try
        steps{k, 3}();
    catch err
        fprintf(2, 'STEP FAILED: %s\n%s\n', steps{k, 2}, getReport(err, 'extended', 'hyperlinks', 'off'));
    end
    close all force;
end
fprintf('\nmain_all finished in %.1f h\n', toc(t0) / 3600);
diary off;
