function run_all(fresh, train)
%RUN_ALL  Every experiment of the journal in one call (unattended).
%
%   run_all          % fresh start: deletes old results first
%   run_all(false)   % keep existing results (sweeps resume where they stopped)
%   run_all(true, false)   % fresh start but keep the trained GPR model
%
%   Steps: GPR training, main ablation (seeds 101-120 + estimator),
%   figures, scenarios, all stress tests (seeds 201-210), sweep figures.
%   A failed step is logged and the next step still runs.
%   Console output is saved to output/run_all_log.txt.

if nargin < 1, fresh = true; end
if nargin < 2, train = true; end
root = fileparts(fileparts(mfilename('fullpath')));
out  = fullfile(root, 'output');
addpath(genpath(fullfile(root, 'class')));
addpath(genpath(fullfile(root, 'function')));
addpath(genpath(fullfile(root, 'simplot')));
addpath(fullfile(root, 'main'));

if fresh
    f = {fullfile(out, 'ablation_runs.csv'), fullfile(out, 'estimator_runs.csv')};
    for k = 1:numel(f), if exist(f{k}, 'file'), delete(f{k}); end, end
    d = {fullfile(out, 'scenarios'), fullfile(out, 'sweeps')};
    for k = 1:numel(d), if exist(d{k}, 'dir'), rmdir(d{k}, 's'); end, end
end

diary(fullfile(out, 'run_all_log.txt'));
cleanupObj = onCleanup(@() diary('off')); %#ok<NASGU>
t0 = tic;

steps = {
    'GPR training',     @() runScript(fullfile(root, 'main', 'main_human_driving_gpr_train.m'))
    'main ablation',    @() run_ablation(101:120, 720, true)
    'figures',          @() make_figures()
    'scenarios',        @() run_scenarios()
    'stress tests',     @() run_sweeps()
    'sweep figures',    @() plot_sweeps()
};
if ~train, steps = steps(2:end, :); end
for k = 1:size(steps, 1)
    fprintf('\n==================== %s (%.1f h so far) ====================\n', ...
            steps{k,1}, toc(t0) / 3600);
    try
        steps{k,2}();
    catch err
        fprintf(2, 'STEP FAILED: %s\n%s\n', steps{k,1}, getReport(err, 'extended'));
    end
    close all force;
end
fprintf('\nrun_all finished in %.1f h\n', toc(t0) / 3600);
end

function runScript(p)
setenv('ABL_BATCH', '1');
c = onCleanup(@() setenv('ABL_BATCH', ''));
run(p);
end
