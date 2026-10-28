%% C3: MPC + RSU slot planning, GPR estimator, speed advice (conference method)
% Runs the shared simulation engine (sim/sim_run.m) for this configuration;
% the configuration switches are in sim/sim_config.m. Settings (seed,
% horizon, plots, animation, logging, traffic) come from abl_settings.m:
% with no ABL_* variables set this runs seed 1, 720 steps, with the live
% animation, saves the figures to output/ and appends one row to
% output/ablation_runs.csv. To only watch, use watch_main('C3').
ROOT_ = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(ROOT_, 'sim')); addpath(fullfile(ROOT_, 'function'));
[R, M] = sim_main('C3');
