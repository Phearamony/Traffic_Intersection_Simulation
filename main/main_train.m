%% MAIN_TRAIN  Train the arrival-time estimators (press F5).
%
% Runs human driving (C1) on the training seeds, records what a roadside
% sensor would see, and fits the queue physics, the plain GPR and the
% PI-GPR (with its uncertainty calibration). The last N_TEST seeds are held
% out and used only to report the accuracy.
% Writes simplot/estimators.mat (used by every RSU configuration),
% output/gpr_holdout_metrics.csv and output/gpr_model_diagnostics.png.
% Run this again whenever the traffic model changes (speeds, drivers).

clear; close all;

% ---------------- settings ----------------
SEEDS  = 1:20;      % training + test seeds
N_TEST = 4;         % the last N_TEST seeds are the test set
KKMAX  = 720;       % steps per run (360 s)
% ------------------------------------------

root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'sim')); addpath(fullfile(root, 'function'));
train_estimators(SEEDS, N_TEST, KKMAX);
