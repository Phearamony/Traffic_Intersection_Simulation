%% MAIN_ONE  Run one configuration and save its figures and metrics (press F5).
%
% Configurations (sim/sim_config.m):
%   C1           human drivers, no coordination
%   V2V_MPC      CAVs with MPC, no RSU
%   C2           method of [5] (Hasan et al.)
%   C3           CISE method (conference paper)
%   C4           proposed
%   C4_NOUNC, C4_NOONLINE, C4_SPEED, C4_STRICT, C4_SEQ, C4_NOADV, C4_ONCE,
%   C4_NOGREEN
%                C4 with one part changed (see README)
%   V2V_IDM, C2G, C3I   also available
% Writes to output/single_runs/: EW_<fig>.png (trajectories),
% EW_<fig>_ac.png (accelerations), fuel_<fig>.png, and one row in
% single_runs.csv (kept apart from the main study).

clear; close all;

% ---------------- settings ----------------
CONFIG    = 'C4';
SEED      = 101;
KKMAX     = 720;        % steps (720 x 0.5 s = 360 s)
ANIMATION = true;       % live animation while it runs (slower)
FIGURES   = true;       % save the figures
SAVE_LOG  = false;      % also save output/single_runs/carlog_<label>.mat
% ------------------------------------------

root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'sim')); addpath(fullfile(root, 'function'));
abl_env_reset();
setenv('ABL_SEED', num2str(SEED)); setenv('ABL_KKMAX', num2str(KKMAX));
setenv('ABL_LIVE', num2str(ANIMATION)); setenv('ABL_PLOT', num2str(FIGURES));
setenv('ABL_SAVELOG', num2str(SAVE_LOG));
S = abl_settings();
[R, M] = sim_main(CONFIG, S);
abl_env_reset();
