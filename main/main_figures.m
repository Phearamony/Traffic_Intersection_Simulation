%% MAIN_FIGURES  Redraw the figures from results that already exist (press F5).
%
% Needs output/2_main_study/ablation_runs.csv (from main_all.m) and, for the
% stress-test figures, output/5_stress_tests/sweep_runs.csv. Re-runs one
% representative seed of every configuration for the trajectory figures (no
% rows are added). Figures go to output/3_figures/ and output/5_stress_tests/.

clear; close all;

% ---------------- settings ----------------
SEED   = [];        % [] = representative seed (C4 fuel closest to the median)
KKMAX  = 720;
SWEEPS = true;      % also redraw the stress-test figures
% ------------------------------------------

root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'sim')); addpath(fullfile(root, 'function'));
make_figures(SEED, KKMAX);
if SWEEPS && exist(out_folder('sweeps', 'sweep_runs.csv'), 'file')
    plot_sweeps();
end
