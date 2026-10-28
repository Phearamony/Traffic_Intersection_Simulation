%% Estimator comparison on one seed (IDM forward simulation, physics, GPR,
% PI-GPR offline and online) - see sim/est_eval_run.m. Settings from
% abl_settings.m; appends to output/estimator_runs.csv and, with plots on,
% writes estimator_comparison_scatter.png and _residuals.png.
ROOT_ = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(ROOT_, 'sim')); addpath(fullfile(ROOT_, 'function'));
est_eval_run(abl_settings());
