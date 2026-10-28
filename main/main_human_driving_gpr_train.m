%% Train the arrival-time estimators (GPR and PI-GPR) - see train_estimators.m
ROOT_ = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(ROOT_, 'sim')); addpath(fullfile(ROOT_, 'function')); addpath(fullfile(ROOT_, 'main'));
train_estimators();
