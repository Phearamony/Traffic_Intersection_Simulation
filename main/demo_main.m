%% Demo: the proposed method (C4) in light constant traffic, live animation,
% nothing saved. See watch_main.m for other configurations and settings.
ROOT_ = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(ROOT_, 'main'));
watch_main('C4', 101, 'ABL_SCHED', 'const', 'ABL_DEMAND', '0.25', 'ABL_KKMAX', '360');
