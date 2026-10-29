function p = out_folder(kind, file)
%OUT_FOLDER  Where each kind of result is written (one place for all paths).
%
%   out_folder('study')                       output/2_main_study
%   out_folder('study', 'ablation_runs.csv')  a file in it
%
%   kind        folder under output/        written by
%   'training'  1_training                  main_train.m (trained model, accuracy)
%   'study'     2_main_study                main_all.m step 2 (run rows, statistics)
%   'figures'   3_figures                   main_all.m step 3, main_figures.m
%   'scenarios' 4_scenarios                 main_all.m step 4
%   'sweeps'    5_stress_tests              main_all.m step 5
%   'single'    single_runs                 main_one.m
%   'demo'      demo_videos                 main_demo.m
%   ''          output itself
%   With one argument the folder is created (callers write into it); with
%   a second argument (a file name, or '' for the folder itself) only the
%   path is returned and nothing is created.

names = struct('training', '1_training', 'study', '2_main_study', 'figures', '3_figures', ...
               'scenarios', '4_scenarios', 'sweeps', '5_stress_tests', 'single', 'single_runs', ...
               'demo', 'demo_videos');
root = fileparts(fileparts(mfilename('fullpath')));
p = fullfile(root, 'output');
if ~isempty(kind)
    if ~isfield(names, kind), error('out_folder: unknown kind ''%s''.', kind); end
    p = fullfile(p, names.(kind));
end
if nargin >= 2
    if ~isempty(file), p = fullfile(p, file); end
elseif ~exist(p, 'dir')
    mkdir(p);
end
end
