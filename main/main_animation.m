%% MAIN_ANIMATION  Watch configurations as a live animation (press F5).
%
% Plays each configuration in CONFIGS one after another on the same
% traffic (same seed), so you can compare what they do. Nothing is saved.
% Cars are drawn at their real position and heading, so turns are visible:
% right-turners purple, left-turners gray, human drivers with a thick
% outline; a waiting right-turner has a yellow dot (cyan once the RSU has
% given it a slot). The window title shows the configuration and time.
% Close the figure window to skip to the next configuration.
%
% Configurations: C1 (human), V2V_MPC (CAVs, no RSU), C2 ([5]), C3 (CISE),
% C4 (proposed), C4_NOUNC, C4_NOONLINE, C4_SPEED, C4_STRICT, C4_SEQ,
% C4_NOADV, C4_ONCE (C4 with one part changed); also V2V_IDM, C2G, C3I.

clear; close all;

% ---------------- settings ----------------
CONFIGS = {'C1', 'C2', 'C3', 'C4'};   % or 'ALL' for all twelve of the study
SEED    = 101;
KKMAX   = 720;          % steps (720 x 0.5 s = 360 s); 240 = first 120 s only
VIEW    = 60;           % half-width of the view [m]; 150 = whole road
TRAFFIC = '';           % '' = normal traffic; 'scenarioA' or 'scenarioB' = fixed scenarios
% ------------------------------------------

root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'sim')); addpath(fullfile(root, 'function'));
if ischar(CONFIGS) && strcmpi(CONFIGS, 'ALL')
    CONFIGS = {'C1', 'V2V_MPC', 'C2', 'C3', 'C4', 'C4_NOUNC', 'C4_NOONLINE', 'C4_SPEED', ...
               'C4_STRICT', 'C4_SEQ', 'C4_NOADV', 'C4_ONCE'};
end
if ischar(CONFIGS), CONFIGS = {CONFIGS}; end
extra = {'ABL_KKMAX', num2str(KKMAX), 'VIEW', VIEW};
if ~isempty(TRAFFIC), extra = [extra, {'ABL_SCHED', TRAFFIC}]; end
for k = 1:numel(CONFIGS)
    try
        watch_main(CONFIGS{k}, SEED, extra{:});
    catch err
        fprintf(2, '%s stopped: %s\n', CONFIGS{k}, err.message);   % e.g. window closed
    end
    close all;
end
