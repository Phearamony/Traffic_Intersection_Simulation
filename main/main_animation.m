%% MAIN_ANIMATION  Watch one configuration as a live animation (press F5).
%
% Nothing is saved. Cars are drawn at their real position and heading, so
% turns are visible: right-turners purple, left-turners gray, human
% drivers with a thick outline; a waiting right-turner has a yellow dot
% (cyan once the RSU has given it a slot).
% Configurations: C1, V2V_MPC, C2 ([5]), C3 (CISE), C4 (proposed), and the
% C4 variants (see main_one.m).

clear; close all;

% ---------------- settings ----------------
CONFIG = 'C4';
SEED   = 101;
KKMAX  = 720;           % steps (720 x 0.5 s = 360 s)
VIEW   = 60;            % half-width of the view [m]; 150 = whole road
TRAFFIC = '';           % '' = normal traffic; 'scenarioA' or 'scenarioB' = fixed scenarios
% ------------------------------------------

root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'sim')); addpath(fullfile(root, 'function'));
extra = {'ABL_KKMAX', num2str(KKMAX), 'VIEW', VIEW};
if ~isempty(TRAFFIC), extra = [extra, {'ABL_SCHED', TRAFFIC}]; end
watch_main(CONFIG, SEED, extra{:});
