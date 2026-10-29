%% MAIN_DEMO  Record a demo video for a presentation (press F5).
%
% Simulates the configurations below on the same traffic and writes one
% video with a panel per configuration, all at the same moment, so the
% difference is visible side by side. Each panel title counts the vehicles
% that have left the junction and the right turns done so far.
% Output: output/demo_videos/<NAME>.mp4 (1920 x 1080, plays in PowerPoint).
% Needs the trained model (main_train.m) for C2 to C4.
%
% Good presets:
%   scenario B, t 55-100 s : two westbound turners, an oncoming platoon and a
%                            mutual right turn (the clearest story, ~1 min)
%   scenario A, t 55-95 s  : one turner, gaps that are too short
%   random traffic, seed 101, t 60-120 s : one full green of normal traffic

clear; close all;

% ---------------- settings ----------------
CONFIGS = {'C1', 'C2', 'C3', 'C4'};   % 1 to 6 panels, e.g. {'C2', 'C4'} or {'C4'}
TRAFFIC = 'scenarioB';                % 'scenarioA', 'scenarioB', or '' = random traffic
SEED    = 101;                        % traffic seed (random traffic)
T_WIN   = [55 100];                   % part of the run to record [s] (green starts at 60 s)
VIEW    = 40;                         % half-width of each panel [m]
SPEED   = 2;                          % playback speed (2 = twice real time)
FPS     = 30;
WIDTH   = 1920; HEIGHT = 1080;        % video size [px]
NAME    = '';                         % file name; '' = demo_<traffic>_<configs>
% ------------------------------------------

root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'sim')); addpath(fullfile(root, 'function'));
D = struct('configs', {CONFIGS}, 'traffic', TRAFFIC, 'seed', SEED, 't_win', T_WIN, 'view', VIEW, ...
           'speed', SPEED, 'fps', FPS, 'width', WIDTH, 'height', HEIGHT, 'name', NAME);
file = record_demo(D);
