function watch_main(name, seed, varargin)
%WATCH_MAIN  Watch one configuration as a live animation, with nothing saved.
%
%   watch_main('C4')                          % proposed method, seed 101
%   watch_main('C1', 105)                     % human driving, seed 105
%   watch_main('C4', 1, 'ABL_SCHED', 'scenarioB')
%   watch_main('C4', 201, 'ABL_SCHED', 'const', 'ABL_PEN', '0.5')   % mixed traffic
%   watch_main('C3', 101, 'ABL_KKMAX', '240') % first 120 s only
%   watch_main('C4', 101, 'VIEW', 150)        % whole road instead of the junction
%
%   name   C1, V2V_IDM, V2V_MPC, C2, C2G, C3I, C3, C4, C4_NOUNC, C4_NOONLINE,
%          C4_SPEED, C4_STRICT, C4_SEQ, C4_NOADV, C4_ONCE (or the old script names, e.g. main_v2v_mpc_rsu_gpr)
%   seed   traffic seed (default 101)
%   extra  any ABL_* setting as name/value pairs (see abl_settings.m), and
%          'VIEW', half-width of the view in meters (default 60)
%   Nothing is written: no results row, no figure. Settings are restored
%   afterwards. Cars are drawn at their real position and heading, so
%   turns are visible; human drivers have a thick outline; a waiting
%   right-turner has a yellow dot (cyan once the RSU has given it a slot).
if nargin < 1 || isempty(name), name = 'C4'; end
if nargin < 2 || isempty(seed), seed = 101; end
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'sim')); addpath(fullfile(root, 'function'));
oldNames = struct('main_human_driving', 'C1', 'main_human_driving_plot', 'C1', ...
    'main_v2v_idm', 'V2V_IDM', 'main_v2v_mpc', 'V2V_MPC', 'main_v2v_idm_rsu', 'C2', ...
    'main_v2v_idm_rsu_gpr', 'C2G', 'main_v2v_mpc_rsu', 'C3I', 'main_v2v_mpc_rsu_gpr', 'C3', ...
    'main_proposed', 'C4');
if isfield(oldNames, name), name = oldNames.(name); end
if strcmpi(name, 'HUMAN'), name = 'C1'; end
view = 60;
names = {'ABL_SEED', 'ABL_PLOT', 'ABL_LIVE', 'ABL_NOLOG', 'ABL_SAVELOG'};
extra = cell(0, 2);
for i = 1:2:numel(varargin)
    if strcmpi(varargin{i}, 'VIEW'), view = varargin{i+1}; continue; end
    extra(end+1, :) = {varargin{i}, varargin{i+1}}; %#ok<AGROW>
end
allNames = [names, extra(:, 1)'];
old = cellfun(@getenv, allNames, 'UniformOutput', false);
c = onCleanup(@() restoreEnv(allNames, old)); %#ok<NASGU>
setenv('ABL_SEED', num2str(seed));
setenv('ABL_PLOT', '0'); setenv('ABL_LIVE', '1'); setenv('ABL_NOLOG', '1'); setenv('ABL_SAVELOG', '0');
for i = 1:size(extra, 1)
    v = extra{i, 2}; if isnumeric(v), v = num2str(v); end
    setenv(extra{i, 1}, v);
end
S = abl_settings();
cfg = sim_config(name);
fprintf('\n=== watching %s | seed %d | nothing is saved ===\n', cfg.label, seed);
R = sim_run(name, S, struct('live', true, 'view', view)); %#ok<NASGU>
end

function restoreEnv(names, old)
for i = 1:numel(names), setenv(names{i}, old{i}); end
end
