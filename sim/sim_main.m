function [R, M] = sim_main(name, S)
%SIM_MAIN  Run one configuration with the ABL_* settings and save the results.
%
%   [R, M] = sim_main('C3')          settings from abl_settings()
%   [R, M] = sim_main('C3', S)
%
%   1. sim_run (live animation when S.live)
%   2. metrics_compute, one row appended to S.csv (unless S.nolog)
%   3. figures in S.outdir when S.plot, with the usual names:
%        EW_<fig>.png    east-west trajectories (distance to the stop line)
%        fuel_<fig>.png  fuel and idle time of right-turners / others
%        EW_<fig>_ac.png accelerations
%      (<fig> from sim_config: human, idm, mpc, idm_rsu, idm_rsu_gpr,
%      mpc_rsu, mpc_rsu_gpr, proposed, ...)
%   4. carlog_<label>.mat in S.outdir when S.savelog (R without the geometry)

if nargin < 2 || isempty(S), S = abl_settings(); end
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'sim')); addpath(fullfile(root, 'function'));
cfg = sim_config(name);
fprintf('\n===== %s | seed %d | %d steps =====\n', cfg.label, S.seed, S.KKmax);
t0 = tic;
R = sim_run(name, S, struct('live', S.live));
M = metrics_compute(R);
fprintf(['[%s] exited %d, RT %d/%d, fuel %.2f L/100km, gap wait %.1f s, delay %.1f s, ' ...
         'PET min %.2f s, conflicts %d, collisions %d, blocked %d, guard %d (%.0f s)\n'], ...
    cfg.label, M.n_exited, M.n_rt_done, M.n_rt_intent, M.fuel_L100km, M.gap_rt_s, M.delay_all_s, ...
    M.pet_min_s, M.n_conflict, M.n_collision, M.n_blocked, M.guard, toc(t0));
if ~S.nolog
    metrics_write(M, cfg.label, S);
end
if (S.plot || S.savelog) && ~exist(S.outdir, 'dir'), mkdir(S.outdir); end
if S.plot
    try
        sim_figures(R, S.outdir);
    catch err
        fprintf(2, '[sim_main] figures failed: %s\n', err.message);
    end
end
if S.savelog
    Rs = R; Rs.G = [];  %#ok<NASGU>
    save(fullfile(S.outdir, ['carlog_' cfg.label '.mat']), 'Rs', 'M', '-v7');
end
end
