function collect_ablation_metrics(FuelLog, label, seed, CarLog)
%COLLECT_ABLATION_METRICS  Append one row per (configuration, seed) to a CSV.
%
%   collect_ablation_metrics(FuelLog, label)
%   collect_ablation_metrics(FuelLog, label, seed, CarLog)
%
%   Call once at the end of each main_*.m script, after FuelLog is complete.
%
%   Labels used in the journal ablation:
%       'C1_human'         main_human_driving_plot.m   (IDM + noise, local gap acceptance)
%       'V2V_IDM'          main_v2v_idm.m
%       'V2V_MPC'          main_v2v_mpc.m
%       'C2_idm_rsu'       main_v2v_idm_rsu.m          (IDM ctrl + IDM fwd-sim RSU)
%       'C2G_idm_rsu_gpr'  main_v2v_idm_rsu_gpr.m      (IDM ctrl + GPR RSU)
%       'C3I_mpc_rsu'      main_v2v_mpc_rsu.m          (MPC ctrl + IDM fwd-sim RSU)
%       'C3_mpc_rsu_gpr'   main_v2v_mpc_rsu_gpr.m      (proposed)
%
%   Output: output/ablation_runs.csv, one row per run. summarize_ablation.m
%   turns it into means, 95% confidence intervals and paired tests.
%
%   Vehicle sets
%     Only vehicles that LEFT the model before the end of the run are used
%     for fuel and idle-time averages (field Exited = true), so every
%     average is over complete trips. Vehicles still inside at the end are
%     counted in n_unfinished.
%     rt  : right-turners that completed the turn and exited
%     str : straight vehicles (no left or right turn) that exited
%
%   Columns
%     seed, label
%     n_rt_intent, n_rt_done, rt_completion   right-turners entered / completed
%     n_exited, n_unfinished                  throughput and vehicles left inside
%     fuel_rt_mL, fuel_str_mL                 mean fuel per completed trip [mL]
%     gap_rt_s, red_rt_s                      mean idle time of completed right-turners [s]
%     red_str_s                               mean red idle time of straight vehicles [s]
%     sep_min_s, sep_mean_s, n_sep_below2     right-turn separation (compute_turn_safety)

if nargin < 3 || isempty(seed), seed = NaN; end
if nargin < 4, CarLog = []; end

if isempty(FuelLog)
    warning('collect_ablation_metrics: FuelLog empty for %s, skipped.', label);
    return;
end

hasExit = isfield(FuelLog, 'Exited');
hasLeft = isfield(FuelLog, 'TurnLeft');

rt_intent = logical([FuelLog.TurnRight]);
rt_done   = logical([FuelLog.TurnedRight]);
if hasExit, exited = logical([FuelLog.Exited]); else, exited = true(size(rt_intent)); end
if hasLeft, left = logical([FuelLog.TurnLeft]); else, left = false(size(rt_intent)); end

is_rt  = rt_done & exited;
is_str = ~rt_intent & ~left & exited;

smean = @(x) mean(x(isfinite(x)));

fuel_rt   = smean([FuelLog(is_rt).fuel_total]);
gap_rt    = smean([FuelLog(is_rt).idle_gap_time]);
red_rt    = smean([FuelLog(is_rt).idle_red_time]);
fuel_str  = smean([FuelLog(is_str).fuel_total]);
red_str   = smean([FuelLog(is_str).idle_red_time]);

n_rt_intent  = sum(rt_intent);
n_rt_done    = sum(rt_done);
completion   = n_rt_done / max(n_rt_intent, 1);
n_exited     = sum(exited);
n_unfinished = sum(~exited);

if ~isempty(CarLog)
    Rs = compute_turn_safety(CarLog);
else
    Rs = struct('sep_min', NaN, 'sep_mean', NaN, 'n_below2', NaN);
end

this_dir   = fileparts(mfilename('fullpath'));
output_dir = fullfile(this_dir, '..', 'output');
if ~exist(output_dir, 'dir'), mkdir(output_dir); end
csv_path = fullfile(output_dir, 'ablation_runs.csv');
write_header = ~exist(csv_path, 'file');

fid = fopen(csv_path, 'a');
if fid == -1
    error('collect_ablation_metrics: cannot open %s', csv_path);
end
if write_header
    fprintf(fid, ['seed,label,n_rt_intent,n_rt_done,rt_completion,n_exited,n_unfinished,', ...
                  'fuel_rt_mL,fuel_str_mL,gap_rt_s,red_rt_s,red_str_s,', ...
                  'sep_min_s,sep_mean_s,n_sep_below2\n']);
end
fprintf(fid, '%g,%s,%d,%d,%.4f,%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%g\n', ...
    seed, label, n_rt_intent, n_rt_done, completion, n_exited, n_unfinished, ...
    fuel_rt, fuel_str, gap_rt, red_rt, red_str, Rs.sep_min, Rs.sep_mean, Rs.n_below2);
fclose(fid);

fprintf(['[metrics] seed %g %s: RT %d/%d, fuel RT %.1f mL, Str %.1f mL, ', ...
         'gap %.1f s, min sep %.1f s\n'], ...
    seed, label, n_rt_done, n_rt_intent, fuel_rt, fuel_str, gap_rt, Rs.sep_min);
end
