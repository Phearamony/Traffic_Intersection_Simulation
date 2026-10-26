function collect_ablation_metrics(FuelLog, label, seed, CarLog, extra)
%COLLECT_ABLATION_METRICS  Append one row per (configuration, seed) to a CSV.
%
%   collect_ablation_metrics(FuelLog, label)
%   collect_ablation_metrics(FuelLog, label, seed, CarLog)
%   collect_ablation_metrics(FuelLog, label, seed, CarLog, extra)
%   extra.n_blocked  arrivals that could not enter (queue reached the entry)
%   extra.qp         struct from mpc_fail_counter('get')
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
%   Output: one row per run in the CSV named by ABL_CSV (default
%   output/ablation_runs.csv). summarize_ablation.m turns the main file into
%   means, 95% confidence intervals and paired tests; plot_sweeps.m reads
%   the sweep file. With ABL_SAVELOG = 1 the CarLog is also saved to
%   ABL_OUTDIR/carlog_<label>.mat (used by plot_scenario.m).
%   If an existing CSV has an older header, it is renamed *_oldformat.csv
%   and a new file is started.
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
%     study, point                            sweep labels (empty in the main ablation)
%     seed, label
%     n_rt_intent, n_rt_done, rt_completion   right-turners entered / completed
%     n_exited, n_unfinished                  throughput and vehicles left inside
%     fuel_rt_mL, fuel_str_mL                 mean fuel per completed trip [mL]
%     gap_rt_s, red_rt_s                      mean idle time of completed right-turners [s]
%     red_str_s                               mean red idle time of straight vehicles [s]
%     sep_min_s, sep_mean_s, n_sep_below2     right-turn separation (compute_turn_safety)
%     delay_rt_s, delay_str_s, delay_all_s    mean trip time minus free-flow time
%                                             (320 m at the vehicle's desired speed), complete trips
%     n_blocked                               arrivals blocked at the entry (spillback)
%     qp_solves, qp_fail_pct                  QP-MPC solves and share that used the fallback

if nargin < 3 || isempty(seed), seed = NaN; end
if nargin < 4, CarLog = []; end
if nargin < 5 || isempty(extra), extra = struct(); end
S = abl_settings();

if isempty(FuelLog)
    warning('collect_ablation_metrics: FuelLog empty for %s, skipped.', label);
    return;
end

hasExit = isfield(FuelLog, 'Exited');
hasLeft = isfield(FuelLog, 'TurnLeft');
hasTrip = isfield(FuelLog, 'trip_time') && isfield(FuelLog, 'Vd');

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

L_model = 320;   % [m] entry at 150 m before the centre to exit 170 m after it
delay_rt = NaN; delay_str = NaN; delay_all = NaN;
if hasTrip
    tt = [FuelLog.trip_time];  vd = max([FuelLog.Vd], 0.1);
    dl = tt - L_model ./ vd;
    delay_rt  = smean(dl(is_rt));
    delay_str = smean(dl(is_str));
    delay_all = smean(dl(exited));
end

n_rt_intent  = sum(rt_intent);
n_rt_done    = sum(rt_done);
completion   = n_rt_done / max(n_rt_intent, 1);
n_exited     = sum(exited);
n_unfinished = sum(~exited);

n_blocked = NaN; qp_solves = NaN; qp_fail = NaN;
if isfield(extra, 'n_blocked'), n_blocked = extra.n_blocked; end
if isfield(extra, 'qp') && isstruct(extra.qp)
    qp_solves = extra.qp.solves;
    if qp_solves > 0, qp_fail = 100 * extra.qp.fails / qp_solves; end
end

if S.savelog && ~isempty(CarLog)
    save(fullfile(S.outdir, ['carlog_' label '.mat']), 'CarLog', 'FuelLog');
end

if S.nolog
    fprintf('[metrics] %s: ABL_NOLOG=1, row not written\n', label);
    return;
end

if ~isempty(CarLog)
    Rs = compute_turn_safety(CarLog);
else
    Rs = struct('sep_min', NaN, 'sep_mean', NaN, 'n_below2', NaN);
end

header = ['study,point,seed,label,n_rt_intent,n_rt_done,rt_completion,n_exited,n_unfinished,', ...
          'fuel_rt_mL,fuel_str_mL,gap_rt_s,red_rt_s,red_str_s,', ...
          'sep_min_s,sep_mean_s,n_sep_below2,delay_rt_s,delay_str_s,delay_all_s,', ...
          'n_blocked,qp_solves,qp_fail_pct'];

csv_path = S.csv;
csv_dir = fileparts(csv_path);
if ~isempty(csv_dir) && ~exist(csv_dir, 'dir'), mkdir(csv_dir); end
if exist(csv_path, 'file')
    fid = fopen(csv_path, 'r'); first = fgetl(fid); fclose(fid);
    if ~ischar(first) || ~strcmp(strtrim(first), header)
        [d, n, e] = fileparts(csv_path);
        movefile(csv_path, fullfile(d, [n '_oldformat' e]));
        fprintf(2, '[metrics] %s had an older header; renamed to %s_oldformat%s\n', csv_path, n, e);
    end
end
write_header = ~exist(csv_path, 'file');

fid = fopen(csv_path, 'a');
if fid == -1
    error('collect_ablation_metrics: cannot open %s', csv_path);
end
if write_header, fprintf(fid, '%s\n', header); end
fprintf(fid, '%s,%s,%g,%s,%d,%d,%.4f,%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%g,%.3f,%.3f,%.3f,%g,%g,%.3f\n', ...
    S.study, S.point, seed, label, n_rt_intent, n_rt_done, completion, n_exited, n_unfinished, ...
    fuel_rt, fuel_str, gap_rt, red_rt, red_str, Rs.sep_min, Rs.sep_mean, Rs.n_below2, ...
    delay_rt, delay_str, delay_all, n_blocked, qp_solves, qp_fail);
fclose(fid);

fprintf(['[metrics] %s %s seed %g %s: RT %d/%d, fuel RT %.1f mL, Str %.1f mL, ', ...
         'gap %.1f s, delay %.1f s, blocked %g, min sep %.1f s\n'], ...
    S.study, S.point, seed, label, n_rt_done, n_rt_intent, fuel_rt, fuel_str, gap_rt, ...
    delay_all, n_blocked, Rs.sep_min);
end
