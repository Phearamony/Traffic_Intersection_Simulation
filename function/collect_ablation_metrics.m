function collect_ablation_metrics(FuelLog, label)
%COLLECT_ABLATION_METRICS  Append one summary row per configuration to a CSV.
%
%   Usage: add ONE line at the very end of each main_*.m script,
%   after FuelLog is fully assembled (i.e., after the end-of-sim harvest):
%
%       collect_ablation_metrics(FuelLog, 'C6_mpc_rsu_gpr');
%
%   Suggested labels (matching the six-configuration ablation):
%       'C1_human'         main_human_driving.m
%       'C2_v2v_idm'       main_v2v_idm.m
%       'C3_v2v_mpc'       main_v2v_mpc.m
%       'C4_idm_rsu'       main_v2v_idm_rsu.m      (IDM ctrl + IDM fwd-sim RSU)
%       'C5_idm_rsu_gpr'   main_v2v_idm_rsu_gpr.m  (IDM ctrl + GPR RSU)  -> isolates GPR
%       'C5b_mpc_rsu'      main_v2v_mpc_rsu.m      (MPC ctrl + IDM fwd-sim RSU) -> isolates MPC
%       'C6_mpc_rsu_gpr'   main_v2v_mpc_rsu_gpr.m  (proposed)
%
%   Output: output/ablation_summary.csv  (one row per run; re-running a
%   config appends a new row — keep the last row per label, or delete the
%   CSV to start fresh).
%
%   Metrics reported (addresses Reviewer 1, points 3 & 6):
%     n_rt_intent    : right-turn vehicles injected that appear in FuelLog
%     n_rt_completed : right-turn vehicles that actually completed the turn
%                      within the simulation horizon (TurnedRight flag)
%     rt_completion  : n_rt_completed / n_rt_intent  (throughput metric)
%     fuel_rt_mL     : mean fuel per COMPLETED right-turning vehicle
%     fuel_str_mL    : mean fuel per straight-going vehicle
%     gap_rt_s       : mean gap-waiting idle time of completed RT vehicles
%     red_rt_s       : mean red-phase idle time of completed RT vehicles

if isempty(FuelLog)
    warning('collect_ablation_metrics: FuelLog empty for %s — skipped.', label);
    return;
end

is_rt_intent = logical([FuelLog.TurnRight]);
is_rt_done   = logical([FuelLog.TurnedRight]);   % completed the turn in-horizon
is_str       = ~is_rt_intent;

smean = @(x) mean(x(~isnan(x) & isfinite(x)));

% Fuel / idle are reported over COMPLETED right-turners only, so that the
% per-vehicle averages are well-defined; completion ratio captures the rest.
fuel_rt  = smean([FuelLog(is_rt_done).fuel_total]);
gap_rt   = smean([FuelLog(is_rt_done).idle_gap_time]);
red_rt   = smean([FuelLog(is_rt_done).idle_red_time]);
fuel_str = smean([FuelLog(is_str).fuel_total]);

n_rt_intent = sum(is_rt_intent);
n_rt_done   = sum(is_rt_done);
completion  = n_rt_done / max(n_rt_intent, 1);

% Resolve the repo's output/ folder relative to THIS file's location
% (function/collect_ablation_metrics.m -> ../output), not the current
% working directory. This avoids failures when a main_*.m script is run
% from a different current folder than the repo root.
this_dir  = fileparts(mfilename('fullpath'));
output_dir = fullfile(this_dir, '..', 'output');
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end
csv_path = fullfile(output_dir, 'ablation_summary.csv');
write_header = ~exist(csv_path, 'file');

fid = fopen(csv_path, 'a');
if fid == -1
    error('collect_ablation_metrics: cannot open %s', csv_path);
end
if write_header
    fprintf(fid, ['label,n_rt_intent,n_rt_completed,rt_completion,', ...
                  'fuel_rt_mL,fuel_str_mL,gap_rt_s,red_rt_s\n']);
end
fprintf(fid, '%s,%d,%d,%.3f,%.2f,%.2f,%.2f,%.2f\n', ...
    label, n_rt_intent, n_rt_done, completion, ...
    fuel_rt, fuel_str, gap_rt, red_rt);
fclose(fid);

fprintf(['[collect_ablation_metrics] %s: RT completed %d/%d (%.0f%%), ', ...
         'fuel RT %.1f mL, fuel Str %.1f mL, gap %.1f s\n'], ...
    label, n_rt_done, n_rt_intent, 100*completion, fuel_rt, fuel_str, gap_rt);
end