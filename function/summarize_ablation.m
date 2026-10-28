function [stats, pairs] = summarize_ablation(csv_path)
%SUMMARIZE_ABLATION  Means, 95% CIs and paired tests from ablation_runs.csv.
%
%   [stats, pairs] = summarize_ablation()            % uses output/ablation_runs.csv
%   [stats, pairs] = summarize_ablation(csv_path)
%
%   stats : one row per configuration and metric: n seeds, mean, std,
%           95% CI half-width (t distribution)
%   pairs : paired comparisons across seeds (same seed = same traffic):
%           mean difference, its 95% CI, Wilcoxon signed-rank p-value,
%           paired t-test p-value
%
%   Writes output/ablation_stats.csv and output/ablation_pairs.csv.
%   Needs the Statistics and Machine Learning Toolbox (tinv, signrank, ttest).

if nargin < 1
    csv_path = fullfile(fileparts(mfilename('fullpath')), '..', 'output', 'ablation_runs.csv');
end
out_dir = fileparts(csv_path);

T = read_runs(csv_path);   % last row per (seed, configuration)
if ismember('study', T.Properties.VariableNames) && any(T.study == "ablation")
    T = T(T.study == "ablation", :);           % only rows written by run_ablation
end

metrics = {'fuel_rt_mL', 'fuel_str_mL', 'fuel_L100km', 'gap_rt_s', 'rt_completion', ...
           'n_exited', 'red_str_s', 'pet_min_s', 'pet_mean_s', 'n_conflict', 'n_collision', ...
           'delay_rt_s', 'delay_str_s', 'delay_all_s', 'n_blocked', 'entry_wait_s', ...
           'jerk_rms', 'stops_per_veh', 'qp_fail_pct'};
metrics = metrics(ismember(metrics, T.Properties.VariableNames));
labels  = unique(T.label, 'stable');

% ---------- per-configuration statistics ----------
rows = cell(0, 6);
for i = 1:numel(labels)
    Ti = T(T.label == labels(i), :);
    for m = 1:numel(metrics)
        x = Ti.(metrics{m});
        x = x(isfinite(x));
        n = numel(x);
        if n == 0, continue; end
        mu = mean(x); sd = std(x);
        hw = NaN;
        if n > 1, hw = tinv(0.975, n - 1) * sd / sqrt(n); end
        rows(end+1, :) = {labels(i), string(metrics{m}), n, mu, sd, hw}; %#ok<AGROW>
    end
end
stats = cell2table(rows, 'VariableNames', {'label', 'metric', 'n', 'mean', 'std', 'ci95'});
writetable(stats, fullfile(out_dir, 'ablation_stats.csv'));

% ---------- paired comparisons (B minus A) ----------
cmp = { 'C1_human',        'C4_proposed';      % proposed vs uncoordinated
        'C3_mpc_rsu_gpr',  'C4_proposed';      % proposed vs conference method
        'V2V_MPC',         'C4_proposed';      % proposed vs connected, no RSU
        'C4_no_unc',       'C4_proposed';      % uncertainty margins
        'C4_no_online',    'C4_proposed';      % online learning
        'C4_speed_adv',    'C4_proposed';      % arrival-time vs speed messages
        'C2_idm_rsu',      'C3I_mpc_rsu';      % controller effect, IDM estimator
        'C2G_idm_rsu_gpr', 'C3_mpc_rsu_gpr';   % controller effect, GPR estimator
        'C2_idm_rsu',      'C2G_idm_rsu_gpr';  % estimator effect, IDM control
        'C3I_mpc_rsu',     'C3_mpc_rsu_gpr';   % estimator effect, MPC control
        'C1_human',        'C2_idm_rsu';       % reference scheme vs uncoordinated
        'C1_human',        'C3_mpc_rsu_gpr';   % conference method vs uncoordinated
        'V2V_IDM',         'V2V_MPC' };        % controllers without RSU
prow = cell(0, 11);
for c = 1:size(cmp, 1)
    A = T(T.label == cmp{c, 1}, :);
    B = T(T.label == cmp{c, 2}, :);
    if isempty(A) || isempty(B), continue; end
    [s, ia, ib] = intersect(A.seed, B.seed);
    for m = 1:numel(metrics)
        xa = A.(metrics{m})(ia); xb = B.(metrics{m})(ib);
        ok = isfinite(xa) & isfinite(xb);
        d = xb(ok) - xa(ok);
        n = numel(d);
        if n < 2, continue; end
        md = mean(d);
        hw = tinv(0.975, n - 1) * std(d) / sqrt(n);
        p_w = NaN; p_t = NaN;
        if any(d ~= 0)
            p_w = signrank(xa(ok), xb(ok));
            [~, p_t] = ttest(xa(ok), xb(ok));
        end
        rel = 100 * md / mean(xa(ok));
        prow(end+1, :) = {string(cmp{c,1}), string(cmp{c,2}), string(metrics{m}), ...
                          n, mean(xa(ok)), mean(xb(ok)), md, hw, rel, p_w, p_t}; %#ok<AGROW>
    end
end
pairs = cell2table(prow, 'VariableNames', {'A', 'B', 'metric', 'n_pairs', 'mean_A', ...
    'mean_B', 'mean_diff', 'ci95_diff', 'rel_diff_pct', 'p_wilcoxon', 'p_ttest'});
writetable(pairs, fullfile(out_dir, 'ablation_pairs.csv'));

fprintf('\n=== Ablation summary (%s) ===\n', csv_path);
disp(stats);
disp(pairs);
end
