function plot_ablation_summary(csv_path, out_dir)
%PLOT_ABLATION_SUMMARY  Journal figures from the multi-seed ablation.
%
%   plot_ablation_summary()                    % output/ablation_runs.csv -> output/
%   plot_ablation_summary(csv_path, out_dir)
%
%   ablation_summary.png
%       One panel per metric (fuel per right-turner, fuel of all vehicles
%       per distance, gap wait, right-turn completion, vehicles that
%       finished, mean delay).
%       Each configuration: one light dot per seed, and the mean with its
%       95% confidence interval (t distribution).
%
%   ablation_paired.png
%       Paired differences per seed (B minus A, same traffic) for each
%       comparison of Table IV, for right-turner fuel, gap wait and throughput.
%       Dot per seed, mean with 95% CI, dashed line at zero. A CI that
%       does not cross zero is a consistent effect across seeds.
%
%   Needs the Statistics and Machine Learning Toolbox (tinv).

root = fileparts(fileparts(mfilename('fullpath')));
if nargin < 1 || isempty(csv_path), csv_path = fullfile(root, 'output', 'ablation_runs.csv'); end
if nargin < 2 || isempty(out_dir),  out_dir  = fullfile(root, 'output'); end
if ~exist(csv_path, 'file')
    error('plot_ablation_summary: %s not found. Run run_ablation first.', csv_path);
end

rng(0);   % fixed jitter, so the figure is the same every time it is made
T = read_runs(csv_path);   % last row per (seed, configuration)
if ismember('study', T.Properties.VariableNames) && any(T.study == "ablation")
    T = T(T.study == "ablation", :);           % only rows written by run_ablation
end

cfg   = {'C1_human', 'V2V_IDM', 'V2V_MPC', 'C2_idm_rsu', 'C2G_idm_rsu_gpr', 'C3I_mpc_rsu', ...
         'C3_mpc_rsu_gpr', 'C4_no_unc', 'C4_no_online', 'C4_speed_adv', 'C4_strict', 'C4_seq', ...
         'C4_no_adv', 'C4_once', 'C4_proposed'};
names = {'C1', 'V2V-IDM', 'V2V-MPC', 'C2 [5]', 'C2G', 'C3I', 'C3 CISE', 'C4-unc', 'C4-onl', 'C4-spd', ...
         'C4-strict', 'C4-seq', 'C4-adv', 'C4-once', 'C4'};
keep  = ismember(cfg, cellstr(unique(T.label)));
cfg = cfg(keep); names = names(keep);

col_dot  = [0.70 0.70 0.70];
col_mean = [0.10 0.10 0.10];
col_prop = [0.80 0.25 0.10];    % proposed configuration C4
fs = 11;

% ---------------- Figure 1: per-configuration summary ----------------
mets   = {'fuel_rt_mL', 'fuel_L100km', 'gap_rt_s', 'rt_completion', 'n_exited', 'delay_all_s', ...
          'n_conflict', 'jerk_rms'};
labels = {'Fuel per right-turner [mL]', 'Fuel, all vehicles [L/100 km]', ...
          'Gap wait of right-turners [s]', 'Right-turn completion [%]', ...
          'Vehicles that finished [veh]', 'Mean delay [s]', ...
          'Turn conflicts per run (PET < 1.5 s)', 'RMS jerk [m/s^3]'};
scale  = [1 1 1 100 1 1 1 1];
mets_ok = ismember(mets, T.Properties.VariableNames);
mets = mets(mets_ok); labels = labels(mets_ok); scale = scale(mets_ok);

f1 = figure('Color', 'w', 'Position', [100 100 1900 720]);
tl = tiledlayout(f1, 2, 4, 'TileSpacing', 'compact', 'Padding', 'compact');
for m = 1:numel(mets)
    ax = nexttile(tl); hold(ax, 'on'); box(ax, 'on'); grid(ax, 'on');
    for i = 1:numel(cfg)
        x = scale(m) * T.(mets{m})(T.label == cfg{i});
        x = x(isfinite(x));
        if isempty(x), continue; end
        jit = (rand(size(x)) - 0.5) * 0.30;
        scatter(ax, i + jit, x, 14, col_dot, 'filled', 'MarkerFaceAlpha', 0.7);
        [mu, hw] = meanCI(x);
        c = col_mean; if strcmp(cfg{i}, 'C4_proposed'), c = col_prop; end
        errorbar(ax, i, mu, hw, 'o', 'Color', c, 'MarkerFaceColor', c, ...
                 'MarkerSize', 6, 'LineWidth', 1.6, 'CapSize', 8);
    end
    xlim(ax, [0.4, numel(cfg) + 0.6]);
    set(ax, 'XTick', 1:numel(cfg), 'XTickLabel', names, 'FontSize', fs, 'XTickLabelRotation', 45);
    ylabel(ax, labels{m}, 'FontSize', fs);
end
n_seeds = numel(unique(T.seed));
title(tl, sprintf('Mean with 95%% CI over %d seeds (gray: one dot per seed)', n_seeds), 'FontSize', fs + 1);
exportgraphics(f1, fullfile(out_dir, 'ablation_summary.png'), 'Resolution', 300);
close(f1);

% ---------------- Figure 2: paired differences ----------------
cmp = { 'C1_human',        'C4_proposed',     'Proposed vs human  (C1\rightarrowC4)';
        'C2_idm_rsu',      'C4_proposed',     'Proposed vs [5]  (C2\rightarrowC4)';
        'C3_mpc_rsu_gpr',  'C4_proposed',     'Proposed vs CISE  (C3\rightarrowC4)';
        'V2V_MPC',         'C4_proposed',     'RSU | MPC  (V2V-MPC\rightarrowC4)';
        'C4_no_unc',       'C4_proposed',     'Uncertainty margins';
        'C4_no_online',    'C4_proposed',     'Online learning';
        'C4_speed_adv',    'C4_proposed',     'Arrival time vs speed advice';
        'C4_strict',       'C4_proposed',     'Gap shaping  (strict\rightarrowC4)';
        'C4_seq',          'C4_proposed',     'Joint vs successive slots';
        'C4_no_adv',       'C4_proposed',     'Pulling forward';
        'C4_once',         'C4_proposed',     'Every step vs once per green' };
pm  = {'fuel_rt_mL', 'gap_rt_s', 'n_exited'};
pl  = {'\Delta fuel per right-turner [mL]', '\Delta gap wait [s]', '\Delta vehicles finished [veh]'};

f2 = figure('Color', 'w', 'Position', [100 100 1500 520]);
tl2 = tiledlayout(f2, 1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
nC = size(cmp, 1);
for m = 1:numel(pm)
    ax = nexttile(tl2); hold(ax, 'on'); box(ax, 'on'); grid(ax, 'on');
    for c = 1:nC
        y = nC + 1 - c;    % first comparison on top
        A = T(T.label == cmp{c,1}, :); B = T(T.label == cmp{c,2}, :);
        [~, iA, iB] = intersect(A.seed, B.seed);
        d = B.(pm{m})(iB) - A.(pm{m})(iA);
        d = d(isfinite(d));
        if isempty(d), continue; end
        jit = (rand(size(d)) - 0.5) * 0.30;
        scatter(ax, d, y + jit, 14, col_dot, 'filled', 'MarkerFaceAlpha', 0.7);
        [mu, hw] = meanCI(d);
        cc = col_mean; if c == 1, cc = col_prop; end
        errorbar(ax, mu, y, hw, 'horizontal', 'o', 'Color', cc, 'MarkerFaceColor', cc, ...
                 'MarkerSize', 6, 'LineWidth', 1.6, 'CapSize', 8);
    end
    xline(ax, 0, '--', 'Color', [0.4 0.4 0.4]);
    ylim(ax, [0.4, nC + 0.6]);
    set(ax, 'YTick', 1:nC, 'FontSize', fs);
    if m == 1
        set(ax, 'YTickLabel', flip(cmp(:,3)));
    else
        set(ax, 'YTickLabel', []);
    end
    xlabel(ax, pl{m}, 'FontSize', fs);
end
title(tl2, sprintf('Paired differences B - A over %d seeds: dot per seed, mean with 95%% CI', n_seeds), ...
      'FontSize', fs + 1);
exportgraphics(f2, fullfile(out_dir, 'ablation_paired.png'), 'Resolution', 300);
close(f2);

fprintf('[plot_ablation_summary] wrote ablation_summary.png and ablation_paired.png to %s\n', out_dir);
end

function [mu, hw] = meanCI(x)
mu = mean(x);
n  = numel(x);
if n > 1
    hw = tinv(0.975, n - 1) * std(x) / sqrt(n);
else
    hw = 0;
end
end
