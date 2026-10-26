function plot_ablation_summary(csv_path, out_dir)
%PLOT_ABLATION_SUMMARY  Journal figures from the multi-seed ablation.
%
%   plot_ablation_summary()                    % output/ablation_runs.csv -> output/
%   plot_ablation_summary(csv_path, out_dir)
%
%   ablation_summary.png
%       One panel per metric (fuel per right-turner, fuel per straight
%       vehicle, gap wait of right-turners, right-turn completion).
%       Each configuration: one light dot per seed, and the mean with its
%       95% confidence interval (t distribution).
%
%   ablation_paired.png
%       Paired differences per seed (B minus A, same traffic) for each
%       comparison of Table IV, for right-turner fuel and gap wait.
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
T = readtable(csv_path, 'TextType', 'string');
[~, ia] = unique(strcat(string(T.seed), '|', T.label), 'last');   % last row per (seed, config)
T = T(sort(ia), :);

cfg   = {'C1_human', 'V2V_IDM', 'V2V_MPC', 'C2_idm_rsu', 'C2G_idm_rsu_gpr', 'C3I_mpc_rsu', 'C3_mpc_rsu_gpr'};
names = {'C1', 'V2V-IDM', 'V2V-MPC', 'C2', 'C2G', 'C3I', 'C3'};
keep  = ismember(cfg, unique(T.label));
cfg = cfg(keep); names = names(keep);

col_dot  = [0.70 0.70 0.70];
col_mean = [0.10 0.10 0.10];
col_prop = [0.80 0.25 0.10];    % proposed configuration C3
fs = 11;

% ---------------- Figure 1: per-configuration summary ----------------
mets   = {'fuel_rt_mL', 'fuel_str_mL', 'gap_rt_s', 'rt_completion'};
labels = {'Fuel per right-turner [mL]', 'Fuel per straight vehicle [mL]', ...
          'Gap wait of right-turners [s]', 'Right-turn completion [%]'};
scale  = [1 1 1 100];

f1 = figure('Color', 'w', 'Position', [100 100 1100 720]);
tl = tiledlayout(f1, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for m = 1:numel(mets)
    ax = nexttile(tl); hold(ax, 'on'); box(ax, 'on'); grid(ax, 'on');
    for i = 1:numel(cfg)
        x = scale(m) * T.(mets{m})(T.label == cfg{i});
        x = x(isfinite(x));
        if isempty(x), continue; end
        jit = (rand(size(x)) - 0.5) * 0.30;
        scatter(ax, i + jit, x, 14, col_dot, 'filled', 'MarkerFaceAlpha', 0.7);
        [mu, hw] = meanCI(x);
        c = col_mean; if strcmp(cfg{i}, 'C3_mpc_rsu_gpr'), c = col_prop; end
        errorbar(ax, i, mu, hw, 'o', 'Color', c, 'MarkerFaceColor', c, ...
                 'MarkerSize', 6, 'LineWidth', 1.6, 'CapSize', 8);
    end
    xlim(ax, [0.4, numel(cfg) + 0.6]);
    set(ax, 'XTick', 1:numel(cfg), 'XTickLabel', names, 'FontSize', fs);
    ylabel(ax, labels{m}, 'FontSize', fs);
end
n_seeds = numel(unique(T.seed));
title(tl, sprintf('Mean with 95%% CI over %d seeds (grey: one dot per seed)', n_seeds), 'FontSize', fs + 1);
exportgraphics(f1, fullfile(out_dir, 'ablation_summary.png'), 'Resolution', 300);
close(f1);

% ---------------- Figure 2: paired differences ----------------
cmp = { 'C2G_idm_rsu_gpr', 'C3_mpc_rsu_gpr', 'Controller | GPR  (C2G\rightarrowC3)';
        'C2_idm_rsu',      'C3I_mpc_rsu',    'Controller | IDM est.  (C2\rightarrowC3I)';
        'C2_idm_rsu',      'C2G_idm_rsu_gpr','Estimator | IDM ctrl  (C2\rightarrowC2G)';
        'C3I_mpc_rsu',     'C3_mpc_rsu_gpr', 'Estimator | MPC ctrl  (C3I\rightarrowC3)';
        'C1_human',        'C2_idm_rsu',     'Coordination | ref.  (C1\rightarrowC2)';
        'C1_human',        'C3_mpc_rsu_gpr', 'Coordination | proposed  (C1\rightarrowC3)';
        'V2V_IDM',         'V2V_MPC',        'Controller | no RSU  (V2V-IDM\rightarrowV2V-MPC)' };
pm  = {'fuel_rt_mL', 'gap_rt_s'};
pl  = {'\Delta fuel per right-turner [mL]', '\Delta gap wait [s]'};

f2 = figure('Color', 'w', 'Position', [100 100 1100 520]);
tl2 = tiledlayout(f2, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
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
