function plot_sweeps(out_dir)
%PLOT_SWEEPS  Figures and tables of the stress tests (run_sweeps.m).
%
%   plot_sweeps()            % reads output/sweeps/
%   plot_sweeps(out_dir)
%
%   For every study found in sweep_runs.csv:
%     sweep_<study>.png      8 panels against the swept setting: fuel per
%                            right-turner and per straight vehicle, gap wait,
%                            right-turn completion, mean delay, throughput,
%                            blocked arrivals, minimum turn separation.
%                            Line per configuration: mean over seeds with
%                            95% CI; dashed line = reference configuration
%                            (C1 where the setting does not apply to it).
%   For studies with the estimator comparison (sweep_estimator.csv):
%     sweep_<study>_estimator.png   RMSE on the common sample set and the
%                                   failure rate of each estimator.
%   Tables
%     sweep_stats.csv        study, point, label, metric, n, mean, std, ci95
%     sweep_limits.csv       for every study and configuration, the first
%                            point where each failure criterion is met:
%                              completion   mean right-turn completion < 90%
%                              gap          mean gap wait > 120 s (one cycle)
%                              spillback    mean blocked arrivals >= 1 per run
%                              separation   mean minimum turn separation < 2 s
%                            NaN = never met in the swept range.

root = fileparts(fileparts(mfilename('fullpath')));
if nargin < 1 || isempty(out_dir), out_dir = fullfile(root, 'output', 'sweeps'); end
runs_csv = fullfile(out_dir, 'sweep_runs.csv');
est_csv  = fullfile(out_dir, 'sweep_estimator.csv');
if ~exist(runs_csv, 'file')
    warning('plot_sweeps: %s not found.', runs_csv); return;
end
T = read_runs(runs_csv);
D = sweep_definitions();
E = [];
if exist(est_csv, 'file'), E = read_runs(est_csv); end

order  = {'C1_human', 'V2V_IDM', 'V2V_MPC', 'C2_idm_rsu', 'C2G_idm_rsu_gpr', 'C3I_mpc_rsu', 'C3_mpc_rsu_gpr'};
short  = {'C1', 'V2V-IDM', 'V2V-MPC', 'C2', 'C2G', 'C3I', 'C3'};
colors = [0.45 0.45 0.45; 0.55 0.35 0.65; 0.30 0.60 0.75; 0.20 0.45 0.80; 0.35 0.70 0.35; 0.90 0.60 0.10; 0.80 0.20 0.10];
markers = {'s', 'v', '^', 'o', 'd', 'p', 'o'};

mets  = {'fuel_rt_mL', 'fuel_str_mL', 'gap_rt_s', 'rt_completion', 'delay_all_s', 'n_exited', 'n_blocked', 'sep_min_s'};
mlab  = {'Fuel per right-turner [mL]', 'Fuel per straight vehicle [mL]', 'Gap wait [s]', ...
         'Right-turn completion [%]', 'Mean delay [s]', 'Vehicles through [veh]', ...
         'Blocked arrivals [veh]', 'Min. turn separation [s]'};
mscale = [1 1 1 100 1 1 1 1];

statRows = {}; limRows = {};
studies = unique(T.study, 'stable');
for si = 1:numel(studies)
    st = char(studies(si));
    if ~isfield(D, st), continue; end
    d = D.(st);
    vals = d.values; isCat = iscell(vals);
    if isCat, ptxt = vals; xs = 1:numel(vals); else, ptxt = arrayfun(@(v) sprintf('%g', v), vals, 'UniformOutput', false); xs = vals * d.xscale; end
    Ts = T(T.study == st, :);
    labs = intersect(order, cellstr(unique(Ts.label(Ts.point ~= "ref"))), 'stable');
    refl = intersect(order, cellstr(unique(Ts.label(Ts.point == "ref"))), 'stable');

    f = figure('Color', 'w', 'Position', [60 60 1500 700]);
    tl = tiledlayout(f, 2, 4, 'TileSpacing', 'compact', 'Padding', 'compact');
    for m = 1:numel(mets)
        ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
        hs = []; hn = {};
        for li = 1:numel(labs)
            lab = labs{li}; ci = find(strcmp(order, lab));
            mu = nan(size(xs)); hw = nan(size(xs)); nn = zeros(size(xs));
            for p = 1:numel(ptxt)
                x = mscale(m) * Ts.(mets{m})(Ts.label == lab & Ts.point == ptxt{p});
                x = x(isfinite(x));
                [mu(p), hw(p), nn(p)] = meanCI(x);
                statRows(end+1, :) = {st, ptxt{p}, lab, mets{m}, nn(p), mu(p), std(x), hw(p)}; %#ok<AGROW>
            end
            h = errorbar(ax, xs, mu, hw, ['-' markers{ci}], 'Color', colors(ci,:), ...
                         'MarkerFaceColor', colors(ci,:), 'MarkerSize', 5, 'LineWidth', 1.4, 'CapSize', 5);
            hs(end+1) = h; hn{end+1} = short{ci}; %#ok<AGROW>
        end
        for li = 1:numel(refl)
            lab = refl{li}; ci = find(strcmp(order, lab));
            x = mscale(m) * Ts.(mets{m})(Ts.label == lab & Ts.point == "ref");
            x = x(isfinite(x));
            [mu, hw, nn] = meanCI(x);
            statRows(end+1, :) = {st, 'ref', lab, mets{m}, nn, mu, std(x), hw}; %#ok<AGROW>
            if isfinite(mu)
                h = yline(ax, mu, '--', 'Color', colors(ci,:), 'LineWidth', 1.2);
                hs(end+1) = h; hn{end+1} = [short{ci} ' (reference)']; %#ok<AGROW>
            end
        end
        ylabel(ax, mlab{m});
        if isCat, set(ax, 'XTick', xs, 'XTickLabel', ptxt); xlim(ax, [0.5 numel(xs) + 0.5]); end
        if m > 4, xlabel(ax, d.xlabel); end
        if m == 1 && ~isempty(hs), legend(ax, hs, hn, 'Location', 'best', 'FontSize', 8); end
    end
    title(tl, sprintf('Study "%s": mean over seeds with 95%% CI', st), 'Interpreter', 'none');
    exportgraphics(f, fullfile(out_dir, ['sweep_' st '.png']), 'Resolution', 300);
    close(f);

    % ---- failure limits ----
    for li = 1:numel(labs)
        lab = labs{li};
        crit = nan(1, 4);
        for p = 1:numel(ptxt)
            sel = Ts.label == lab & Ts.point == ptxt{p};
            mc = mean(Ts.rt_completion(sel), 'omitnan');
            mg = mean(Ts.gap_rt_s(sel), 'omitnan');
            mb = mean(Ts.n_blocked(sel), 'omitnan');
            ms = mean(Ts.sep_min_s(sel), 'omitnan');
            hit = [mc < 0.9, mg > 120, mb >= 1, ms < 2];
            for c = 1:4
                if hit(c) && isnan(crit(c)), crit(c) = p; end
            end
        end
        pv = cell(1, 4);
        for c = 1:4
            if isnan(crit(c)), pv{c} = 'never'; else, pv{c} = ptxt{crit(c)}; end
        end
        limRows(end+1, :) = {st, lab, pv{1}, pv{2}, pv{3}, pv{4}}; %#ok<AGROW>
    end

    % ---- estimator accuracy ----
    if d.est && ~isempty(E)
        Es = E(E.study == st, :);
        if ~isempty(Es)
            f2 = figure('Color', 'w', 'Position', [60 60 1000 380]);
            tl2 = tiledlayout(f2, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
            names = {'naive', 'idm', 'gpr'}; nlab = {'naive d/v', 'IDM forward sim.', 'GPR'};
            ecol = [0.55 0.55 0.55; 0.20 0.45 0.80; 0.80 0.20 0.10];
            panels = {{'rmse_s', 'common', 'RMSE on common queries [s]'}, {'fail_pct', 'own', 'Queries without an estimate [%]'}};
            for pp = 1:2
                ax = nexttile(tl2); hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
                hs = [];
                for j = 1:3
                    mu = nan(size(xs)); hw = nan(size(xs));
                    for p = 1:numel(ptxt)
                        x = Es.(panels{pp}{1})(Es.estimator == names{j} & Es.set == panels{pp}{2} & Es.point == ptxt{p});
                        [mu(p), hw(p)] = meanCI(x(isfinite(x)));
                    end
                    hs(j) = errorbar(ax, xs, mu, hw, '-o', 'Color', ecol(j,:), 'MarkerFaceColor', ecol(j,:), ...
                                     'MarkerSize', 5, 'LineWidth', 1.4, 'CapSize', 5); %#ok<AGROW>
                end
                ylabel(ax, panels{pp}{3}); xlabel(ax, d.xlabel);
                if isCat, set(ax, 'XTick', xs, 'XTickLabel', ptxt); xlim(ax, [0.5 numel(xs) + 0.5]); end
                if pp == 1, legend(ax, hs, nlab, 'Location', 'best'); end
            end
            title(tl2, sprintf('Arrival-time estimators, study "%s"', st), 'Interpreter', 'none');
            exportgraphics(f2, fullfile(out_dir, ['sweep_' st '_estimator.png']), 'Resolution', 300);
            close(f2);
        end
    end
end

if ~isempty(statRows)
    writetable(cell2table(statRows, 'VariableNames', {'study', 'point', 'label', 'metric', 'n', 'mean', 'std', 'ci95'}), ...
               fullfile(out_dir, 'sweep_stats.csv'));
end
if ~isempty(limRows)
    L = cell2table(limRows, 'VariableNames', {'study', 'label', 'completion_below_90pct', ...
                   'gap_above_120s', 'spillback', 'separation_below_2s'});
    writetable(L, fullfile(out_dir, 'sweep_limits.csv'));
    disp(L);
end
fprintf('[plot_sweeps] figures and tables written to %s\n', out_dir);
end

function [mu, hw, n] = meanCI(x)
n = numel(x);
if n == 0, mu = NaN; hw = NaN; return; end
mu = mean(x);
if n > 1, hw = tinv(0.975, n - 1) * std(x) / sqrt(n); else, hw = 0; end
end
