function plot_scenario(outdir, cfgs)
%PLOT_SCENARIO  Overview figure of an illustrative scenario.
%
%   plot_scenario(outdir, cfgs)   reads carlog_<label>.mat of each
%   configuration from outdir and writes scenario_overview.png: one column
%   per configuration,
%     top     signed distance to the stop line [m] over time (eastbound
%             above zero, westbound below), continued through the junction
%     bottom  speed over time
%   Right-turners red, straight black, left-turners blue; the east-west
%   green is shaded.
if nargin < 2 || isempty(cfgs), cfgs = {'C1', 'C2', 'C3', 'C4'}; end
ttl = struct('C1', 'C1  human driving', 'C2', 'C2  IDM + RSU', 'C3', 'C3  MPC + RSU (GPR)', ...
             'C3I', 'C3I  MPC + RSU (IDM est.)', 'C4', 'C4  proposed');
green = [60, 114.5];
nC = numel(cfgs);
f = figure('Color', 'w', 'Position', [60 60 380 * nC 750]);
for c = 1:nC
    cfg = sim_config(cfgs{c});
    axT = subplot(2, nC, c); hold on; grid on; box on;
    axB = subplot(2, nC, c + nC); hold on; grid on; box on;
    p = fullfile(outdir, ['carlog_' cfg.label '.mat']);
    name = cfgs{c}; if isfield(ttl, name), name = ttl.(name); end
    if ~exist(p, 'file'), title(axT, [name '  (no data)']); continue; end
    L = load(p, 'Rs'); R = L.Rs; LOG = R.log; v = R.veh; ids = [v.id];
    for ax = [axT, axB]
        patch(ax, [green(1) green(2) green(2) green(1)], [-1e3 -1e3 1e3 1e3], ...
              [0.85 0.95 0.85], 'EdgeColor', 'none', 'HandleVisibility', 'off');
    end
    for k = 1:numel(v)
        m = LOG(:, 2) == v(k).id;
        if ~any(m) || ~any(v(k).app == 'EW'), continue; end
        t = LOG(m, 1); s = LOG(m, 3); sp = LOG(m, 4);
        keep = s <= 165; t = t(keep); s = s(keep); sp = sp(keep);
        y = 140 - s; if v(k).app == 'W', y = -y; end
        switch v(k).mov
            case 'R', col = [0.80 0.15 0.10]; lw = 2.0;
            case 'L', col = [0.20 0.40 0.85]; lw = 1.0;
            otherwise, col = [0.15 0.15 0.15]; lw = 1.0;
        end
        plot(axT, t, y, '-', 'Color', col, 'LineWidth', lw);
        plot(axB, t, sp, '-', 'Color', col, 'LineWidth', lw);
    end
    plot(axT, [0 200], [0 0], ':', 'Color', [0.3 0.3 0.3]);
    title(axT, name);
    ylim(axT, [-110 110]); ylim(axB, [0 32]); xlim(axT, [30 140]); xlim(axB, [30 140]);
    xlabel(axB, 'Time [s]');
    if c == 1
        ylabel(axT, 'Distance to stop line [m] (E > 0, W < 0)');
        ylabel(axB, 'Speed [m/s]');
    end
end
[~, sc] = fileparts(outdir);
annotation(f, 'textbox', [0 0.95 1 0.05], 'String', ...
    sprintf('Illustrative scenario %s (red: right-turners, shaded: east-west green)', sc), ...
    'EdgeColor', 'none', 'HorizontalAlignment', 'center', 'Interpreter', 'none');
print(f, fullfile(outdir, 'scenario_overview.png'), '-dpng', '-r200');
close(f);
end
