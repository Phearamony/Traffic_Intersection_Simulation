function plot_scenario(outdir)
%PLOT_SCENARIO  One overview figure of an illustrative scenario.
%
%   plot_scenario(outdir)   reads carlog_C1_human.mat, carlog_C2_idm_rsu.mat
%   and carlog_C3_mpc_rsu_gpr.mat from outdir and writes
%   scenario_overview.png: one column per configuration,
%     top     signed distance to the stop line [m] over time
%             (eastbound above zero, westbound below, as in the trajectory figures)
%     bottom  speed over time
%   Right-turners red, straight vehicles black, left-turners blue; the
%   east-west green phase is shaded. A vehicle is drawn until it turns.

cfg  = {'carlog_C1_human.mat', 'carlog_C2_idm_rsu.mat', 'carlog_C3_mpc_rsu_gpr.mat'};
ttl  = {'C1  human driving', 'C2  IDM + RSU', 'C3  QP-MPC + RSU (GPR)'};
stop_line = 10;
green = [60, 114.5];     % east-west green of the 120 s cycle

f = figure('Color', 'w', 'Position', [60 60 1500 750]);
tl = tiledlayout(f, 2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
axTop = gobjects(1, 3); axBot = gobjects(1, 3);
for c = 1:3
    axTop(c) = nexttile(tl, c);     hold(axTop(c), 'on'); grid(axTop(c), 'on'); box(axTop(c), 'on');
    axBot(c) = nexttile(tl, c + 3); hold(axBot(c), 'on'); grid(axBot(c), 'on'); box(axBot(c), 'on');
    p = fullfile(outdir, cfg{c});
    if ~exist(p, 'file')
        title(axTop(c), [ttl{c} '  (no data)']); continue;
    end
    L = load(p, 'CarLog'); CL = L.CarLog;
    T = [CL.Time]; ID = [CL.ID]; X = [CL.X]; V = [CL.V];
    DIR = {CL.Dir}; TR = [CL.TurnRight]; TLf = [CL.TurnLeft];
    for ax = [axTop(c), axBot(c)]
        yl = [-1e3 1e3];
        patch(ax, [green(1) green(2) green(2) green(1)], [yl(1) yl(1) yl(2) yl(2)], ...
              [0.85 0.95 0.85], 'EdgeColor', 'none', 'HandleVisibility', 'off');
    end
    ids = unique(ID);
    for k = 1:numel(ids)
        m = find(ID == ids(k));
        [~, o] = sort(T(m)); m = m(o);
        d0 = DIR{m(1)};
        if ~any(strcmp(d0, {'E', 'W'})), continue; end
        stay = find(~strcmp(DIR(m), d0), 1);          % stop at the turn
        if ~isempty(stay), m = m(1:stay-1); end
        if isempty(m), continue; end
        if strcmp(d0, 'E'), y = -stop_line - X(m); else, y = -(X(m) - stop_line); end
        if any(TR(m)), col = [0.80 0.15 0.10]; lw = 2.0;
        elseif any(TLf(m)), col = [0.20 0.40 0.85]; lw = 1.0;
        else, col = [0.15 0.15 0.15]; lw = 1.0; end
        plot(axTop(c), T(m), y, '-', 'Color', col, 'LineWidth', lw);
        plot(axBot(c), T(m), V(m), '-', 'Color', col, 'LineWidth', lw);
    end
    yline(axTop(c), 0, ':', 'Color', [0.3 0.3 0.3]);
    title(axTop(c), ttl{c});
    ylim(axTop(c), [-110 110]); ylim(axBot(c), [0 32]);
    xlim(axTop(c), [30 140]);   xlim(axBot(c), [30 140]);
    xlabel(axBot(c), 'Time [s]');
    if c == 1
        ylabel(axTop(c), 'Distance to stop line [m]  (E > 0, W < 0)');
        ylabel(axBot(c), 'Speed [m/s]');
    end
end
title(tl, sprintf('Illustrative scenario: %s  (red: right-turners, shaded: east-west green)', ...
      lastFolder(outdir)), 'Interpreter', 'none');
exportgraphics(f, fullfile(outdir, 'scenario_overview.png'), 'Resolution', 300);
close(f);
end

function s = lastFolder(p)
[~, s] = fileparts(p);
end
