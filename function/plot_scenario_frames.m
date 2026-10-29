function plot_scenario_frames(outdir, cfgs, times, view)
%PLOT_SCENARIO_FRAMES  Snapshots of the junction for a scenario, side by side.
%
%   plot_scenario_frames(outdir)                       C1 and C4, t = 62, 68, 74, 80 s
%   plot_scenario_frames(outdir, {'C1','C3','C4'}, [60 66 72 78 84], 30)
%
%   Reads carlog_<label>.mat of each configuration (run_scenarios saves
%   them) and writes scenario_frames.png: one row per configuration, one
%   column per time. Vehicles are drawn at their real position and heading
%   (purple = right turn, gray = left turn, blue / yellow = straight from
%   the west / east).
if nargin < 2 || isempty(cfgs), cfgs = {'C1', 'C4'}; end
if nargin < 3 || isempty(times), times = [62 68 74 80]; end
if nargin < 4 || isempty(view), view = 30; end
G = geom_build(); C = G.C;
nr = numel(cfgs); nc = numel(times);
f = figure('Color', 'w', 'Position', [60 60 260 * nc 270 * nr]);
dirCol = struct('E', [0.10 0.35 0.85], 'W', [0.90 0.75 0.10]);
for r = 1:nr
    cfg = sim_config(cfgs{r});
    p = fullfile(outdir, ['carlog_' cfg.label '.mat']);
    if ~exist(p, 'file'), continue; end
    L = load(p, 'Rs'); R = L.Rs; LOG = R.log; v = R.veh; ids = [v.id];
    for c = 1:nc
        ax = subplot(nr, nc, (r - 1) * nc + c); hold(ax, 'on'); axis(ax, 'equal');
        axis(ax, [-view view -view view]);
        rw = 4 * C.LANE;
        fill(ax, [-rw/2 rw/2 rw/2 -rw/2], [-200 -200 200 200], [0.62 0.62 0.62], 'EdgeColor', 'none');
        fill(ax, [-200 200 200 -200], [-rw/2 -rw/2 rw/2 rw/2], [0.62 0.62 0.62], 'EdgeColor', 'none');
        plot(ax, [-C.STOP -C.STOP], [0 rw/2], 'w-', [C.STOP C.STOP], [-rw/2 0], 'w-', 'LineWidth', 2);
        m = abs(LOG(:, 1) - times(c)) < 1e-6;
        X = LOG(m, :);
        for k = 1:size(X, 1)
            j = find(ids == X(k, 2), 1);
            P = G.P(X(k, 6));
            [x, y, psi] = geom_pose(P, X(k, 3) - C.VEH_L / 2);
            switch v(j).mov
                case 'R', fc = [0.55 0.10 0.60];
                case 'L', fc = [0.55 0.55 0.55];
                otherwise, fc = dirCol.(v(j).app);
            end
            cx = [-1 1 1 -1] * C.VEH_L / 2; cy = [-1 -1 1 1] * C.VEH_W / 2;
            patch(ax, x + cx * cos(psi) - cy * sin(psi), y + cx * sin(psi) + cy * cos(psi), fc, 'EdgeColor', 'k');
        end
        set(ax, 'XTick', [], 'YTick', []); box(ax, 'on');
        title(ax, sprintf('%s   t = %g s', cfgs{r}, times(c)));
    end
end
print(f, fullfile(outdir, 'scenario_frames.png'), '-dpng', '-r200');
close(f);
end
