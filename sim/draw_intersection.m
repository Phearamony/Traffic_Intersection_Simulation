function hfig = draw_intersection(V, G, t, sig, rsu, hfig, view)
%DRAW_INTERSECTION  One animation frame: roads, signals, RSU and vehicles.
%
%   Vehicles are drawn as rotated rectangles at their real position and
%   heading on their path, so turns are visible as turns.
%   Colours: straight = approach colour, left turn = grey, right turn =
%   purple. Human drivers (mixed traffic) have a thick black outline. A
%   right-turner waiting for its gap has a small yellow mark; one that the
%   RSU has planned (go time received) a small cyan mark.
%   view  axis half-width [m] (default 60; 150 shows the whole road)

if nargin < 7 || isempty(view), view = 60; end
C = G.C;
if isempty(hfig) || ~ishandle(hfig)
    hfig = figure('Color', 'w', 'Name', 'Intersection');
end
set(0, 'CurrentFigure', hfig);
clf; hold on; axis equal; axis([-view view -view view]);
set(gca, 'Color', [0.35 0.55 0.30]);           % grass
rw = 2 * 2 * C.LANE;                            % road width 6 m
fill([-rw/2 rw/2 rw/2 -rw/2], [-200 -200 200 200], [0.62 0.62 0.62], 'EdgeColor', 'none');
fill([-200 200 200 -200], [-rw/2 -rw/2 rw/2 rw/2], [0.62 0.62 0.62], 'EdgeColor', 'none');
plot([-200 -rw/2], [0 0], 'w--', [rw/2 200], [0 0], 'w--', 'LineWidth', 1);
plot([0 0], [-200 -rw/2], 'w--', [0 0], [rw/2 200], 'w--', 'LineWidth', 1);
% stop lines (left-hand traffic: E lane at y > 0 stops at x = -10, ...)
plot([-C.STOP -C.STOP], [0 rw/2], 'w-', 'LineWidth', 3);
plot([C.STOP C.STOP], [-rw/2 0], 'w-', 'LineWidth', 3);
plot([-rw/2 0], [-C.STOP -C.STOP], 'w-', 'LineWidth', 3);   % N lane (x < 0)
plot([0 rw/2], [C.STOP C.STOP], 'w-', 'LineWidth', 3);      % S lane (x > 0)

% signals
col = struct('green', [0 0.8 0], 'yellow', [1 0.85 0], 'red', [0.9 0 0]);
ew = 'red'; if isfield(sig, 'E'), ew = sig.E; end
ns = 'red'; if isfield(sig, 'N'), ns = sig.N; end
lamp(-C.STOP - 3, rw/2 + 2, col.(ew)); lamp(C.STOP + 3, -rw/2 - 2, col.(ew));
lamp(-rw/2 - 2, -C.STOP - 3, col.(ns)); lamp(rw/2 + 2, C.STOP + 3, col.(ns));

% RSU
if ~isempty(rsu)
    rectangle('Position', [rw/2 + 4, rw/2 + 4, 3, 3], 'FaceColor', [0.2 0.2 0.2], 'EdgeColor', 'k');
    text(rw/2 + 5.5, rw/2 + 9, 'RSU', 'HorizontalAlignment', 'center', 'FontSize', 8, 'FontWeight', 'bold');
end

dirCol = struct('E', [0.10 0.35 0.85], 'W', [0.90 0.75 0.10], 'N', [0.85 0.15 0.15], 'S', [0.15 0.65 0.20]);
L = C.VEH_L; W = C.VEH_W;
for i = 1:numel(V)
    P = G.P(V(i).p);
    [x, y, psi] = geom_pose(P, V(i).s - L/2);     % vehicle centre
    switch V(i).mov
        case 'L', fc = [0.55 0.55 0.55];
        case 'R', fc = [0.55 0.10 0.60];
        otherwise, fc = dirCol.(V(i).app);
    end
    cx = [-L/2 L/2 L/2 -L/2]; cy = [-W/2 -W/2 W/2 W/2];
    xr = x + cx * cos(psi) - cy * sin(psi);
    yr = y + cx * sin(psi) + cy * cos(psi);
    lw = 0.5; if ~V(i).isCAV, lw = 2.0; end
    patch(xr, yr, fc, 'EdgeColor', 'k', 'LineWidth', lw);
    if V(i).mov == 'R' && ~V(i).committed && V(i).s >= P.s_stop
        mk = [1 0.9 0];
        if isfield(V(i).msg, 'go') && isfinite(V(i).msg.go), mk = [0 0.9 0.9]; end
        plot(x, y, 'o', 'MarkerSize', 4, 'MarkerFaceColor', mk, 'MarkerEdgeColor', 'k');
    end
end
title(sprintf('t = %5.1f s', t));
set(gca, 'XTick', [], 'YTick', []); box on;
drawnow;
end

function lamp(x, y, c)
rectangle('Position', [x - 1.2, y - 1.2, 2.4, 2.4], 'Curvature', [1 1], 'FaceColor', c, 'EdgeColor', 'k');
end
