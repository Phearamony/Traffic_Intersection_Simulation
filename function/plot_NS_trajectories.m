function plot_NS_trajectories(LightLog, CarLogN, CarLogS, stop_line, outFile, opts)
% Merged NS trajectory plot for conference paper
% North vehicles: positive y
% South vehicles: negative y
% y = signed distance to stop line [m]

if nargin < 5 || isempty(outFile)
    outFile = 'NS_merged.png';
end
if nargin < 6
    opts = struct;
end

% ---------------- Options ----------------
draw_guides = getfielddef(opts, 'draw_guides', false);
v_free      = getfielddef(opts, 'v_free', 12);
light_width = getfielddef(opts, 'light_width', 4);

fig_width   = getfielddef(opts, 'fig_width', 650);   % shorter width
fig_height  = getfielddef(opts, 'fig_height', 500);  % taller height
font_size   = getfielddef(opts, 'font_size', 10);
line_width  = getfielddef(opts, 'line_width', 1.0);

% Optional manual y limit, for example opts.ymax = 120
manual_ymax = getfielddef(opts, 'ymax', []);

% ---------------- Time range ----------------
tMax = 0;

if ~isempty(LightLog)
    tMax = max(tMax, max([LightLog.Time]));
end
if ~isempty(CarLogN)
    tMax = max(tMax, max([CarLogN.Time]));
end
if ~isempty(CarLogS)
    tMax = max(tMax, max([CarLogS.Time]));
end

if tMax <= 0
    warning('No data to plot.');
    return;
end

% ---------------- Figure ----------------
hFig = figure('Name', 'NS Trajectories', 'Color', 'w');
set(hFig, 'Position', [100 100 fig_width fig_height]);

ax = axes(hFig);
hold(ax, 'on');
grid(ax, 'on');
box(ax, 'on');
ax.Layer = 'top';

xlabel(ax, 'Time [s]', 'FontSize', font_size);
ylabel(ax, 'Distance From Intersections [m]', 'FontSize', font_size);
xlim(ax, [0 tMax]);

% ---------------- Traffic light strip ----------------
paint_light_strip_y0(ax, LightLog, 'NS', tMax, light_width);

% ---------------- Optional guide lines ----------------
if draw_guides
    % North side guide lines, positive
    y0sN = 5:5:60;
    for k = 1:numel(y0sN)
        t1 = y0sN(k) / max(v_free, 0.1);
        plot(ax, [0 min(t1, tMax)], ...
                 [y0sN(k) max(0, y0sN(k) - v_free * min(t1, tMax))], ...
                 'Color', [0.75 0.75 0.75], 'LineWidth', 0.6);
    end

    % South side guide lines, negative
    y0sS = -5:-5:-60;
    for k = 1:numel(y0sS)
        t1 = abs(y0sS(k)) / max(v_free, 0.1);
        plot(ax, [0 min(t1, tMax)], ...
                 [y0sS(k) min(0, y0sS(k) + v_free * min(t1, tMax))], ...
                 'Color', [0.75 0.75 0.75], 'LineWidth', 0.6);
    end
end

% ---------------- Plot North and South together ----------------
% North cars: positive distance
plot_traj_colored(ax, CarLogN, @(Y) max(0, Y - stop_line), line_width);

% South cars: negative distance
plot_traj_colored(ax, CarLogS, @(Y) -max(0, stop_line - Y), line_width);

% ---------------- Y limits ----------------
if isempty(manual_ymax)
    yMaxN = get_abs_ymax(CarLogN, @(Y) max(0, Y - stop_line));
    yMaxS = get_abs_ymax(CarLogS, @(Y) -max(0, stop_line - Y));

    yMax = max([yMaxN, yMaxS, 10]);
    yMax = ceil(yMax / 10) * 10;   % make it clean, e.g. 117 -> 120
else
    yMax = manual_ymax;
end

ylim(ax, [-yMax yMax]);

% ---------------- Small labels inside graph ----------------
text(ax, 0.02, 0.94, '(a) From North', ...
    'Units', 'normalized', ...
    'FontSize', font_size, ...
    'FontWeight', 'bold', ...
    'VerticalAlignment', 'top');

text(ax, 0.02, 0.06, '(b) From South', ...
    'Units', 'normalized', ...
    'FontSize', font_size, ...
    'FontWeight', 'bold', ...
    'VerticalAlignment', 'bottom');

% ---------------- Single legend ----------------
hS = plot(ax, nan, nan, 'k', 'LineWidth', line_width);
hL = plot(ax, nan, nan, 'Color', [0.15 0.35 0.85], 'LineWidth', line_width);
hR = plot(ax, nan, nan, 'Color', [0.85 0.15 0.15], 'LineWidth', line_width);

legend(ax, [hS hL hR], {'Straight', 'Left turn', 'Right turn'}, ...
       'Location', 'northeast', ...
       'FontSize', font_size, ...
       'Box', 'on');

set(ax, 'FontSize', font_size);

% ---------------- Export ----------------
exportgraphics(hFig, outFile, 'Resolution', 300);
fprintf('Saved merged figure to %s\n', outFile);

end

%% =================== HELPER FUNCTIONS ===================

function paint_light_strip_y0(ax, LLog, whichField, tmax, light_width)
% Paint traffic light phases as colored strips at y = 0

if isempty(LLog)
    return;
end

times  = [LLog.Time];
phases = lower(string({LLog.(whichField)}));

i0 = 1;
for k = 2:numel(times)
    if phases(k) ~= phases(k - 1)
        drawSeg(times(i0), times(k), phases(k - 1), ax, light_width);
        i0 = k;
    end
end

drawSeg(times(i0), tmax, phases(end), ax, light_width);

% Center line at y = 0
plot(ax, [0 tmax], [0 0], 'k-', 'LineWidth', 0.8);

end

function drawSeg(t1, t2, phase, ax, light_width)
% Draw one traffic light segment

if t2 <= t1
    return;
end

switch phase
    case 'green'
        c = [0 0.7 0];
    case 'yellow'
        c = [0.95 0.75 0];
    otherwise
        c = [0.8 0 0];
end

dy = light_width / 2;

patch(ax, [t1 t2 t2 t1], [-dy -dy dy dy], c, ...
      'EdgeColor', 'none', ...
      'FaceAlpha', 0.9);

end

function plot_traj_colored(ax, CLog, distFun, lw)
% Plot trajectories colored by turn type:
% black = straight, blue = left turn, red = right turn

if isempty(CLog)
    return;
end

ids = unique([CLog.ID]);

for ii = 1:numel(ids)
    id = ids(ii);
    sel = CLog([CLog.ID] == id);

    [t, idx] = sort([sel.Time]);
    sel = sel(idx);

    y = distFun([sel.Y]);

    is_left  = sel(1).TurnLeft;
    is_right = sel(1).TurnRight;

    if is_right
        plot(ax, t, y, 'Color', [0.85 0.15 0.15], 'LineWidth', lw);
    elseif is_left
        plot(ax, t, y, 'Color', [0.15 0.35 0.85], 'LineWidth', lw);
    else
        plot(ax, t, y, 'k', 'LineWidth', lw);
    end
end

end

function ym = get_abs_ymax(CLog, distFun)
% Get maximum absolute y value from trajectory data

if isempty(CLog)
    ym = 0;
    return;
end

ids = unique([CLog.ID]);
ys = [];

for ii = 1:numel(ids)
    id = ids(ii);
    sel = CLog([CLog.ID] == id);

    y = distFun([sel.Y]);
    ys = [ys; y(:)]; %#ok<AGROW>
end

ys = ys(isfinite(ys));

if isempty(ys)
    ym = 0;
else
    ym = max(abs(ys));
end

end

function v = getfielddef(s, f, d)
% Get field from struct with default value

if isstruct(s) && isfield(s, f)
    v = s.(f);
else
    v = d;
end

end