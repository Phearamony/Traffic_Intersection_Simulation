function plot_EW_trajectories(LightLog, CarLogE, CarLogW, stop_line, outFile, opts)
% Merged EW trajectory plot for conference paper
% East vehicles: positive y
% West vehicles: negative y
% y = signed distance to stop line [m]

if nargin < 5 || isempty(outFile), outFile = 'EW_merged.png'; end
if nargin < 6, opts = struct; end

% ---------------- Options ----------------
draw_guides = getfielddef(opts,'draw_guides',false);
v_free      = getfielddef(opts,'v_free',12);
light_width = getfielddef(opts,'light_width',4);

fig_width   = getfielddef(opts,'fig_width',500);   % narrower
fig_height  = getfielddef(opts,'fig_height',625);  % taller
font_size   = getfielddef(opts,'font_size',15);
line_width  = getfielddef(opts,'line_width',1.4);

% Optional manual y-limit, e.g. 120
manual_ymax = getfielddef(opts,'ymax',[]);

% ---------------- Time range ----------------
tMax = 0;
if ~isempty(LightLog), tMax = max(tMax, max([LightLog.Time])); end
if ~isempty(CarLogE),  tMax = max(tMax, max([CarLogE.Time]));  end
if ~isempty(CarLogW),  tMax = max(tMax, max([CarLogW.Time]));  end

if tMax <= 0
    warning('No data to plot.');
    return;
end

% ---------------- Figure ----------------
hFig = figure('Name','EW Trajectories','Color','w');
set(hFig,'Position',[100 100 fig_width fig_height]);

ax = axes(hFig);
hold(ax,'on');
grid(ax,'on');
box(ax,'on');
ax.Layer = 'top';

xlabel(ax,'Time [s]','FontSize',font_size + 2,'FontWeight','bold');
ylabel(ax,'Distance From Intersections [m]','FontSize',font_size + 2,'FontWeight','bold');
xlim(ax,[0 tMax]);

% ---------------- Traffic light strip ----------------
paint_light_strip_y0(ax, LightLog, 'EW', tMax, light_width);

% ---------------- Optional guide lines ----------------
if draw_guides
    % East side guides (positive)
    y0sE = 5:5:60;
    for k = 1:numel(y0sE)
        t1 = y0sE(k)/max(v_free,0.1);
        plot(ax, [0 min(t1,tMax)], ...
                 [y0sE(k) max(0,y0sE(k)-v_free*min(t1,tMax))], ...
                 'Color',[0.75 0.75 0.75], 'LineWidth',0.6);
    end

    % West side guides (negative)
    y0sW = -5:-5:-60;
    for k = 1:numel(y0sW)
        t1 = abs(y0sW(k))/max(v_free,0.1);
        plot(ax, [0 min(t1,tMax)], ...
                 [y0sW(k) min(0,y0sW(k)+v_free*min(t1,tMax))], ...
                 'Color',[0.75 0.75 0.75], 'LineWidth',0.6);
    end
end

% ---------------- Plot East and West together ----------------
% East: positive
plot_traj_colored(ax, CarLogE, @(X) max(0, (-stop_line) - X), line_width);

% West: negative
plot_traj_colored(ax, CarLogW, @(X) -max(0, X - stop_line), line_width);

% ---------------- Y-limits ----------------
if isempty(manual_ymax)
    yMaxE = get_abs_ymax(CarLogE, @(X) max(0, (-stop_line) - X));
    yMaxW = get_abs_ymax(CarLogW, @(X) -max(0, X - stop_line));
    yMax = max([yMaxE, yMaxW, 10]);
    yMax = ceil(yMax/10)*10;   % round nicely
else
    yMax = manual_ymax;
end

ylim(ax,[-yMax yMax]);

% ---------------- Small labels inside graph ----------------
text(ax, 0.02, 0.94, '(a) From East', ...
    'Units','normalized', ...
    'FontSize',font_size + 1, ...
    'FontWeight','bold', ...
    'VerticalAlignment','top');

text(ax, 0.02, 0.06, '(b) From West', ...
    'Units','normalized', ...
    'FontSize',font_size + 1, ...
    'FontWeight','bold', ...
    'VerticalAlignment','bottom');

% ---------------- Legend (single legend only) ----------------
hS = plot(ax, nan, nan, 'k', 'LineWidth', line_width);
hL = plot(ax, nan, nan, 'Color', [0.15 0.35 0.85], 'LineWidth', line_width);
hR = plot(ax, nan, nan, 'Color', [0.85 0.15 0.15], 'LineWidth', line_width);

legend(ax, [hS hL hR], {'Straight','Left turn','Right turn'}, ...
       'Location','northeast', 'FontSize',font_size + 1, 'Box','on');

set(ax,'FontSize',font_size,'LineWidth',1.2);

exportgraphics(hFig, outFile, 'Resolution', 300);
fprintf('Saved merged figure to %s\n', outFile);

end

% =================== HELPERS ===================

function paint_light_strip_y0(ax, LLog, whichField, tmax, light_width)
if isempty(LLog), return; end

times  = [LLog.Time];
phases = lower(string({LLog.(whichField)}));

i0 = 1;
for k = 2:numel(times)
    if phases(k) ~= phases(k-1)
        drawSeg(times(i0), times(k), phases(k-1), ax, light_width);
        i0 = k;
    end
end
drawSeg(times(i0), tmax, phases(end), ax, light_width);

% center line at y = 0
plot(ax,[0 tmax],[0 0],'k-','LineWidth',0.8);
end

function drawSeg(t1, t2, phase, ax, light_width)
if t2 <= t1, return; end

switch phase
    case 'green'
        c = [0 0.7 0];
    case 'yellow'
        c = [0.95 0.75 0];
    otherwise
        c = [0.8 0 0];
end

dy = light_width/2;
patch(ax, [t1 t2 t2 t1], [-dy -dy dy dy], c, ...
      'EdgeColor','none', 'FaceAlpha',0.9);
end

function plot_traj_colored(ax, CLog, distFun, lw)
if isempty(CLog), return; end

ids = unique([CLog.ID]);

for ii = 1:numel(ids)
    id = ids(ii);
    sel = CLog([CLog.ID] == id);
    [t, idx] = sort([sel.Time]);
    sel = sel(idx);

    y = distFun([sel.X]);

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
if isempty(CLog)
    ym = 0;
    return;
end

ids = unique([CLog.ID]);
ys = [];

for ii = 1:numel(ids)
    id = ids(ii);
    sel = CLog([CLog.ID] == id);
    y = distFun([sel.X]);
    ys = [ys; y(:)];
end

ys = ys(isfinite(ys));

if isempty(ys)
    ym = 0;
else
    ym = max(abs(ys));
end
end

function v = getfielddef(s,f,d)
if isstruct(s) && isfield(s,f)
    v = s.(f);
else
    v = d;
end
end