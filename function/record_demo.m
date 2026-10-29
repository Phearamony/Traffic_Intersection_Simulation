function file = record_demo(D)
%RECORD_DEMO  Record configurations side by side as a video (for slides).
%
%   file = record_demo(D)      D: settings struct (main/main_demo.m)
%     D.configs  e.g. {'C1','C2','C3','C4'}  (1 to 6 panels, same traffic)
%     D.traffic  '' (random traffic of D.seed), 'scenarioA' or 'scenarioB'
%     D.seed     traffic seed
%     D.t_win    [t0 t1] part of the run to record [s]
%     D.view     half-width of each panel [m]
%     D.speed    playback speed (2 = twice real time)
%     D.fps      frames per second of the video
%     D.width, D.height   video size [px]
%     D.name     file name without extension ('' = automatic)
%
%   Each configuration is simulated first (no drawing, nothing saved), then
%   the video is drawn from the logs: positions are interpolated between
%   the 0.5 s steps, so the motion is smooth. Every panel shows the same
%   moment. Writes output/demo_videos/<name>.mp4 (MATLAB, VideoWriter); where
%   VideoWriter is missing (Octave) the frames are written as PNG files to
%   output/demo_videos/<name>_frames/ instead.

dt = 0.5;
G = geom_build(); C = G.C;
nc = numel(D.configs);
if nc < 1 || nc > 6, error('record_demo: 1 to 6 configurations.'); end
kk = ceil(D.t_win(2) / dt) + 2;

% ---------------- 1. simulate ----------------
runs = cell(1, nc);
for k = 1:nc
    abl_env_reset();
    setenv('ABL_SEED', num2str(D.seed)); setenv('ABL_KKMAX', num2str(kk));
    setenv('ABL_PLOT', '0'); setenv('ABL_LIVE', '0'); setenv('ABL_NOLOG', '1');
    if ~isempty(D.traffic), setenv('ABL_SCHED', D.traffic); end
    S = abl_settings();
    cfg = sim_config(D.configs{k});
    fprintf('[record_demo] simulating %s ...\n', cfg.label);
    R = sim_run(D.configs{k}, S, struct('live', false));
    runs{k} = prep(R, cfg, dt);
end
abl_env_reset();

% ---------------- 2. video ----------------
if isempty(D.name)
    tr = D.traffic; if isempty(tr), tr = sprintf('seed%d', D.seed); end
    D.name = sprintf('demo_%s_%s', tr, strjoin(D.configs, '_'));
end
out = out_folder('demo');
useVW = exist('VideoWriter', 'file') == 2 || exist('VideoWriter', 'class') == 8;
if useVW
    file = fullfile(out, [D.name '.mp4']);
    vw = VideoWriter(file, 'MPEG-4'); vw.FrameRate = D.fps; vw.Quality = 95;
    open(vw);
else
    file = fullfile(out, [D.name '_frames']);
    if ~exist(file, 'dir'), mkdir(file); end
end

f = figure('Color', 'w', 'Visible', 'off', 'Units', 'pixels', ...
           'Position', [20 20 D.width D.height], 'InvertHardcopy', 'off');
[ax, tl] = layout(f, nc);
legendStrip(f);
hTop = annotation(f, 'textbox', [0 0.945 1 0.05], 'String', '', 'EdgeColor', 'none', ...
    'HorizontalAlignment', 'center', 'FontSize', 18, 'FontWeight', 'bold');
trName = D.traffic; if isempty(trName), trName = sprintf('random traffic, seed %d', D.seed); end

tf = D.t_win(1):(D.speed / D.fps):D.t_win(2);
hold0 = round(D.fps * 1); hold1 = round(D.fps * 2);      % still frames at the start and end
nf = 0;
for q = 1:numel(tf)
    t = tf(q);
    set(hTop, 'String', sprintf('t = %5.1f s      %s      (%gx speed)', t, trName, D.speed));
    for k = 1:nc
        drawPanel(ax(k), tl(k), runs{k}, G, C, t, D.view, dt);
    end
    drawnow;
    reps = 1; if q == 1, reps = hold0; elseif q == numel(tf), reps = hold1; end
    if useVW
        fr = getframe(f);
        for r = 1:reps, writeVideo(vw, fr); end
    else
        nf = nf + 1;
        print(f, fullfile(file, sprintf('frame_%05d.png', nf)), '-dpng', '-r72');
    end
end
if useVW, close(vw); end
close(f);
fprintf('[record_demo] written: %s\n', file);
end

% ======================================================================
function r = prep(R, cfg, dt)
% what the drawing needs from a run
r.cfg = cfg;
r.title = niceName(cfg.name);
LOG = R.log;
r.k = round(LOG(:, 1) / dt);
r.LOG = LOG;
v = R.veh;
r.ids = [v.id];
r.mov = [v.mov]; r.app = [v.app];
te = [v.t_exit]; te([v.exited] == false) = inf; r.t_exit = te;
end

function s = niceName(name)
switch upper(name)
    case 'C1', s = 'C1  Human drivers';
    case 'V2V_MPC', s = 'V2V-MPC  CAVs, no RSU';
    case 'C2', s = 'C2  Hasan et al. [5]';
    case 'C3', s = 'C3  CISE (GPR + MPC)';
    case 'C4', s = 'C4  Proposed';
    otherwise, s = strrep(upper(name), '_', '-');
end
end

function [ax, tl] = layout(f, n)
switch n
    case 1, nr = 1; ncol = 1;
    case 2, nr = 1; ncol = 2;
    case {3, 4}, nr = 2; ncol = 2;
    otherwise, nr = 2; ncol = 3;
end
if n == 3, ncol = 3; nr = 1; end
top = 0.93; bot = 0.07; gapx = 0.01; gapy = 0.055;
h = (top - bot - (nr - 1) * gapy) / nr - 0.035;
w = (1 - (ncol + 1) * gapx) / ncol;
pos = get(f, 'Position');
ax = gobjects(1, n); tl = gobjects(1, n);
for k = 1:n
    rI = ceil(k / ncol); cI = k - (rI - 1) * ncol;
    x0 = gapx + (cI - 1) * (w + gapx);
    y0 = top - rI * (h + 0.035) - (rI - 1) * gapy;
    % keep the panel square in pixels
    pw = w * pos(3); ph = h * pos(4); s = min(pw, ph);
    ax(k) = axes(f, 'Units', 'normalized', ...
        'Position', [x0 + (w - s / pos(3)) / 2, y0 + (h - s / pos(4)) / 2, s / pos(3), s / pos(4)]);
    tl(k) = annotation(f, 'textbox', [x0, y0 + h, w, 0.035], 'String', '', 'EdgeColor', 'none', ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 13, 'Interpreter', 'none');
end
end

function legendStrip(f)
items = {[0.10 0.35 0.85], 'straight, westbound'; [0.90 0.75 0.10], 'straight, eastbound'; ...
         [0.55 0.10 0.60], 'right turn'; [0.55 0.55 0.55], 'left turn'; ...
         [0.85 0.15 0.15], 'north-south'};
x = 0.04;
for i = 1:size(items, 1)
    annotation(f, 'rectangle', [x 0.022 0.015 0.022], 'FaceColor', items{i, 1}, 'Color', 'k');
    annotation(f, 'textbox', [x + 0.018 0.012 0.14 0.04], 'String', items{i, 2}, 'EdgeColor', 'none', ...
        'FontSize', 11, 'VerticalAlignment', 'middle');
    x = x + 0.135;
end
annotation(f, 'textbox', [x 0.012 0.30 0.04], 'String', ...
    'thick outline = human driver,  yellow dot = turner waiting', 'EdgeColor', 'none', ...
    'FontSize', 11, 'VerticalAlignment', 'middle');
end

function drawPanel(ax, tl, r, G, C, t, view, dt)
cla(ax); hold(ax, 'on'); axis(ax, 'equal'); axis(ax, [-view view -view view]);
set(ax, 'Color', [0.35 0.55 0.30], 'XTick', [], 'YTick', [], 'Box', 'on');
rw = 4 * C.LANE;
fill(ax, [-rw/2 rw/2 rw/2 -rw/2], [-200 -200 200 200], [0.62 0.62 0.62], 'EdgeColor', 'none');
fill(ax, [-200 200 200 -200], [-rw/2 -rw/2 rw/2 rw/2], [0.62 0.62 0.62], 'EdgeColor', 'none');
plot(ax, [-200 -rw/2], [0 0], 'w--', [rw/2 200], [0 0], 'w--', 'LineWidth', 1);
plot(ax, [0 0], [-200 -rw/2], 'w--', [0 0], [rw/2 200], 'w--', 'LineWidth', 1);
plot(ax, [-C.STOP -C.STOP], [0 rw/2], 'w-', [C.STOP C.STOP], [-rw/2 0], 'w-', 'LineWidth', 3);
plot(ax, [-rw/2 0], [-C.STOP -C.STOP], 'w-', [0 rw/2], [C.STOP C.STOP], 'w-', 'LineWidth', 3);
col = struct('green', [0 0.8 0], 'yellow', [1 0.85 0], 'red', [0.9 0 0]);
ew = signal_info(t, 'E', C); ns = signal_info(t, 'N', C);
lamp(ax, -C.STOP - 3, rw/2 + 2, col.(ew)); lamp(ax, C.STOP + 3, -rw/2 - 2, col.(ew));
lamp(ax, -rw/2 - 2, -C.STOP - 3, col.(ns)); lamp(ax, rw/2 + 2, C.STOP + 3, col.(ns));
if strcmp(r.cfg.coord, 'rsu')
    rectangle(ax, 'Position', [rw/2 + 4, rw/2 + 4, 3, 3], 'FaceColor', [0.2 0.2 0.2], 'EdgeColor', 'k');
    text(ax, rw/2 + 5.5, rw/2 + 9, 'RSU', 'HorizontalAlignment', 'center', 'FontSize', 8, 'FontWeight', 'bold');
end

% vehicles at time t, interpolated between the logged steps
k0 = floor(t / dt + 1e-9); w = t / dt - k0;
A = r.LOG(r.k == k0, :); B = r.LOG(r.k == k0 + 1, :);
dirCol = struct('E', [0.10 0.35 0.85], 'W', [0.90 0.75 0.10], 'N', [0.85 0.15 0.15], 'S', [0.15 0.65 0.20]);
L = C.VEH_L; W = C.VEH_W;
cx = [-L/2 L/2 L/2 -L/2]; cy = [-W/2 -W/2 W/2 W/2];
for i = 1:size(A, 1)
    id = A(i, 2); s = A(i, 3);
    j = find(B(:, 2) == id, 1);
    if ~isempty(j) && B(j, 6) == A(i, 6), s = s + w * (B(j, 3) - s); end
    P = G.P(A(i, 6));
    if s > P.len, continue; end
    [x, y, psi] = geom_pose(P, s - L/2);
    jv = find(r.ids == id, 1);
    mv = r.mov(jv);
    switch mv
        case 'L', fc = [0.55 0.55 0.55];
        case 'R', fc = [0.55 0.10 0.60];
        otherwise, fc = dirCol.(r.app(jv));
    end
    lw = 0.5; if ~A(i, 7), lw = 2.2; end
    patch(ax, x + cx * cos(psi) - cy * sin(psi), y + cx * sin(psi) + cy * cos(psi), fc, ...
          'EdgeColor', 'k', 'LineWidth', lw);
    if mv == 'R' && s >= P.s_stop && s <= P.s_wait + 0.05 && A(i, 4) < 1
        plot(ax, x, y, 'o', 'MarkerSize', 5, 'MarkerFaceColor', [1 0.9 0], 'MarkerEdgeColor', 'k');
    end
end
nEx = sum(r.t_exit <= t); nRt = sum(r.t_exit <= t & r.mov == 'R');
set(tl, 'String', sprintf('%s     exited %d   right turns %d', r.title, nEx, nRt));
end

function lamp(ax, x, y, c)
rectangle(ax, 'Position', [x - 1.2, y - 1.2, 2.4, 2.4], 'Curvature', [1 1], 'FaceColor', c, 'EdgeColor', 'k');
end
