function G = geom_build()
%GEOM_BUILD  Lane paths, conflict zones and waiting points of the intersection.
%
%   Japanese left-hand traffic, one lane per direction, lane width 3 m.
%   Approach E travels +x in the lane y = +1.5, W travels -x at y = -1.5,
%   N travels +y at x = -1.5, S travels -y at x = +1.5. Every approach has
%   three paths: straight (S), left turn (L) and right turn (R). Paths are
%   built for approach E and rotated for the others.
%
%   Path coordinate s [m] is the position of the vehicle FRONT along its
%   path, from s = 0 at the entry (150 m before the centre). The stop line
%   is at s = 140 (10 m before the centre) on every path.
%
%     straight  line through the junction to 170 m past the centre
%     right     line, circular arc of radius R_RIGHT (clockwise), line
%     left      line, circular arc of radius R_LEFT (counter-clockwise), line
%
%   Right turners from opposite approaches pass each other on the inside of
%   the centre (Japanese rule), so a mutual right turn is not a conflict.
%
%   Conflict zones are computed from the geometry: two paths of different
%   approaches conflict where their centre lines come closer than D_CONF.
%   For each pair the zone is an interval [z0, z1] of s on each path; a
%   vehicle occupies the zone from the time its front reaches z0 until its
%   rear (front - L) passes z1. Merges (two paths into the same exit lane)
%   get a zone of MERGE_LEN metres from the first contact.
%
%   G.P(k)        path k: name, app, mov, s, x, y, psi, kappa, len,
%                 s_stop, s_div (end of the shared approach lane),
%                 s_out (start of the exit line), w_out (its distance from
%                 the centre), out ('E','W','N','S' = exit road direction),
%                 vcurve(s) speed limit from the lateral-acceleration limit,
%                 s_wait (right turns: waiting point), s_ff (free-flow time)
%   G.idx.(name)  path index, e.g. G.idx.WR
%   G.Z           conflict zones: struct array with fields p, q (paths),
%                 zp = [z0 z1] on p, zq = [z0 z1] on q, type ('cross','merge')

C = geom_constants();
apps = {'E', 'N', 'W', 'S'};            % rotation 0, +90, 180, -90 degrees
rot  = [0, pi/2, pi, -pi/2];
movs = {'S', 'L', 'R'};

base = struct();
base.S = pathE_straight(C);
base.L = pathE_turn(C, 'L');
base.R = pathE_turn(C, 'R');

P = struct('name', {}, 'app', {}, 'mov', {}, 's', {}, 'x', {}, 'y', {}, 'psi', {}, ...
           'kappa', {}, 'len', {}, 's_stop', {}, 's_div', {}, 's_out', {}, 'w_out', {}, ...
           'out', {}, 'vcurve', {}, 's_wait', {}, 't_ff_unit', {});
idx = struct();
for a = 1:4
    th = rot(a);
    Rm = [cos(th) -sin(th); sin(th) cos(th)];
    for m = 1:3
        b = base.(movs{m});
        xy = Rm * [b.x; b.y];
        p = struct();
        p.name  = [apps{a} movs{m}];
        p.app   = apps{a};
        p.mov   = movs{m};
        p.s     = b.s;
        p.x     = xy(1,:);
        p.y     = xy(2,:);
        p.psi   = b.psi + th;
        p.kappa = b.kappa;
        p.len   = b.s(end);
        p.s_stop = C.L_APP - C.STOP;
        p.s_div  = b.s_div;
        p.s_out  = b.s_out;
        p.w_out  = b.w_out;
        p.out    = exitRoad(apps{a}, movs{m});
        vc = sqrt(C.A_LAT ./ max(abs(b.kappa), 1e-6));
        p.vcurve = min(vc, C.V_MAX);
        p.s_wait = NaN;
        p.t_ff_unit = NaN;
        P(end+1) = p; %#ok<AGROW>
        idx.(p.name) = numel(P);
    end
end

% ---------------- conflict zones ----------------
Z = struct('p', {}, 'q', {}, 'zp', {}, 'zq', {}, 'type', {});
for i = 1:numel(P)
    for j = i+1:numel(P)
        if P(i).app == P(j).app, continue; end
        if P(i).mov == 'R' && P(j).mov == 'R' && isOpposite(P(i).app, P(j).app)
            continue;                   % mutual right turn: pass on the inside
        end
        [zi, zj, typ] = zoneBetween(P(i), P(j), C);
        if isempty(zi), continue; end
        Z(end+1) = struct('p', i, 'q', j, 'zp', zi, 'zq', zj, 'type', typ); %#ok<AGROW>
    end
end

% waiting point of each right turn: just before it crosses the opposing
% straight lane (crossings with the perpendicular roads are separated by
% the signal)
opp = struct('E', 'W', 'W', 'E', 'N', 'S', 'S', 'N');
for k = 1:numel(P)
    if P(k).mov ~= 'R', continue; end
    q = idx.([opp.(P(k).app) 'S']);
    for n = 1:numel(Z)
        if Z(n).p == k && Z(n).q == q, P(k).s_wait = Z(n).zp(1) - C.WAIT_GAP; end
        if Z(n).q == k && Z(n).p == q, P(k).s_wait = Z(n).zq(1) - C.WAIT_GAP; end
    end
end

% conflicts of each right turn with the opposite approach (straight and
% left): rows [q, z0 on k, z1 on k, z0 on q, z1 on q]
oppZ = cell(1, numel(P));
for k = 1:numel(P)
    rows = zeros(0, 5);
    if P(k).mov == 'R'
        oa = opp.(P(k).app);
        for n = 1:numel(Z)
            if Z(n).p == k, q = Z(n).q; zk = Z(n).zp; zq = Z(n).zq;
            elseif Z(n).q == k, q = Z(n).p; zk = Z(n).zq; zq = Z(n).zp;
            else, continue; end
            if P(q).app ~= oa || P(q).mov == 'R', continue; end
            rows(end+1, :) = [q, zk, zq]; %#ok<AGROW>
        end
    end
    oppZ{k} = rows;
end

% merge points: MZ(p, q) = start of the merge zone on p (NaN if p and q
% do not merge). Used by find_leaders to order vehicles before a merge.
MZ = nan(numel(P));
for n = 1:numel(Z)
    if strcmp(Z(n).type, 'merge')
        MZ(Z(n).p, Z(n).q) = Z(n).zp(1);
        MZ(Z(n).q, Z(n).p) = Z(n).zq(1);
    end
end

G.P = P; G.idx = idx; G.Z = Z; G.C = C; G.oppZ = oppZ; G.MZ = MZ;
G.apps = apps;
end

% ======================================================================
function p = pathE_straight(C)
s = 0:C.DS:(C.L_APP + C.L_EXIT);
p.s = s;
p.x = -C.L_APP + s;
p.y = C.LANE * ones(size(s));
p.psi = zeros(size(s));
p.kappa = zeros(size(s));
p.s_div = s(end);            % never leaves the approach lane
p.s_out = C.L_APP;           % exit coordinate measured from the centre
p.w_out = 0;
end

function p = pathE_turn(C, mov)
% line from the entry to the arc start, arc, line to the exit end
if mov == 'R'
    R = C.R_RIGHT;
    x0 = C.LANE - R;                 % arc start x (lane y = +1.5)
    cen = [x0, C.LANE - R];          % clockwise: centre to the right (south)
    th0 = pi/2; th1 = 0; sgn = -1;   % angle runs 90 -> 0 degrees
else
    R = C.R_LEFT;
    x0 = -C.LANE - R;
    cen = [x0, C.LANE + R];          % counter-clockwise: centre to the left
    th0 = -pi/2; th1 = 0; sgn = +1;
end
L1 = x0 + C.L_APP;                   % length of the approach line
La = R * abs(th1 - th0);             % arc length
ex = [cen(1) + R*cos(th1), cen(2) + R*sin(th1)];   % arc end point
w0 = abs(ex(2));                     % distance of the arc end from the centre
L2 = C.L_EXIT - w0;                  % exit line length to 170 m from the centre
s = 0:C.DS:(L1 + La + L2);
x = zeros(size(s)); y = x; psi = x; kap = x;
for k = 1:numel(s)
    sk = s(k);
    if sk <= L1
        x(k) = -C.L_APP + sk; y(k) = C.LANE; psi(k) = 0; kap(k) = 0;
    elseif sk <= L1 + La
        th = th0 + sgn * (sk - L1) / R;
        x(k) = cen(1) + R*cos(th); y(k) = cen(2) + R*sin(th);
        psi(k) = sgn * (sk - L1) / R;
        kap(k) = 1 / R;
    else
        d = sk - L1 - La;
        psi(k) = sgn * pi/2;
        x(k) = ex(1); y(k) = ex(2) + sign(sgn) * d;
        kap(k) = 0;
    end
end
p.s = s; p.x = x; p.y = y; p.psi = psi; p.kappa = kap;
p.s_div = L1;               % leaves the approach lane at the arc start
p.s_out = L1 + La;          % exit line starts at the arc end
p.w_out = w0;
end

function [zi, zj, typ] = zoneBetween(Pi, Pj, C)
zi = []; zj = []; typ = '';
% only the part near the junction can conflict
mi = find(abs(Pi.x) < 15 & abs(Pi.y) < 15);
mj = find(abs(Pj.x) < 15 & abs(Pj.y) < 15);
if isempty(mi) || isempty(mj), return; end
dx = Pi.x(mi)' - Pj.x(mj);
dy = Pi.y(mi)' - Pj.y(mj);
D = sqrt(dx.^2 + dy.^2);
[ii, jj] = find(D < C.D_CONF);
if isempty(ii), return; end
si = Pi.s(mi(ii)); sj = Pj.s(mj(jj));
if Pi.out == Pj.out
    typ = 'merge';                                  % same exit lane
    zi = [min(si), min(si) + C.MERGE_LEN];
    zj = [min(sj), min(sj) + C.MERGE_LEN];
else
    typ = 'cross';
    zi = [min(si), max(si)];
    zj = [min(sj), max(sj)];
end
end

function o = exitRoad(app, mov)
% direction the vehicle travels after the junction
order = 'ENWS';                       % counter-clockwise headings
k = strfind(order, app);
switch mov
    case 'S', o = app;
    case 'L', o = order(mod(k, 4) + 1);          % counter-clockwise
    case 'R', o = order(mod(k - 2, 4) + 1);      % clockwise
end
end

function t = isOpposite(a, b)
t = any(strcmp([a b], {'EW', 'WE', 'NS', 'SN'}));
end
