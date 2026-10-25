function R = compute_turn_safety(CarLog)
%COMPUTE_TURN_SAFETY  Time separation between right turns and opposing traffic.
%
%   For every right turn in CarLog (a vehicle with TurnRight = 1 whose Dir
%   changes between two consecutive log entries), the turn instant t_turn
%   is the time of the first entry with the new direction. For every
%   opposing straight vehicle, t_pass is the time it passes the
%   intersection centre (linear interpolation between log samples).
%
%   Separation of one turn = min over opposing vehicles of |t_pass - t_turn|.
%   This is a post-encroachment-time style measure at the conflict area:
%   small values mean the turner and an opposing vehicle used the conflict
%   area close together in time.
%
%   R.n_turns    number of right turns found
%   R.sep_min    smallest separation over all turns [s]   (NaN if none)
%   R.sep_mean   mean separation over turns             [s]
%   R.n_below2   number of turns with separation < 2 s

R = struct('n_turns', 0, 'sep_min', NaN, 'sep_mean', NaN, 'n_below2', 0);
if isempty(CarLog), return; end

T   = [CarLog.Time];
ID  = [CarLog.ID];
TR  = [CarLog.TurnRight];
TL  = [CarLog.TurnLeft];
DIR = {CarLog.Dir};
X   = [CarLog.X];
Y   = [CarLog.Y];

ids = unique(ID);

% ---- passing times of straight vehicles, per approach ----
pass = struct('E', [], 'W', [], 'N', [], 'S', []);
for k = 1:numel(ids)
    m = find(ID == ids(k));
    if any(TR(m)) || any(TL(m)), continue; end
    [~, o] = sort(T(m)); m = m(o);
    d = DIR{m(1)};
    switch d
        case 'E', p = X(m);       % moves +x, centre at x = 0
        case 'W', p = -X(m);      % moves -x
        case 'N', p = Y(m);       % moves +y
        case 'S', p = -Y(m);      % moves -y
        otherwise, continue;
    end
    j = find(p(1:end-1) < 0 & p(2:end) >= 0, 1);
    if isempty(j), continue; end
    t0 = T(m(j)); t1 = T(m(j+1)); p0 = p(j); p1 = p(j+1);
    tp = t0 + (0 - p0) * (t1 - t0) / max(p1 - p0, eps);
    pass.(d)(end+1) = tp;
end

% ---- right-turn instants and separations ----
opp = struct('E', 'W', 'W', 'E', 'N', 'S', 'S', 'N');
sep = [];
for k = 1:numel(ids)
    m = find(ID == ids(k));
    if ~any(TR(m)), continue; end
    [~, o] = sort(T(m)); m = m(o);
    for j = 2:numel(m)
        if ~strcmp(DIR{m(j)}, DIR{m(j-1)})
            d0 = DIR{m(j-1)};
            if ~isfield(opp, d0), break; end
            tp = pass.(opp.(d0));
            if ~isempty(tp)
                sep(end+1) = min(abs(tp - T(m(j)))); %#ok<AGROW>
            end
            break;
        end
    end
end

R.n_turns = numel(sep);
if ~isempty(sep)
    R.sep_min  = min(sep);
    R.sep_mean = mean(sep);
    R.n_below2 = sum(sep < 2.0);
end
end
