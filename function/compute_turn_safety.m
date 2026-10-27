function R = compute_turn_safety(CarLog)
%COMPUTE_TURN_SAFETY  Oncoming gap accepted by each right turn.
%
%   For every right turn in CarLog (a vehicle with TurnRight = 1 whose Dir
%   changes between two consecutive log entries), the turn starts at
%   t_turn, the time of the first entry with the new direction. For every
%   opposing straight vehicle, t_pass is the time it passes the
%   intersection centre (linear interpolation between log samples).
%
%   Oncoming gap of one turn = t_pass - t_turn of the FIRST opposing
%   vehicle that passes after the turn starts (capped at GAP_CAP s when
%   none comes). The turn itself takes about 2 s from the waiting point
%   (same geometry as local_gap_ok), so a gap under 2 s means the oncoming
%   vehicle reaches the conflict area while the turner is still in it: a
%   conflict. Vehicles that passed just BEFORE the turn are not counted:
%   in this model the turner starts from the waiting point after they have
%   left the conflict point, which is ordinary behaviour, not a conflict.
%
%   R.n_turns    number of right turns found
%   R.sep_min    smallest oncoming gap over all turns [s]   (NaN if no turn)
%   R.sep_mean   mean oncoming gap over turns          [s]
%   R.n_below2   number of turns with an oncoming gap < 2 s (conflicts)
%   (field names kept from the earlier separation measure)

GAP_CAP = 30;   % [s]

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

% ---- right-turn instants and oncoming gaps ----
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
            t_turn = T(m(j));
            tp = pass.(opp.(d0));
            tp = tp(tp >= t_turn);
            if isempty(tp)
                g = GAP_CAP;
            else
                g = min(min(tp) - t_turn, GAP_CAP);
            end
            sep(end+1) = g; %#ok<AGROW>
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
