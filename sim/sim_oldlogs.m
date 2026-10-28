function [LightLog, CarLogE, CarLogW, FuelLog] = sim_oldlogs(R)
%SIM_OLDLOGS  Engine results in the CarLog / LightLog / FuelLog formats of
%   the earlier scripts, so the earlier plotting functions can be reused.
%   CarLog X is set so that the old distance-to-stop-line formulas hold:
%   approach E: X = s - 150 (the old code used max(0, -10 - X));
%   approach W: X = 150 - s (old: max(0, X - 10)). Turning vehicles keep
%   this coordinate along their path, so every line ends at the stop line.
G = R.G; C = G.C; LOG = R.log;
t = unique(LOG(:, 1));
LightLog = struct('Time', num2cell(t'), 'EW', '', 'NS', '');
for k = 1:numel(t)
    LightLog(k).EW = signal_info(t(k), 'E', C);
    LightLog(k).NS = signal_info(t(k), 'N', C);
end
v = R.veh; ids = [v.id];
[~, loc] = ismember(LOG(:, 2), ids);
appv = [v.app]; movv = [v.mov];
app = appv(loc)'; mov = movv(loc)';
CarLogE = mk(LOG(app == 'E', :), mov(app == 'E'), 'E', C);
CarLogW = mk(LOG(app == 'W', :), mov(app == 'W'), 'W', C);

ex = find([v.exited]);
if isempty(ex)
    FuelLog = struct('ID', {}, 'Dir', {}, 'TurnRight', {}, 'TurnedRight', {}, 'TurnLeft', {}, ...
        'Exited', {}, 'fuel_total', {}, 'idle_gap_time', {}, 'idle_red_time', {});
    return;
end
FuelLog = struct('ID', num2cell([v(ex).id]), 'Dir', num2cell([v(ex).app]), ...
    'TurnRight', num2cell(double([v(ex).mov] == 'R')), ...
    'TurnedRight', num2cell(double([v(ex).mov] == 'R')), ...
    'TurnLeft', num2cell(double([v(ex).mov] == 'L')), 'Exited', num2cell(true(1, numel(ex))), ...
    'fuel_total', num2cell([v(ex).fuel]), 'idle_gap_time', num2cell([v(ex).idle_gap]), ...
    'idle_red_time', num2cell([v(ex).idle_red]));
end

function CL = mk(L, mov, ap, C)
if isempty(L)
    CL = struct('Time', {}, 'ID', {}, 'Dir', {}, 'TurnLeft', {}, 'TurnRight', {}, ...
                'X', {}, 'Y', {}, 'V', {}, 'Ac', {});
    return;
end
if ap == 'E', X = L(:, 3) - C.L_APP; Y = C.LANE * ones(size(X));
else, X = C.L_APP - L(:, 3); Y = -C.LANE * ones(size(X)); end
n = size(L, 1);
CL = struct('Time', num2cell(L(:, 1)'), 'ID', num2cell(L(:, 2)'), 'Dir', repmat({ap}, 1, n), ...
    'TurnLeft', num2cell(double(mov' == 'L')), 'TurnRight', num2cell(double(mov' == 'R')), ...
    'X', num2cell(X'), 'Y', num2cell(Y'), 'V', num2cell(L(:, 4)'), 'Ac', num2cell(L(:, 5)'));
end
