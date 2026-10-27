function ok = local_gap_ok(car, opp, oppDir, turn_release, bigGap, accelAware)
%LOCAL_GAP_OK  Driver's own gap acceptance for a right-turner at its waiting point.
%   Same rule as the uncoordinated baseline C1 (main_human_driving_plot.m):
%   the turn is accepted when every opposing vehicle that has not yet
%   passed the conflict point and is not itself turning right is at least
%   minGapDist away and arrives later than the turn time plus a reaction
%   time and a margin. Used for human drivers in mixed traffic and as the
%   fallback of a CAV that has no fresh RSU advisory.
%
%   opp     array of Car objects on the opposing approach
%   oppDir  'E','W','N','S' : direction the opposing vehicles travel
%   bigGap  extra margin [s] on top of turn time + reaction time
%           (default 4 s, the C1 human rule; 0 = on-board check of a CAV
%           that the RSU told to proceed)
%   accelAware  true = arrival time of each opposing vehicle assumes it
%           accelerates (1.5 m/s^2, the IDM maximum) up to its desired
%           speed, the earliest it can arrive; false (default) = constant
%           current speed, as a human driver judges it. The on-board check
%           of a CAV uses true: a vehicle that the RSU held back speeds up
%           again once it is past its stop line.

minGapDist = 6.0;   % [m]
rt_react   = 0.7;   % [s]
rt_bigGap  = 4.0;   % [s]
if nargin >= 5 && ~isempty(bigGap), rt_bigGap = bigGap; end
if nargin < 6 || isempty(accelAware), accelAware = false; end

% turn time from the waiting point (same geometry as estimateTurnTime in C1)
turn_wait = 5;
flat = (turn_wait - turn_release) / 2;
arc  = sqrt(2 * flat^2);
v_turn = min((car.V + car.Vd) / 2, 3);
t_turn = (2 * flat + arc) / max(v_turn, 0.1);

ok = true;
if isempty(opp), return; end
switch oppDir
    case 'E', pos = [opp.X];  notPassed = pos < -turn_release; dist = -turn_release - pos;
    case 'W', pos = [opp.X];  notPassed = pos >  turn_release; dist = pos - turn_release;
    case 'N', pos = [opp.Y];  notPassed = pos < -turn_release; dist = -turn_release - pos;
    case 'S', pos = [opp.Y];  notPassed = pos >  turn_release; dist = pos - turn_release;
    otherwise, return;
end
conf = notPassed & ~[opp.TurnRight];
idx = find(conf);
if isempty(idx), return; end
for j = idx
    if accelAware
        t_arr = earliestArrival(dist(j), max(opp(j).V, 0), max(opp(j).Vd, 0.1), 1.5);
    else
        t_arr = dist(j) / max(opp(j).V, 0.1);
    end
    if numel(idx) == 1
        if dist(j) < minGapDist || t_arr < t_turn + rt_react + rt_bigGap
            ok = false; return;
        end
    else
        if dist(j) < minGapDist + 2 || t_arr < t_turn + rt_react + rt_bigGap
            ok = false; return;
        end
    end
end
end

function t = earliestArrival(d, v0, vmax, a)
% time to cover d [m] starting at v0, accelerating at a up to vmax
if d <= 0, t = 0; return; end
v0 = min(v0, vmax);
t_acc = (vmax - v0) / a;
d_acc = v0 * t_acc + 0.5 * a * t_acc^2;
if d <= d_acc
    t = (-v0 + sqrt(v0^2 + 2 * a * d)) / a;
else
    t = t_acc + (d - d_acc) / vmax;
end
end
