function ok = local_gap_ok(car, opp, oppDir, turn_release)
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

minGapDist = 6.0;   % [m]
rt_react   = 0.7;   % [s]
rt_bigGap  = 4.0;   % [s]

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
    t_arr = dist(j) / max(opp(j).V, 0.1);
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
