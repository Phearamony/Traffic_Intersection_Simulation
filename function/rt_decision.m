function canTurn = rt_decision(car, opp, oppDir, light, turn_release)
%RT_DECISION  Right-turn decision at the waiting point (RSU configurations).
%
%   The RSU can only add turn opportunities, never take one away:
%
%   1. A CAV whose fresh RSU advisory says PROCEED (+1) turns, after a short
%      on-board check that no opposing vehicle is about to reach the
%      conflict point during the turn (local_gap_ok with no extra margin,
%      assuming each opposing vehicle accelerates: earliest arrival).
%      The RSU created this gap by holding opposing CAVs back, so the
%      vehicle does not need the large margin of the human rule.
%   2. In every other case (advisory WAIT, no advisory, advisory lost or
%      older than Car.X2V_TIMEOUT, human driver in mixed traffic) the
%      vehicle uses the C1 gap-acceptance rule (local_gap_ok with the human
%      margin): a natural gap that is large enough is always taken, and the
%      vehicle clears the junction once its signal is no longer green.
%
%   So a turner in an RSU configuration has every opportunity a C1 driver
%   has, plus the gaps the RSU makes for it.

x2v = [];
if car.IsCAV
    x2v = car.GetLastX2V();
end
if ~strcmp(light, 'green')
    canTurn = true;          % end of green: clear the junction, as in C1
    return;
end
if ~isempty(x2v) && isfield(x2v, 'turn_signal') && ~isempty(x2v.turn_signal) ...
        && x2v.turn_signal == 1
    canTurn = local_gap_ok(car, opp, oppDir, turn_release, 0, true);
    if canTurn, return; end
end
canTurn = local_gap_ok(car, opp, oppDir, turn_release);
end
