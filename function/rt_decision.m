function canTurn = rt_decision(car, opp, oppDir, light, turn_release)
%RT_DECISION  Right-turn decision at the waiting point (RSU configurations).
%   Two-tier rule:
%   1. A CAV with a fresh RSU advisory (+1 proceed / -1 wait) follows it.
%   2. Otherwise (human driver in mixed traffic, advisory lost or older than
%      Car.X2V_TIMEOUT, or the RSU has no plan for it) the driver decides
%      alone with the C1 gap-acceptance rule (local_gap_ok), and also clears
%      the junction once its signal is no longer green, as in C1.
x2v = [];
if car.IsCAV
    x2v = car.GetLastX2V();
end
if ~isempty(x2v) && isfield(x2v, 'turn_signal') && ~isempty(x2v.turn_signal) && x2v.turn_signal ~= 0
    canTurn = (x2v.turn_signal == 1);
    return;
end
canTurn = ~strcmp(light, 'green') || local_gap_ok(car, opp, oppDir, turn_release);
end
