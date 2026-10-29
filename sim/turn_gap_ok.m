function [ok, why] = turn_gap_ok(i, V, G, sig, mode, margin)
%TURN_GAP_OK  Gap acceptance of right-turner i against the opposite approach.
%
%   ok = turn_gap_ok(i, V, G, sig, mode, margin)
%   For each conflict zone (one per opposing movement) the turner needs
%   t_clear seconds until its rear has left that zone (from its current
%   position and speed, accelerating at 1.5 m/s^2 up to the arc speed). Every opposing straight or left-turning vehicle
%   that has not yet cleared its own zone must reach that zone no earlier
%   than t_clear + margin. A vehicle that will have left the zone at least
%   PET_MIN before the turner enters it does not matter, and a vehicle that
%   has to stop at its red (or yellow) light is ignored.
%
%   mode 'human'  arrival time at constant current speed (how a driver judges
%                 it); margin = reaction time + the driver's own gap margin
%   mode 'cav'    the vehicle keeps its current acceleration (known from V2V
%                 or on-board sensing) up to its desired speed; a braking
%                 vehicle is taken at constant speed; margin = PET_MIN; a
%                 vehicle that has just cleared the zone must have left it
%                 at least PET_MIN before the turner enters
%   sig.(app)     signal state of each approach ('green','yellow','red')
%   Opposing straight or left-turning vehicles behind an opposing right-turner that stands
%   at its waiting point are blocked by it: they arrive at the earliest
%   after it has left their lane, and a blocked vehicle that can still stop
%   before the conflict zone yields to a turner already in the junction, so
%   it is ignored (this is what allows a mutual right turn).
%   In mode 'cav', opposing vehicles also keep the order of their shared
%   approach lane: a vehicle cannot reach the junction before the vehicle
%   ahead of it in that lane has left the lane, whatever path either takes
%   (a straight vehicle behind a stopped or held left-turner is not free).

PET_MIN = 1.5;
why = [];
C = G.C; L = C.VEH_L;
k = V(i).p;
rows = G.oppZ{k};
ok = true;
if isempty(rows), return; end
P = G.P(k);
vt = min(P.vcurve(P.kappa > 0));                 % arc speed

pk = [V.p];
% an opposing right-turner standing in the junction (past its stop line,
% waiting for its gap) blocks its lane:
% the vehicles behind it can only come after it has started and left the
% lane (assumed to start now, from rest, which is the earliest case)
oa = G.P(rows(1, 1)).app;
Po = G.P(G.idx.([oa 'R']));
wt = find([V.app] == oa & [V.mov] == 'R' & ~[V.committed] & [V.v] < 0.5 & ...
          [V.s] >= Po.s_stop);
sBlock = -inf; t_vac = 0;
if ~isempty(wt)
    [sBlock, kb] = max([V(wt).s]);
    vto = min(Po.vcurve(Po.kappa > 0));
    t_vac = travel_time(Po.s_div + L - V(wt(kb)).s, 0, 1.5, vto);
end
% shared approach lane of the opposite approach (mode 'cav'): the earliest
% time each vehicle can leave it (front at s_div), in lane order
leaveAhead = containers.Map('KeyType', 'double', 'ValueType', 'double');
if strcmp(mode, 'cav')
    % the lane is shared up to the first point where a path leaves it
    sDiv = min([Po.s_div, G.P(G.idx.([oa 'L'])).s_div, G.P(G.idx.([oa 'S'])).s_div]);
    ln = find([V.app] == oa & [V.s] < sDiv);
    [~, o] = sort([V(ln).s], 'descend'); ln = ln(o);
    tl = -inf;
    for u = ln
        leaveAhead(V(u).id) = tl;                  % the vehicle ahead leaves the lane at tl
        if strcmp(sig.(oa), 'red') && V(u).s < G.P(V(u).p).s_stop, tl = inf; continue; end
        vu = V(u).v; au = max(V(u).a, 0);
        if vu < 2, au = max(au, 1.5); end
        tu = travel_time(sDiv - V(u).s, vu, au, max(V(u).Vd, vu));
        m = V(u).msg;
        if ~isempty(fieldnames(m)) && isfield(m, 'hard') && m.hard && isfinite(m.tgt) && V(u).s < m.spt ...
                && tnow(V) < m.tgt && tnow(V) - V(u).msg_rx <= 10
            tu = max(tu, (m.tgt - tnow(V)) + max(sDiv - m.spt, 0) / max(vu, 6));
        end
        if V(u).mov == 'R' && V(u).s >= Po.s_stop - 0.05, tu = max(tu, t_vac); end
        tl = max(tu, tl + 1.0);
    end
end
for r = 1:size(rows, 1)
    q = rows(r, 1); z0q = rows(r, 4); z1q = rows(r, 5);
    % the turner's entry / clearing time of this conflict zone
    t_enter = travel_time(rows(r, 2) - V(i).s, V(i).v, 1.5, vt);
    t_clear = travel_time(rows(r, 3) + L - V(i).s, V(i).v, 1.5, vt);
    js = find(pk == q);
    [~, o] = sort([V(js).s], 'descend'); js = js(o);      % closest to the zone first
    tprev = -inf;
    for j = js
        sj = V(j).s; vj = V(j).v;
        if sj - L > z1q
            % already cleared; a CAV still keeps PET_MIN after its rear left
            if strcmp(mode, 'cav') && (sj - L - z1q) / max(vj, 1) + t_enter < PET_MIN
                ok = false; why = [V(j).id, -1, PET_MIN, sj, vj, V(j).a]; return;
            end
            continue;
        end
        blocked = sj < sBlock;          % straight or left: behind the turner in its lane
        if sj < G.P(q).s_stop + 0.05
            st = sig.(G.P(q).app);
            if strcmp(st, 'red'), continue; end
            if strcmp(st, 'yellow') && (G.P(q).s_stop - sj) >= vj^2 / (2 * 3.0)
                continue;                                  % will stop
            end
        end
        if sj >= z0q
            t_arr = 0;
        elseif strcmp(mode, 'cav')
            aj = max(V(j).a, 0);
            if vj < 2, aj = max(aj, 1.5); end              % may start from rest
            t_arr = travel_time(z0q - sj, vj, aj, max(V(j).Vd, vj));
            % a vehicle held back by an RSU arrival-time target (known over V2X)
            m = V(j).msg;
            if ~isempty(fieldnames(m)) && isfield(m, 'hard') && m.hard && isfinite(m.tgt) && sj < m.spt ...
                    && tnow(V) < m.tgt && tnow(V) - V(j).msg_rx <= 10
                t_arr = max(t_arr, (m.tgt - tnow(V)) + (z0q - m.spt) / max(vj, 6));
            end
        else
            t_arr = (z0q - sj) / max(vj, 0.1);
        end
        if blocked
            % it yields to a turner in the junction (sim_run, step 6) if it
            % can still stop before the zone; otherwise it counts as usual
            % (the yielding rule in sim_run uses 4 m/s^2 too and stops 0.5 m before
            % the zone; this test asks for that 0.5 m as well, so it is slightly stricter)
            if (z0q - 0.5 - sj) >= vj^2 / (2 * 4.0), continue; end
            t_arr = max(t_arr, t_vac + travel_time(z0q - sj, vj, 2.0, max(V(j).Vd, 1)));
        end
        % a vehicle cannot pass the one ahead of it in its lane
        t_arr = max(t_arr, tprev + 1.0);
        if isKey(leaveAhead, V(j).id), t_arr = max(t_arr, leaveAhead(V(j).id) + 1.0); end
        tprev = t_arr;
        % passes completely before the turner gets there
        t_out = t_arr + (z1q - max(sj, z0q) + L) / max(vj, 3);
        if t_out + PET_MIN <= t_enter, continue; end
        if t_arr < t_clear + margin
            ok = false; why = [V(j).id, t_arr, t_clear + margin, sj, vj, V(j).a]; return;
        end
    end
end
end

function t = tnow(V)
% current time is stored by the engine in every vehicle (field t_now)
t = V(1).t_now;
end
