function [gap, vl, lead, al] = find_leaders(V, G)
%FIND_LEADERS  Bumper-to-bumper gap to the vehicle ahead of every vehicle.
%
%   Two vehicles interact when they share a lane:
%   1. the approach lane: same approach, and the vehicle ahead has not yet
%      left the lane the follower is in (its rear is still before the point
%      where the two paths separate);
%   2. the exit lane: both paths end on the same exit road. Positions are
%      compared as the distance w from the centre along the exit road; a
%      vehicle still in its arc gets w = w_out - (s_out - s), the distance
%      it still has to drive to the exit line. This also handles merging
%      (a right turn and an opposite left turn into the same exit).
%   gap(i)  [m] Inf when nothing is ahead;  vl(i) speed of that vehicle;
%   lead(i) index of that vehicle (0 if none); al(i) its acceleration.

n = numel(V);
gap = inf(1, n); vl = zeros(1, n); lead = zeros(1, n); al = zeros(1, n);
if n < 2, return; end
L = G.C.VEH_L;
pk   = [V.p];
s    = [V.s];
v    = [V.v];
app  = [G.P(pk).app];
outr = [G.P(pk).out];
sdiv = [G.P(pk).s_div];
sout = [G.P(pk).s_out];
wout = [G.P(pk).w_out];
w    = s - sout + wout;              % position along the exit road
mov  = [G.P(pk).mov];

for i = 1:n
    best = inf; bv = 0; bj = 0;
    % --- approach lane ---
    if s(i) <= sdiv(i)
        m = find(app == app(i) & s > s(i));
        for j = m
            shared = min(sdiv(i), sdiv(j));
            if s(j) - L <= shared            % its rear is still in my lane
                g = s(j) - L - s(i);
                if g < best, best = g; bv = v(j); bj = j; end
            end
        end
    end
    % --- exit lane (only near or after the junction) ---
    if s(i) >= sdiv(i) - 20 || mov(i) == 'S'
        m = find(outr == outr(i) & pk ~= pk(i) | (outr == outr(i) & w > w(i)));
        for j = m
            if j == i, continue; end
            % the other vehicle must already be at or past its divergence
            if mov(j) ~= 'S' && s(j) < sdiv(j), continue; end
            if mov(j) == 'S' && app(j) ~= app(i) && w(j) < -5, continue; end
            zi = G.MZ(pk(i), pk(j));
            % a right-turner that has not committed has not entered the
            % junction for good: it gives way, it is nobody's leader there
            if ~isnan(zi) && mov(j) == 'R' && ~V(j).committed, continue; end
            if ~isnan(zi) && (s(i) < zi + 8 || s(j) < G.MZ(pk(j), pk(i)) + 8)
                % before / at a merge: order by distance to the merge point
                rj = s(j) - G.MZ(pk(j), pk(i)); ri = s(i) - zi;
                if rj <= ri, continue; end
                g = rj - L - ri;
            else
                if w(j) <= w(i), continue; end
                g = w(j) - L - w(i);
            end
            if g < best, best = g; bv = v(j); bj = j; end
        end
    end
    gap(i) = best; vl(i) = bv; lead(i) = bj;
    if bj > 0, al(i) = V(bj).a; end
end
end
