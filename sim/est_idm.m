function tau = est_idm(view, F, app, t, G)
%EST_IDM  IDM forward-simulation estimate (the conference method).
%
%   tau = est_idm(view, F, app, t, G)
%   The RSU simulates the known vehicles of the approach forward with
%   nominal IDM parameters (a = 1.5, b = 2.5, s0 = 2 m, T = 1.5 s,
%   v0 = 25 m/s), dt = 0.5 s, for up to 120 s, with the signal plan:
%   during red, and during yellow when it can still stop, the first vehicle
%   before the stop line follows a standing obstacle at the stop line.
%   tau(q) is the time vehicle F.ids(q) first reaches the stop line (Inf if
%   not within 120 s). As in the conference method, a right-turner waiting
%   in the junction is not modelled (the RSU does not know when it goes).
C = G.C; L = C.VEH_L; dt = C.DT;
k = numel(F.ids);
tau = inf(1, k);
if k == 0, return; end
sStop = C.L_APP - C.STOP;
sObs = inf;
s = sStop - F.d; v = F.v;                      % closest first
a0 = 1.5; b0 = 2.5; s00 = 2.0; T0 = 1.5; v0 = 25;
done = false(1, k);
for step = 1:240
    tt = t + (step - 1) * dt;
    st = signal_info(tt, app, C);
    acc = zeros(1, k);
    for q = 1:k
        if q == 1, lead_s = sObs; lead_v = 0;
        else, lead_s = s(q-1); lead_v = v(q-1);
        end
        if s(q) < sStop && (strcmp(st, 'red') || ...
                (strcmp(st, 'yellow') && sStop - s(q) >= v(q)^2 / 6))
            if lead_s > sStop + L          % nobody between me and the stop line
                lead_s = sStop + L; lead_v = 0;
            end
        end
        gap = lead_s - L - s(q);
        ss = s00 + max(v(q) * T0 + v(q) * (v(q) - lead_v) / (2 * sqrt(a0 * b0)), 0);
        acc(q) = a0 * (1 - (v(q) / v0)^4 - (ss / max(gap, 0.1))^2);
    end
    acc = min(max(acc, -8), a0);
    vn = max(v + acc * dt, 0);
    sn = s + 0.5 * (v + vn) * dt;
    cr = ~done & s < sStop & sn >= sStop;
    tau(cr) = (step - 1) * dt + dt * (sStop - s(cr)) ./ max(sn(cr) - s(cr), 1e-9);
    done = done | sn >= sStop;
    s = sn; v = vn;
    if all(done), break; end
end
end
