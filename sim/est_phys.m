function tau = est_phys(d, v, n, t, tg0, tend, theta, C)
%EST_PHYS  Queue-discharge physics estimate of the time to the stop line.
%
%   tau = est_phys(d, v, n, t, tg0, tend, theta, C)
%   d, v, n   distance [m], speed [m/s] and number of vehicles ahead
%   t, tg0, tend  current time, start of the current green (if green or
%             yellow) or of the next green (if red), and the end of that
%             green's yellow [s]; scalars or vectors of the same size as d
%   theta     [t_L, h_s, a_p]: start-up lost time, saturation discharge
%             headway, acceleration of the kinematic part
%
%   kinematic   tau_kin = time to cover d from v, accelerating at a_p up to
%               the typical desired speed C.V_DES0
%   queue       tau_q   = max(tg0 - t + t_L, 0) + n h_s
%               (the queue starts t_L after green; the n vehicles ahead
%               pass the stop line h_s apart)
%   estimate    tau     = max(tau_kin, tau_q)
%   left over   if t + tau is after the yellow ends, the vehicle waits for
%               the following green: tau = tg0 + cycle - t + t_L + n_left h_s,
%               n_left = vehicles ahead that this green could not serve
tL = theta(1); hs = theta(2); ap = theta(3);
d = d(:)'; v = v(:)'; n = n(:)';
e = ones(size(d));
t = t(:)' .* e; tg0 = tg0(:)' .* e; tend = tend(:)' .* e;
tk = travel_time_vec(d, v, ap, C.V_DES0);
tq = max(tg0 - t + tL, 0) + n * hs;
tau = max(tk, tq);
late = t + tau > tend;
if any(late)
    served = floor(max(tend - max(tg0, t) - tL, 0) / hs);
    nleft = max(n - served, 0);
    alt = max(tg0 + C.CYCLE - t + tL + nleft * hs, tk);
    tau(late) = alt(late);
end
end

function t = travel_time_vec(d, v0, a, vmax)
v0 = max(min(v0, vmax), 0);
t_acc = (vmax - v0) / a;
d_acc = v0 .* t_acc + 0.5 * a * t_acc.^2;
t = t_acc + (d - d_acc) / vmax;
m = d <= d_acc;
t(m) = (-v0(m) + sqrt(v0(m).^2 + 2 * a * d(m))) / a;
t(d <= 0) = 0;
end
