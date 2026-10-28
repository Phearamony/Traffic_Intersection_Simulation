function t = travel_time(d, v0, a, vmax)
%TRAVEL_TIME  Time to cover d metres from speed v0, accelerating at a up to vmax.
if d <= 0, t = 0; return; end
if a <= 1e-6, t = d / max(v0, 0.1); return; end
v0 = max(min(v0, vmax), 0);
t_acc = (vmax - v0) / a;
d_acc = v0 * t_acc + 0.5 * a * t_acc^2;
if d <= d_acc
    t = (-v0 + sqrt(v0^2 + 2 * a * d)) / a;
else
    t = t_acc + (d - d_acc) / vmax;
end
end
