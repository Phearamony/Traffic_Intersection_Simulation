function a = ctrl_idm(v, v0, gap, vl, prm)
%CTRL_IDM  Intelligent Driver Model acceleration.
%   a = ctrl_idm(v, v0, gap, vl, prm)
%   v, v0   speed and desired speed [m/s]; gap bumper-to-bumper gap [m]
%   (Inf = free road); vl speed of the vehicle (or stop point) ahead;
%   prm.a, prm.b, prm.s0, prm.T  IDM parameters of this driver.
%   a_IDM = a [1 - (v/v0)^4 - (s*/gap)^2],  s* = s0 + vT + v(v - vl)/(2 sqrt(ab))
%   Limited to [-8, prm.a].
v0 = max(v0, 0.1);
free = 1 - (v / v0)^4;
if isfinite(gap)
    ss = prm.s0 + max(v * prm.T + v * (v - vl) / (2 * sqrt(prm.a * prm.b)), 0);
    inter = (ss / max(gap, 0.1))^2;
else
    inter = 0;
end
a = prm.a * (free - inter);
a = min(max(a, -8), prm.a);
end
