function acap = curve_cap(P, s, v, b)
%CURVE_CAP  Deceleration needed now to reach every upcoming curve speed.
%   acap = curve_cap(P, s, v, b) returns +Inf when no braking is needed,
%   otherwise -b_req, the constant deceleration that brings the vehicle
%   from v to the curve speed at the start of the slowest point within the
%   next 60 m. b is the comfortable deceleration (used only to decide when
%   to start: braking starts once b_req exceeds 0.5 b).
acap = inf;
ds = P.s(2) - P.s(1);
k0 = floor(s / ds) + 1;
k1 = min(numel(P.s), k0 + round(60 / ds));
if k0 >= k1, return; end
vc = P.vcurve(k0:k1);
d  = P.s(k0:k1) - s;
m  = vc < v & d > 0.5;
if ~any(m), return; end
breq = max((v^2 - vc(m).^2) ./ (2 * d(m)));
if breq > 0.5 * b
    acap = -breq;
end
end
