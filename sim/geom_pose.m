function [x, y, psi] = geom_pose(P, s)
%GEOM_POSE  Position and heading at path coordinate s (vector allowed).
s = min(max(s, 0), P.len);
k = s / (P.s(2) - P.s(1)) + 1;
k0 = min(floor(k), numel(P.s) - 1);
f = k - k0;
x   = P.x(k0)   .* (1 - f) + P.x(k0 + 1)   .* f;
y   = P.y(k0)   .* (1 - f) + P.y(k0 + 1)   .* f;
psi = P.psi(k0) .* (1 - f) + P.psi(k0 + 1) .* f;
end
