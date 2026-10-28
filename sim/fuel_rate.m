function fc = fuel_rate(v, a)
%FUEL_RATE  Instantaneous fuel rate [mL/s] (same model as the earlier Car.m).
%   fc = b0 + b1 v + b2 v^2 + b3 v^3 + max(a,0) (c0 + c1 v + c2 v^2), floored at b0
b0 = 0.156;      b1 = 2.450e-2;
b2 = -7.415e-4;  b3 = 5.975e-5;
c0 = 0.07224;    c1 = 9.681e-2;   c2 = 1.075e-3;
v = max(v, 0); u = max(a, 0);
fc = b0 + b1*v + b2*v.^2 + b3*v.^3 + u.*(c0 + c1*v + c2*v.^2);
fc = max(fc, b0);
end
