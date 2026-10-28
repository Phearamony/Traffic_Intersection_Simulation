function [st, tg0, tend] = signal_info(t, app, C)
%SIGNAL_INFO  Signal state of an approach at time t and its timing.
%   [st, tg0, tend] = signal_info(t, app, C)
%   st    'green', 'yellow' or 'red'
%   tg0   start of the current green (if green or yellow) or of the next
%         green (if red), absolute time [s]
%   tend  end of the current or next yellow, absolute time [s]
%   The plan is the fixed-time plan of geom_constants (cycle 120 s).
if any(app == 'EW'), g = C.G_EW; y = C.Y_EW; else, g = C.G_NS; y = C.Y_NS; end
cyc = floor(t / C.CYCLE) * C.CYCLE;
tc  = t - cyc;
if tc >= g(1) && tc < g(2)
    st = 'green';  tg0 = cyc + g(1); tend = cyc + y(2);
elseif tc >= y(1) && tc < y(2)
    st = 'yellow'; tg0 = cyc + g(1); tend = cyc + y(2);
else
    st = 'red';
    if tc < g(1), tg0 = cyc + g(1); else, tg0 = cyc + C.CYCLE + g(1); end
    tend = tg0 + (y(2) - g(1));
end
end
