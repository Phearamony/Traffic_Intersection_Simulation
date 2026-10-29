function g = signal_feature(t, axisName)
%SIGNAL_FEATURE  Signal-timing input of the GPR arrival-time model.
%
%   g = signal_feature(t, 'EW')   or   signal_feature(t, 'NS')
%   t is the simulation time [s]. The fixed-time plan is the one used by
%   every main script: cycle 120 s, NS green 0-54.5 s, yellow to 58 s,
%   all-red to 60 s, EW green 60-114.5 s, yellow to 118 s, all-red to 120 s.
%
%   g > 0   the axis has green or yellow: g = time left until its yellow ends
%   g < 0   the axis has red: -g = time until its next green starts
%
%   Without this input the model cannot tell a stopped car at the head of
%   a queue at the start of green from the same car in the middle of red.

Cycle = 120;
if strcmpi(axisName, 'EW') || any(strcmpi(axisName, {'E', 'W'}))
    g0 = 60;  g1 = 118;      % green start, yellow end
else
    g0 = 0;   g1 = 58;
end
tc = mod(t, Cycle);
if tc >= g0 && tc < g1
    g = g1 - tc;
else
    g = -mod(g0 - tc, Cycle);
end
end
