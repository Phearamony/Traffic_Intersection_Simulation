function C = geom_constants()
%GEOM_CONSTANTS  Fixed numbers of the intersection and the vehicles.
C.DS       = 0.25;   % [m]   path sampling
C.L_APP    = 150;    % [m]   entry distance before the center
C.L_EXIT   = 170;    % [m]   exit distance after the center
C.STOP     = 10;     % [m]   stop line distance before the center
C.LANE     = 1.5;    % [m]   lane center offset (lane width 3 m)
C.R_RIGHT  = 6.0;    % [m]   right-turn radius (passes inside the center)
C.R_LEFT   = 4.5;    % [m]   left-turn radius
C.A_LAT    = 3.0;    % [m/s2] lateral acceleration limit on arcs
C.V_MAX    = 30;     % [m/s]
C.D_CONF   = 2.6;    % [m]   center distance below which two paths conflict
C.MERGE_LEN = 6;     % [m]   length of a merge zone
C.WAIT_GAP = 0.5;    % [m]   right-turn waiting point before its conflict zone
C.VEH_L    = 4.0;    % [m]   vehicle length
C.VEH_W    = 1.8;    % [m]   vehicle width
C.SPAWN_GAP = 8;     % [m]   minimum free length at the entry to spawn
C.DT       = 0.5;    % [s]   simulation step
% signal plan (same as every earlier script): cycle 120 s
C.CYCLE = 120;
C.G_NS = [0, 54.5];  C.Y_NS = [54.5, 58];     % NS green, NS yellow
C.G_EW = [60, 114.5]; C.Y_EW = [114.5, 118];  % EW green, EW yellow
end
