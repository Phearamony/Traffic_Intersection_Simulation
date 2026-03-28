%% 4-Way Intersection Simulation (with RSU free-flow ETA priority on GREEN axis)
close all; clear; clc;

% addpath
addpath(genpath('C:/Users/monea/OneDrive/Documents/MATLAB/Traffic_Intersection/class'))
addpath(genpath('C:/Users/monea/OneDrive/Documents/MATLAB/Traffic_Intersection/function'))
addpath(genpath('C:/Users/monea/OneDrive/Documents/MATLAB/Traffic_Intersection/simplot'))

global CarN CarS CarE CarW TrafficLight
global dt KK

InitVals;
fm = 1;
dt = 0.5;
KKmax = 500;

% --- Traffic light parameters (time-based) ---
Cycle = 120;           % seconds
gNS = 54.5;  yNS = 3.5;  ar1 = 2.0;   % NS green, NS yellow, all-red after NS
gEW = 54.5;  yEW = 3.5;  ar2 = 2.0;   % EW green, EW yellow, all-red after EW

% --- Traffic light / cycle log ---
LightLog = struct('Time', {}, 'CycleIdx', {}, 'tInCycle', {}, ...
                  'Phase', {}, 'NS', {}, 'EW', {});

% --- wait parameters ---
stop_line = 10;  % meters before the center intersection

% --- Gap acceptance parameters for right turns ---
minGapTime = 2.0;   % [s]
minGapDist = 6.0;   % [m]

% --- turn parameters ---
turn_start = 10;
turn_wait  = 2;
turn_release = 1.5;
out_eps = 0.5;
wait_eps = 0.3;
appear = 3;

% --- Log ---
CarLog = struct('Time', {}, 'ID', {}, 'Dir', {}, 'TurnLeft', {},'TurnRight', {}, 'X', {}, 'Y', {}, 'V', {}, 'Ac', {});
CarLogN = CarLog;  CarLogS = CarLog;  CarLogE = CarLog;  CarLogW = CarLog;

% --- Cars parameters ---
CarN = Car.empty();
CarS = Car.empty();
CarE = Car.empty();
CarW = Car.empty();

CarIDN = 1000;
CarIDS = 2000;
CarIDE = 3000;
CarIDW = 4000;

hFig = figure;
set(hFig, 'Position', [100, 100, 1000, 800]);

% --- Traffic flow parameters ---
spawn_gap_min = 8;            % meters between entry and nearest car

% Piecewise ramp: 0–120s light, 120–300s medium, 300s+ heavy
DemandStages = [0,120,300];   % seconds

% Axis flow rates (veh/s) — split equally to the two approaches on that axis
lambdaNS_axis = [0.30, 0.45, 0.60];  % NS total
lambdaEW_axis = [0.25, 0.40, 0.55];  % EW total

%% === Initialize seed cars ===
% SOUTH → NORTH
carN = Car(CarIDN, -1.5, -150);
CarIDN = CarIDN + 1;
carN.Dir = 'N';
CarN(end+1) = carN;

% NORTH → SOUTH
carS = Car(CarIDS, 1.5, 150);
CarIDS = CarIDS + 1;
carS.Dir = 'S';
CarS(end+1) = carS;

% WEST → EAST
carE = Car(CarIDE, -150, 1.5);
CarIDE = CarIDE + 1;
carE.Dir = 'E';
CarE(end+1) = carE;

% EAST → WEST
carW = Car(CarIDW, 150, -1.5);
CarIDW = CarIDW + 1;
carW.Dir = 'W';
CarW(end+1) = carW;

%% === Main loop ===
for KK = 1:KKmax
    %% === Traffic Light Phases ===
    t = mod(KK*dt, Cycle);

    if t < gNS
        TrafficLight.NS = 'green';  TrafficLight.EW = 'red';
    elseif t < gNS + yNS
        TrafficLight.NS = 'yellow'; TrafficLight.EW = 'red';
    elseif t < gNS + yNS + ar1
        TrafficLight.NS = 'red';    TrafficLight.EW = 'red';   % all-red clearance
    elseif t < gNS + yNS + ar1 + gEW
        TrafficLight.NS = 'red';    TrafficLight.EW = 'green';
    elseif t < gNS + yNS + ar1 + gEW + yEW
        TrafficLight.NS = 'red';    TrafficLight.EW = 'yellow';
    else
        TrafficLight.NS = 'red';    TrafficLight.EW = 'red';   % all-red clearance
    end

    % --- Log traffic cycle state ---
    cycIdx = floor((KK*dt) / Cycle) + 1;       % 1-based cycle index
    phs = phaseName(t, gNS, yNS, ar1, gEW, yEW);
    LightLog(end+1) = struct( ...
        'Time',      KK*dt, ...
        'CycleIdx',  cycIdx, ...
        'tInCycle',  t, ...
        'Phase',     phs, ...
        'NS',        TrafficLight.NS, ...
        'EW',        TrafficLight.EW );

    %% === Plot (optional throttle) ===
    if(mod(KK,2)==1)
        SimPlotIntersection;
    end

    %% === Spawn new cars (Poisson, ramps from light->heavy) ===
    tsec = KK * dt;
    [lamN, lamS, lamE, lamW] = getApproachRates(tsec, DemandStages, lambdaNS_axis, lambdaEW_axis);
    spawnProb = @(lam) 1 - exp(-lam * dt);

    % NORTH
    if rand < spawnProb(lamN) && canSpawnY(CarN, -150, spawn_gap_min)
        carN = Car(CarIDN, -1.5, -150); CarIDN = CarIDN + 1;
        carN.Dir = 'N';  CarN(end+1) = carN;
    end
    % SOUTH
    if rand < spawnProb(lamS) && canSpawnY(CarS, 150, spawn_gap_min)
        carS = Car(CarIDS, 1.5, 150);  CarIDS = CarIDS + 1;
        carS.Dir = 'S';  CarS(end+1) = carS;
    end
    % EAST
    if rand < spawnProb(lamE) && canSpawnX(CarE, -150, spawn_gap_min)
        carE = Car(CarIDE, -150, 1.5); CarIDE = CarIDE + 1;
        carE.Dir = 'E';  CarE(end+1) = carE;
    end
    % WEST
    if rand < spawnProb(lamW) && canSpawnX(CarW, 150, spawn_gap_min)
        carW = Car(CarIDW, 150, -1.5); CarIDW = CarIDW + 1;
        carW.Dir = 'W';  CarW(end+1) = carW;
    end

    %% === RSU priority on the current GREEN axis (free-flow ETA) ===
    gate_eps = 0.3;  % window around stop-line to consider "at the line"
    [goID, goDir] = RSU_pick_priority_freeflow(CarN, CarS, CarE, CarW, TrafficLight);

    %% === Compute Acceleration and movement with Traffic Light Algorithm ===

    % NORTH → SOUTH
    used_dummyN = false;
    for i = 1:length(CarN)
        % RSU gating on GREEN axis (hold non-priority cars at stop line)
        if RSU_gate_on_green_axis(CarN(i),'N',goID,goDir,stop_line,gate_eps), continue; end

        if strcmp(TrafficLight.NS, 'red') || strcmp(TrafficLight.NS, 'yellow')
            if ~used_dummyN && (CarN(i).Y < -stop_line + CarN(i).R0)
                dummy = Car(-1, 0, 0);
                dummy.Y = -stop_line + CarN(i).R0;
                dummy.V = 0;
                dummy.Dir = 'N';
                CarN(i).Ac = CarN(i).IDM(CarN(i), dummy);
                used_dummyN = true;
            elseif i > 1
                CarN(i).Ac = CarN(i).IDM(CarN(i), CarN(i-1));
            else
                CarN(i).Ac = 0.5 * (CarN(i).Vd - CarN(i).V);
            end
        else
            if i > 1
                CarN(i).Ac = CarN(i).IDM(CarN(i), CarN(i-1));
            else
                CarN(i).Ac = 0.5 * (CarN(i).Vd - CarN(i).V);
            end
        end
    end

    % SOUTH → NORTH
    used_dummyS = false;
    for i = 1:length(CarS)
        if RSU_gate_on_green_axis(CarS(i),'S',goID,goDir,stop_line,gate_eps), continue; end

        if strcmp(TrafficLight.NS, 'red') || strcmp(TrafficLight.NS, 'yellow')
            if ~used_dummyS && (CarS(i).Y >  stop_line - CarS(i).R0)
                dummy = Car(-1, 0, 0);
                dummy.Y =  stop_line - CarS(i).R0;
                dummy.V = 0;
                dummy.Dir = 'S';
                CarS(i).Ac = CarS(i).IDM(CarS(i), dummy);
                used_dummyS = true;
            elseif i > 1
                CarS(i).Ac = CarS(i).IDM(CarS(i), CarS(i-1));
            else
                CarS(i).Ac = 0.5 * (CarS(i).Vd - CarS(i).V);
            end
        else
            if i > 1
                CarS(i).Ac = CarS(i).IDM(CarS(i), CarS(i-1));
            else
                CarS(i).Ac = 0.5 * (CarS(i).Vd - CarS(i).V);
            end
        end
    end

    % WEST → EAST
    used_dummyE = false;
    for i = 1:length(CarE)
        if RSU_gate_on_green_axis(CarE(i),'E',goID,goDir,stop_line,gate_eps), continue; end

        if strcmp(TrafficLight.EW, 'red') || strcmp(TrafficLight.EW, 'yellow')
            if ~used_dummyE && (CarE(i).X < -stop_line + CarE(i).R0)
                dummy = Car(-1, 0, 0);
                dummy.X = -stop_line + CarE(i).R0;
                dummy.V = 0;
                dummy.Dir = 'E';
                CarE(i).Ac = CarE(i).IDM(CarE(i), dummy);
                used_dummyE = true;
            elseif i > 1
                CarE(i).Ac = CarE(i).IDM(CarE(i), CarE(i-1));
            else
                CarE(i).Ac = 0.5 * (CarE(i).Vd - CarE(i).V);
            end
        else
            if i > 1
                CarE(i).Ac = CarE(i).IDM(CarE(i), CarE(i-1));
            else
                CarE(i).Ac = 0.5 * (CarE(i).Vd - CarE(i).V);
            end
        end
    end

    % EAST → WEST
    used_dummyW = false;
    for i = 1:length(CarW)
        if RSU_gate_on_green_axis(CarW(i),'W',goID,goDir,stop_line,gate_eps), continue; end

        if strcmp(TrafficLight.EW, 'red') || strcmp(TrafficLight.EW, 'yellow')
            if ~used_dummyW && (CarW(i).X >  stop_line - CarW(i).R0)
                dummy = Car(-1, 0, 0);
                dummy.X =  stop_line - CarW(i).R0;
                dummy.V = 0;
                dummy.Dir = 'W';
                CarW(i).Ac = CarW(i).IDM(CarW(i), dummy);
                used_dummyW = true;
            elseif i > 1
                CarW(i).Ac = CarW(i).IDM(CarW(i), CarW(i-1));
            else
                CarW(i).Ac = 0.5 * (CarW(i).Vd - CarW(i).V);
            end
        else
            if i > 1
                CarW(i).Ac = CarW(i).IDM(CarW(i), CarW(i-1));
            else
                CarW(i).Ac = 0.5 * (CarW(i).Vd - CarW(i).V);
            end
        end
    end

    %% === Turn Left (unchanged) ===
    % W -> S
    for i = length(CarW):-1:1
        if CarW(i).TurnLeft == 1 && CarW(i).X <= turn_release && CarW(i).TurnedLeft == 0
            turningCar = CarW(i);
            turningCar.Dir = 'S';
            turningCar.X = 1.5;
            turningCar.Y = -appear;
            turningCar.TurnedLeft = 1;
            insertIndex = find([CarS.Y] > turningCar.Y, 1);
            if isempty(insertIndex)
                CarS(end+1) = turningCar;
            else
                CarS(insertIndex+1:end+1) = CarS(insertIndex:end);
                CarS(insertIndex) = turningCar;
            end
            CarW(i) = [];
        end
    end

    % E -> N
    for i = length(CarE):-1:1
        if CarE(i).TurnLeft == 1 && CarE(i).X >= -turn_release && CarE(i).TurnedLeft == 0
            turningCar = CarE(i);
            turningCar.Dir = 'N';
            turningCar.X = -1.5;
            turningCar.Y = appear;
            turningCar.TurnedLeft = 1;
            insertIndex = find([CarN.Y] < turningCar.Y, 1);
            if isempty(insertIndex)
                CarN(end+1) = turningCar;
            else
                CarN(insertIndex+1:end+1) = CarN(insertIndex:end);
                CarN(insertIndex) = turningCar;
            end
            CarE(i) = [];
        end
    end

    % S -> E
    for i = length(CarS):-1:1
        if CarS(i).TurnLeft == 1 && CarS(i).Y <= turn_release && CarS(i).TurnedLeft == 0
            turningCar = CarS(i);
            turningCar.Dir = 'E';
            turningCar.X = appear;
            turningCar.Y = 1.5;
            turningCar.TurnedLeft = 1;
            insertIndex = find([CarE.X] < turningCar.X, 1);
            if isempty(insertIndex)
                CarE(end+1) = turningCar;
            else
                CarE(insertIndex+1:end+1) = CarE(insertIndex:end);
                CarE(insertIndex) = turningCar;
            end
            CarS(i) = [];
        end
    end

    % N -> W
    for i = length(CarN):-1:1
        if CarN(i).TurnLeft == 1 && CarN(i).Y >= -turn_release && CarN(i).TurnedLeft == 0
            turningCar = CarN(i);
            turningCar.Dir = 'W';
            turningCar.X = -appear;
            turningCar.Y = -1.5;
            turningCar.TurnedLeft = 1;
            insertIndex = find([CarW.X] > turningCar.X, 1);
            if isempty(insertIndex)
                CarW(end+1) = turningCar;
            else
                CarW(insertIndex+1:end+1) = CarW(insertIndex:end);
                CarW(insertIndex) = turningCar;
            end
            CarN(i) = [];
        end
    end

    %% === Turn Right (unchanged) ===
    % W -> N
    used_dummyW = false;
    for i = length(CarW):-1:1
        car = CarW(i);
        if car.TurnRight == 1 && car.TurnedRight == 0
            if car.X > turn_wait && car.X <= turn_start
                if ~used_dummyW
                    dummy = Car(-1, 0, 0);
                    dummy.X = -car.R0;
                    dummy.Y = car.Y;
                    dummy.V = 0;
                    car.Ac = car.IDM(car, dummy);
                    used_dummyW = true;
                end
            end
            if car.X <= turn_wait
                dummy = Car(-1, 0, 0);
                dummy.X = turn_wait - car.R0;
                dummy.Y = car.Y;
                dummy.V = 0;
                car.Ac = car.IDM(car, dummy);

                t_turn = estimateTurnTime(car, turn_release, turn_wait, 5, -1.5);

                conf = CarE(([CarE.X] < -turn_release) & (~[CarE.TurnRight]));
                nconf = numel(conf);

                if nconf == 0
                    safeToTurn = true;
                elseif nconf == 1
                    other = conf(1);
                    dist_to_conf = abs(-turn_release - other.X);
                    t_arrival = dist_to_conf / max(other.V, 0.1);
                    safeToTurn = (dist_to_conf >= minGapDist) || (t_arrival >= t_turn + minGapTime);
                else
                    safeToTurn = true;
                    for j = 1:nconf
                        other = conf(j);
                        dist_to_conf = abs(-turn_release - other.X);
                        t_arrival = dist_to_conf / max(other.V, 0.1);
                        if (t_arrival < t_turn + minGapTime) && (dist_to_conf < (minGapDist + 2))
                            safeToTurn = false; break;
                        end
                    end
                end

                allowOnSignal = ~strcmp(TrafficLight.EW,'green');
                bothRight = leadRightAtWait(CarW,'W',turn_wait,wait_eps) && ...
                            leadRightAtWait(CarE,'E',turn_wait,wait_eps);
                leftBlocks = ~bothRight && leftTurnQueuePresent(CarE) && ...
                             leftTurnHasPriority(CarE,'E',turn_start);
                if ~leftBlocks && (safeToTurn || allowOnSignal)
                    car.Dir = 'N';
                    car.X = -1.5;
                    car.Y = appear - out_eps;
                    car.V = 3;
                    car.Ac = 0.5 * (car.Vd - car.V);
                    car.TurnedRight = 1;
                    insertIndex = find([CarN.Y] < car.Y, 1);
                    if isempty(insertIndex)
                        CarN(end+1) = car;
                    else
                        CarN(insertIndex+1:end+1) = CarN(insertIndex:end);
                        CarN(insertIndex) = car;
                    end
                    CarW(i) = [];
                    continue;
                end
            end
            CarW(i) = car;
        end
    end

    % E -> S
    used_dummyE = false;
    for i = length(CarE):-1:1
        car = CarE(i);
        if car.TurnRight == 1 && car.TurnedRight == 0
            if car.X < -turn_wait && car.X >= -turn_start
                if ~used_dummyE
                    dummy = Car(-1, 0, 0);
                    dummy.X = car.R0;
                    dummy.Y = car.Y;
                    dummy.V = 0;
                    car.Ac = car.IDM(car, dummy);
                    used_dummyE = true;
                end
            end
            if car.X >= -turn_wait
                dummy = Car(-1, 0, 0);
                dummy.X = -turn_wait + car.R0;
                dummy.Y = car.Y;
                dummy.V = 0;
                car.Ac = car.IDM(car, dummy);

                t_turn = estimateTurnTime(car, -turn_release, -turn_wait, -5, 1.5);

                conf = CarW(([CarW.X] >  turn_release) & (~[CarW.TurnRight]));
                nconf = numel(conf);

                if nconf == 0
                    safeToTurn = true;
                elseif nconf == 1
                    other = conf(1);
                    dist_to_conf = abs(turn_release - other.X);
                    t_arrival = dist_to_conf / max(other.V, 0.1);
                    safeToTurn = (dist_to_conf >= minGapDist) || (t_arrival >= t_turn + minGapTime);
                else
                    safeToTurn = true;
                    for j = 1:nconf
                        other = conf(j);
                        dist_to_conf = abs(turn_release - other.X);
                        t_arrival = dist_to_conf / max(other.V, 0.1);
                        if (t_arrival < t_turn + minGapTime) && (dist_to_conf < (minGapDist + 2))
                            safeToTurn = false; break;
                        end
                    end
                end

                allowOnSignal = ~strcmp(TrafficLight.EW,'green');
                bothRight = leadRightAtWait(CarE,'E',turn_wait,wait_eps) && ...
                            leadRightAtWait(CarW,'W',turn_wait,wait_eps);
                leftBlocks = ~bothRight && leftTurnQueuePresent(CarW) && ...
                             leftTurnHasPriority(CarW,'W',turn_start);

                if ~leftBlocks && (safeToTurn || allowOnSignal)
                    car.Dir = 'S';
                    car.X = 1.5;
                    car.Y = -(appear - out_eps);
                    car.V = 3;
                    car.Ac = 0.5 * (car.Vd - car.V);
                    car.TurnedRight = 1;
                    insertIndex = find([CarS.Y] > car.Y, 1);
                    if isempty(insertIndex)
                        CarS(end+1) = car;
                    else
                        CarS(insertIndex+1:end+1) = CarS(insertIndex:end);
                        CarS(insertIndex) = car;
                    end
                    CarE(i) = [];
                    continue;
                end
            end
            CarE(i) = car;
        end
    end

    % S -> W
    used_dummyS = false;
    for i = length(CarS):-1:1
        car = CarS(i);
        if car.TurnRight == 1 && car.TurnedRight == 0
            if car.Y > turn_wait && car.Y <= turn_start
                if ~used_dummyS
                    dummy = Car(-1, 0, 0);
                    dummy.Y = -car.R0;
                    dummy.X = car.X;
                    dummy.V = 0;
                    car.Ac = car.IDM(car, dummy);
                    used_dummyS = true;
                end
            end
            if car.Y <= turn_wait
                dummy = Car(-1, 0, 0);
                dummy.Y = turn_wait - car.R0;
                dummy.X = car.X;
                dummy.V = 0;
                car.Ac = car.IDM(car, dummy);

                t_turn = estimateTurnTime(car, turn_release, turn_wait, 1.5, 5);

                conf = CarN(([CarN.Y] < -turn_release) & (~[CarN.TurnRight]));
                nconf = numel(conf);

                if nconf == 0
                    safeToTurn = true;
                elseif nconf == 1
                    other = conf(1);
                    dist_to_conf = abs(-turn_release - other.Y);
                    t_arrival = dist_to_conf / max(other.V, 0.1);
                    safeToTurn = (dist_to_conf >= minGapDist) || (t_arrival >= t_turn + minGapTime);
                else
                    safeToTurn = true;
                    for j = 1:nconf
                        other = conf(j);
                        dist_to_conf = abs(-turn_release - other.Y);
                        t_arrival = dist_to_conf / max(other.V, 0.1);
                        if (t_arrival < t_turn + minGapTime) && (dist_to_conf < (minGapDist + 2))
                            safeToTurn = false; break;
                        end
                    end
                end

                allowOnSignal = ~strcmp(TrafficLight.NS,'green');
                bothRight = leadRightAtWait(CarS,'S',turn_wait,wait_eps) && ...
                            leadRightAtWait(CarN,'N',turn_wait,wait_eps);
                leftBlocks = ~bothRight && leftTurnQueuePresent(CarN) && ...
                             leftTurnHasPriority(CarN,'N',turn_start);

                if ~leftBlocks && (safeToTurn || allowOnSignal)
                    car.Dir = 'W';
                    car.X = -(appear - out_eps);
                    car.Y = -1.5;
                    car.V = 3;
                    car.Ac = 0.5 * (car.Vd - car.V);
                    car.TurnedRight = 1;
                    insertIndex = find([CarW.X] > car.X, 1);
                    if isempty(insertIndex)
                        CarW(end+1) = car;
                    else
                        CarW(insertIndex+1:end+1) = CarW(insertIndex:end);
                        CarW(insertIndex) = car;
                    end
                    CarS(i) = [];
                    continue;
                end
            end
            CarS(i) = car;
        end
    end

    % N -> E
    used_dummyN = false;
    for i = length(CarN):-1:1
        car = CarN(i);
        if car.TurnRight == 1 && car.TurnedRight == 0
            if car.Y < -turn_wait && car.Y >= -turn_start
                if ~used_dummyN
                    dummy = Car(-1, 0, 0);
                    dummy.Y = car.R0;
                    dummy.X = car.X;
                    dummy.V = 0;
                    car.Ac = car.IDM(car, dummy);
                    used_dummyN = true;
                end
            end
            if car.Y >= -turn_wait
                dummy = Car(-1, 0, 0);
                dummy.Y = -turn_wait + car.R0;
                dummy.X = car.X;
                dummy.V = 0;
                car.Ac = car.IDM(car, dummy);

                t_turn = estimateTurnTime(car, -turn_release, -turn_wait, -1.5, -5);

                conf = CarS(([CarS.Y] >  turn_release) & (~[CarS.TurnRight]));
                nconf = numel(conf);

                if nconf == 0
                    safeToTurn = true;
                elseif nconf == 1
                    other = conf(1);
                    dist_to_conf = abs(turn_release - other.Y);
                    t_arrival = dist_to_conf / max(other.V, 0.1);
                    safeToTurn = (dist_to_conf >= minGapDist) || (t_arrival >= t_turn + minGapTime);
                else
                    safeToTurn = true;
                    for j = 1:nconf
                        other = conf(j);
                        dist_to_conf = abs(turn_release - other.Y);
                        t_arrival = dist_to_conf / max(other.V, 0.1);
                        if (t_arrival < t_turn + minGapTime) && (dist_to_conf < (minGapDist + 2))
                            safeToTurn = false; break;
                        end
                    end
                end

                allowOnSignal = ~strcmp(TrafficLight.NS,'green');
                bothRight = leadRightAtWait(CarN,'N',turn_wait,wait_eps) && ...
                            leadRightAtWait(CarS,'S',turn_wait,wait_eps);
                leftBlocks = ~bothRight && leftTurnQueuePresent(CarS) && ...
                             leftTurnHasPriority(CarS,'S',turn_start);

                if ~leftBlocks && (safeToTurn || allowOnSignal)
                    car.Dir = 'E';
                    car.X = appear - out_eps;
                    car.Y = 1.5;
                    car.V = 3;
                    car.Ac = 0.5 * (car.Vd - car.V);
                    car.TurnedRight = 1;
                    insertIndex = find([CarE.X] < car.X, 1);
                    if isempty(insertIndex)
                        CarE(end+1) = car;
                    else
                        CarE(insertIndex+1:end+1) = CarE(insertIndex:end);
                        CarE(insertIndex) = car;
                    end
                    CarN(i) = [];
                    continue;
                end
            end
            CarN(i) = car;
        end
    end

    %% === Update movement ===
    for i = 1:length(CarN), CarN(i).fdp(CarN(i)); CarN(i).V = CarN(i).fdv(CarN(i)); end
    for i = 1:length(CarS), CarS(i).fdp(CarS(i)); CarS(i).V = CarS(i).fdv(CarS(i)); end
    for i = 1:length(CarE), CarE(i).fdp(CarE(i)); CarE(i).V = CarE(i).fdv(CarE(i)); end
    for i = 1:length(CarW), CarW(i).fdp(CarW(i)); CarW(i).V = CarW(i).fdv(CarW(i)); end

    %% === Log data after movement (ALL CARS) ===
    for i = 1:length(CarN)
        CarLog(end+1) = struct('Time',KK*dt,'ID',CarN(i).ID,'Dir',CarN(i).Dir, ...
            'TurnLeft',CarN(i).TurnLeft,'TurnRight',CarN(i).TurnRight, ...
            'X',CarN(i).X,'Y',CarN(i).Y,'V',CarN(i).V,'Ac',CarN(i).Ac);
    end
    for i = 1:length(CarS)
        CarLog(end+1) = struct('Time',KK*dt,'ID',CarS(i).ID,'Dir',CarS(i).Dir, ...
            'TurnLeft',CarS(i).TurnLeft,'TurnRight',CarS(i).TurnRight, ...
            'X',CarS(i).X,'Y',CarS(i).Y,'V',CarS(i).V,'Ac',CarS(i).Ac);
    end
    for i = 1:length(CarE)
        CarLog(end+1) = struct('Time',KK*dt,'ID',CarE(i).ID,'Dir',CarE(i).Dir, ...
            'TurnLeft',CarE(i).TurnLeft,'TurnRight',CarE(i).TurnRight, ...
            'X',CarE(i).X,'Y',CarE(i).Y,'V',CarE(i).V,'Ac',CarE(i).Ac);
    end
    for i = 1:length(CarW)
        CarLog(end+1) = struct('Time',KK*dt,'ID',CarW(i).ID,'Dir',CarW(i).Dir, ...
            'TurnLeft',CarW(i).TurnLeft,'TurnRight',CarW(i).TurnRight, ...
            'X',CarW(i).X,'Y',CarW(i).Y,'V',CarW(i).V,'Ac',CarW(i).Ac);
    end

    %% === SAME STRUCT, PER-LANE LOGS ===
    for i = 1:length(CarN)
        CarLogN(end+1) = struct('Time',KK*dt,'ID',CarN(i).ID,'Dir',CarN(i).Dir, ...
            'TurnLeft',CarN(i).TurnLeft,'TurnRight',CarN(i).TurnRight, ...
            'X',CarN(i).X,'Y',CarN(i).Y,'V',CarN(i).V,'Ac',CarN(i).Ac);
    end
    for i = 1:length(CarS)
        CarLogS(end+1) = struct('Time',KK*dt,'ID',CarS(i).ID,'Dir',CarS(i).Dir, ...
            'TurnLeft',CarS(i).TurnLeft,'TurnRight',CarS(i).TurnRight, ...
            'X',CarS(i).X,'Y',CarS(i).Y,'V',CarS(i).V,'Ac',CarS(i).Ac);
    end
    for i = 1:length(CarE)
        CarLogE(end+1) = struct('Time',KK*dt,'ID',CarE(i).ID,'Dir',CarE(i).Dir, ...
            'TurnLeft',CarE(i).TurnLeft,'TurnRight',CarE(i).TurnRight, ...
            'X',CarE(i).X,'Y',CarE(i).Y,'V',CarE(i).V,'Ac',CarE(i).Ac);
    end
    for i = 1:length(CarW)
        CarLogW(end+1) = struct('Time',KK*dt,'ID',CarW(i).ID,'Dir',CarW(i).Dir, ...
            'TurnLeft',CarW(i).TurnLeft,'TurnRight',CarW(i).TurnRight, ...
            'X',CarW(i).X,'Y',CarW(i).Y,'V',CarW(i).V,'Ac',CarW(i).Ac);
    end

    %% === Remove cars out of bounds ===
    CarN = CarN(abs([CarN.X]) <= 170 & abs([CarN.Y]) <= 170);
    CarS = CarS(abs([CarS.X]) <= 170 & abs([CarS.Y]) <= 170);
    CarE = CarE(abs([CarE.X]) <= 170 & abs([CarE.Y]) <= 170);
    CarW = CarW(abs([CarW.X]) <= 170 & abs([CarW.Y]) <= 170);
end


%% === Plot ===
plot_EW_trajectories(LightLog, CarLogE, CarLogW, stop_line, 'C:/Users/monea/OneDrive/Documents/MATLAB/Traffic_Intersection/output/EW_mimic.png');
plot_NS_trajectories(LightLog, CarLogS, CarLogN, stop_line, 'C:/Users/monea/OneDrive/Documents/MATLAB/Traffic_Intersection/output/NS_mimic.png');


%% === Helper Functions ===
function t_turn = estimateTurnTime(car, turn_release, turn_wait, turn_waitX, turn_waitY)
d_entry = sqrt((car.X - turn_waitX)^2 + (car.Y - turn_waitY)^2);
flat = (turn_wait - turn_release) / 2;
arc = sqrt(2 * flat^2);
total_dist = d_entry + 2 * flat + arc;
v_turn = min((car.V + car.Vd) / 2, 3);
t_turn = total_dist / max(v_turn, 0.1);
end

function [lamN, lamS, lamE, lamW] = getApproachRates(tsec, stages, lamNS_axis, lamEW_axis)
if tsec < stages(2)
    k = 1;
elseif tsec < stages(3)
    k = 2;
else
    k = 3;
end
lamNS = lamNS_axis(k);  lamEW = lamEW_axis(k);
lamN = 0.5 * lamNS;  lamS = 0.5 * lamNS;
lamE = 0.5 * lamEW;  lamW = 0.5 * lamEW;
end

function ok = canSpawnY(list, y0, gap)
if isempty(list), ok = true; return; end
ok = all(abs([list.Y] - y0) >= gap);
end

function ok = canSpawnX(list, x0, gap)
if isempty(list), ok = true; return; end
ok = all(abs([list.X] - x0) >= gap);
end

function hasLeftQueue = leftTurnQueuePresent(list)
if isempty(list), hasLeftQueue = false; return; end
hasLeftQueue = any(([list.TurnLeft] == 1) & (~[list.TurnedLeft]));
end

function waitLeft = leftTurnHasPriority(opList, oppDir, prioWin)
waitLeft = false; if isempty(opList), return; end
switch oppDir
    case 'E', cand = ([opList.TurnLeft]==1)&(~[opList.TurnedLeft])&([opList.X] >= -prioWin);
    case 'W', cand = ([opList.TurnLeft]==1)&(~[opList.TurnedLeft])&([opList.X] <=  prioWin);
    case 'N', cand = ([opList.TurnLeft]==1)&(~[opList.TurnedLeft])&([opList.Y] >= -prioWin);
    case 'S', cand = ([opList.TurnLeft]==1)&(~[opList.TurnedLeft])&([opList.Y] <=  prioWin);
    otherwise, cand = false(size(opList));
end
waitLeft = any(cand);
end

function yes = leadRightAtWait(list, dir, turn_wait, eps)
yes = false; if isempty(list), return; end
switch dir
    case 'E'
        [~,k] = max([list.X]); c = list(k); yes = c.TurnRight && ~c.TurnedRight && abs(c.X + turn_wait) <= eps;
    case 'W'
        [~,k] = min([list.X]); c = list(k); yes = c.TurnRight && ~c.TurnedRight && abs(c.X - turn_wait) <= eps;
    case 'N'
        [~,k] = max([list.Y]); c = list(k); yes = c.TurnRight && ~c.TurnedRight && abs(c.Y + turn_wait) <= eps;
    case 'S'
        [~,k] = min([list.Y]); c = list(k); yes = c.TurnRight && ~c.TurnedRight && abs(c.Y - turn_wait) <= eps;
    otherwise
        yes = false;
end
end

function phase = phaseName(t, gNS, yNS, ar1, gEW, yEW)
if t < gNS
    phase = 'NS_GREEN';
elseif t < gNS + yNS
    phase = 'NS_YELLOW';
elseif t < gNS + yNS + ar1
    phase = 'ALL_RED_1';
elseif t < gNS + yNS + ar1 + gEW
    phase = 'EW_GREEN';
elseif t < gNS + yNS + ar1 + gEW + yEW
    phase = 'EW_YELLOW';
else
    phase = 'ALL_RED_2';
end
end

%% === RSU FREE-FLOW PRIORITY HELPERS ===
function [goID, goDir] = RSU_pick_priority_freeflow(CarN, CarS, CarE, CarW, TrafficLight)
% Choose one priority car only on the current GREEN axis, using free-flow ETA:
% ETA = distance to (0,0) / current speed. Ties -> random.

goID = -1; goDir = '';
tie_eps = 1e-6;   % ETA tie tolerance

if strcmp(TrafficLight.NS,'green')
    cand = struct('ID',{},'Dir',{},'ETA',{});
    if ~isempty(CarN)
        [~,k] = max([CarN.Y]); c = CarN(k);
        cand(end+1) = struct('ID',c.ID,'Dir','N','ETA', eta_NS(c)); %#ok<AGROW>
    end
    if ~isempty(CarS)
        [~,k] = min([CarS.Y]); c = CarS(k);
        cand(end+1) = struct('ID',c.ID,'Dir','S','ETA', eta_NS(c)); %#ok<AGROW>
    end
    [goID, goDir] = decide_one(cand, tie_eps);

elseif strcmp(TrafficLight.EW,'green')
    cand = struct('ID',{},'Dir',{},'ETA',{});
    if ~isempty(CarE)
        [~,k] = max([CarE.X]); c = CarE(k);
        cand(end+1) = struct('ID',c.ID,'Dir','E','ETA', eta_EW(c)); %#ok<AGROW>
    end
    if ~isempty(CarW)
        [~,k] = min([CarW.X]); c = CarW(k);
        cand(end+1) = struct('ID',c.ID,'Dir','W','ETA', eta_EW(c)); %#ok<AGROW>
    end
    [goID, goDir] = decide_one(cand, tie_eps);
end
end

function E = eta_NS(c)
d = abs(c.Y);
v = max(c.V, 0);     % true current speed
if v <= 0
    E = inf;
else
    E = d / v;
end
end

function E = eta_EW(c)
d = abs(c.X);
v = max(c.V, 0);
if v <= 0
    E = inf;
else
    E = d / v;
end
end

function [goID, goDir] = decide_one(cand, tie_eps)
% Robust chooser:
% - Prefer minimum FINITE ETA (distance/speed).
% - If multiple within tie_eps, pick random among those.
% - If NO finite ETA exists (all Inf/NaN), pick random among ALL candidates.
% - If tie set somehow empty, fall back to first candidate.

    goID = -1; goDir = '';
    if isempty(cand)
        return;
    end

    ETAs = [cand.ETA];

    % Prefer finite ETAs
    valid = isfinite(ETAs);
    if any(valid)
        minETA = min(ETAs(valid));
        tied = find(valid & abs(ETAs - minETA) <= tie_eps);
    else
        % No finite ETA (all Inf/NaN): pick random among all present
        tied = 1:numel(ETAs);
    end

    % Safety net: if still empty (e.g., all NaN and tie filter removed all), pick first non-NaN, else first
    if isempty(tied)
        nn = find(~isnan(ETAs), 1, 'first');
        if isempty(nn), nn = 1; end
        tied = nn;
    end

    % Now random among tied
    if numel(tied) == 1
        k = tied;
    else
        k = tied(randi(numel(tied)));
    end

    goID  = cand(k).ID;
    goDir = cand(k).Dir;
end


function held = RSU_gate_on_green_axis(car, dir, goID, goDir, stop_line, eps)
% On the current GREEN axis, ONLY the chosen 'goID' may proceed at the stop line.
% Others on the same axis WAIT at stop line. Red axis is untouched.

    held = false;

    % No decision this tick?
    if goID < 0 || isempty(goDir)
        return;
    end

    % Determine axes safely (scalar logicals)
    isNS = ismember(goDir, ['N','S']);
    isEW = ismember(goDir, ['E','W']);
    onNS = ismember(dir,   ['N','S']);
    onEW = ismember(dir,   ['E','W']);

    sameAxis = (isNS && onNS) || (isEW && onEW);
    if ~sameAxis
        return;
    end

    % Priority car proceeds with normal logic
    if car.ID == goID
        return;
    end

    % Non-priority car at stop line? -> hold via zero-speed dummy
    dummy = Car(-1,0,0); dummy.V = 0; dummy.Dir = dir;
    switch dir
        case 'N'
            atLine = abs(car.Y - (-stop_line + car.R0)) <= eps;
            if atLine
                dummy.Y = -stop_line + car.R0;
                car.Ac = car.IDM(car, dummy);
                held = true;
            end
        case 'S'
            atLine = abs(car.Y - ( stop_line - car.R0)) <= eps;
            if atLine
                dummy.Y =  stop_line - car.R0;
                car.Ac = car.IDM(car, dummy);
                held = true;
            end
        case 'E'
            atLine = abs(car.X - (-stop_line + car.R0)) <= eps;
            if atLine
                dummy.X = -stop_line + car.R0;
                car.Ac = car.IDM(car, dummy);
                held = true;
            end
        case 'W'
            atLine = abs(car.X - ( stop_line - car.R0)) <= eps;
            if atLine
                dummy.X =  stop_line - car.R0;
                car.Ac = car.IDM(car, dummy);
                held = true;
            end
    end
end

