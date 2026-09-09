%% DEMO: GPR + MPC + RSU — sparse traffic, synchronized trajectory view
% Purpose: teaching/demo version of main_v2v_mpc_rsu_gpr.m, meant to sit
% alongside the other main_*.m scripts in this folder.
% Same Car/RSU classes, same MPC weights, same RSU coordination logic as
% the real thesis pipeline — only the traffic DENSITY is lowered and a
% live side-by-side animation (intersection + trajectory plot moving
% together) is added so the GPR+MPC coordination is easy to see.
%
% Only East/West lanes are used (mirrors the real GPR+MPC RSU config).
%
% IMPORTANT: 'clear classes' forces MATLAB to reload handle classes (Car, RSU)
% from disk. Without this, MATLAB uses the in-memory compiled version and will
% NOT pick up any edits made to Car.m or RSU.m since the last run.
clear classes %#ok<CLCLS>
close all; clear; clc;

% addpath — same layout as the other main_*.m scripts: class/, function/,
% simplot/ are sibling folders one level up from main/.
REPO_ROOT = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(REPO_ROOT, 'class')))
addpath(genpath(fullfile(REPO_ROOT, 'function')))
addpath(genpath(fullfile(REPO_ROOT, 'simplot')))

global CarN CarS CarE CarW TrafficLight
global dt KK

InitVals;
fm = 1;
dt = 0.5;
KKmax = 320;               % ~160 s of sim time — a bit more than one full 120s
                           % traffic-light cycle, so a red->green transition
                           % with real RSU coordination is visible

rng(7);                     % fixed seed so demo traffic arrivals are reproducible

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
minGapTime = 2.0;   % [s] minimum time gap vs your estimated turn time
minGapDist = 6.0;   % [m] minimum distance gap to the conflict point

% --- turn parameters ---
turn_start = 10;     % x position to begin slowing
turn_wait = 5; % x position to wait
turn_release = 1.5; % x position to trigger turning
out_eps = 0.5;   % meter past the stop line to drop a turning vehicle
wait_eps = 0.3;   % meter window to detect “at the wait point”
appear = 3;

T_reset_short = 10; % Reset short/temp storage every 10 seconds

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

% --- RSU ---
global RSUObjs
RSU1 = RSU(1, 0, 0);
RSUObjs = [RSU1];
% --- Load GPR arrival-time model ---
% Trained by main_human_driving_gpr_train.m (reused as-is from the repo)
gpr_model_path = fullfile(REPO_ROOT, 'simplot', 'gpr_arrival_model.mat');
if ~exist(gpr_model_path, 'file')
    error('GPR model not found. Copy gpr_arrival_model.mat into simplot/.');
end
RSU1.LoadGPRModel(gpr_model_path);


% Track previous traffic light state for green phase detection
prev_TrafficLight = struct('NS', 'red', 'EW', 'red');
opt_run_EW = false;
opt_run_NS = false;

hFig = figure;
set(hFig, 'Position', [80, 60, 1500, 700], 'Color', 'w');
axIntersection = subplot(1, 2, 1);
axTrajectory   = subplot(1, 2, 2);

% --- Video output (dual-panel: intersection view + live trajectory) ---
out_dir = fullfile(REPO_ROOT, 'output');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end
vid = VideoWriter(fullfile(out_dir, 'demo_gpr_mpc_sparse.mp4'), 'MPEG-4');
vid.FrameRate = 8;    % 320 frames / 8 fps = 40 s playback -> 160 s of sim time
                      % compressed 4x, smooth motion (every physics step is a frame)
open(vid);

%% === Build the STATIC trajectory panel once (same look as plot_EW_trajectories.m) ===
% Axes, labels, x/y-limits, "From East"/"From West" text, and legend are
% drawn exactly once, before the loop, and never touched again. Only the
% traffic-light strip (extended patch) and each car's line (XData/YData
% updated in place) change per frame — nothing else in this panel is
% redrawn, so it always looks like the final paper figure.
demo_tMax  = KKmax * dt;
demo_yMax  = 120;    % fixed y-limit — matches the intersection panel's spatial
                     % extent and the real study's typical queue lengths at
                     % this density
traj_line_width  = 1.4;
traj_font_size   = 12;

hold(axTrajectory, 'on'); grid(axTrajectory, 'on'); box(axTrajectory, 'on');
set(axTrajectory, 'Layer', 'top');
xlabel(axTrajectory, 'Time [s]', 'FontSize', traj_font_size + 2, 'FontWeight', 'bold');
ylabel(axTrajectory, 'Distance From Intersection [m]', 'FontSize', traj_font_size + 2, 'FontWeight', 'bold');
title(axTrajectory, 'GPR + MPC Trajectories (East/West)');
xlim(axTrajectory, [0 demo_tMax]);
ylim(axTrajectory, [-demo_yMax demo_yMax]);

% Center reference line at y = 0 (static)
plot(axTrajectory, [0 demo_tMax], [0 0], 'k-', 'LineWidth', 0.8);

% "From East" / "From West" labels (static, same placement as the paper figure)
text(axTrajectory, 0.02, 1.00, 'From East', 'Units','normalized', ...
    'FontSize', traj_font_size, 'VerticalAlignment','top', 'HorizontalAlignment','left', ...
    'Clipping','off', 'BackgroundColor','white');
text(axTrajectory, 0.02, 0.00, 'From West', 'Units','normalized', ...
    'FontSize', traj_font_size, 'VerticalAlignment','bottom', 'HorizontalAlignment','left', ...
    'Clipping','off', 'BackgroundColor','white');

% Legend (static) — AutoUpdate off so every car line/light patch added
% AFTER this point does NOT get appended to the legend as "dataN".
hTrajS = plot(axTrajectory, nan, nan, 'k', 'LineWidth', traj_line_width);
hTrajL = plot(axTrajectory, nan, nan, 'Color', [0.15 0.35 0.85], 'LineWidth', traj_line_width);
hTrajR = plot(axTrajectory, nan, nan, 'Color', [0.85 0.15 0.15], 'LineWidth', traj_line_width);
legTraj = legend(axTrajectory, [hTrajS hTrajL hTrajR], {'Straight','Left turn','Right turn'}, ...
    'Location','northeast', 'FontSize', traj_font_size, 'Box','on');
set(legTraj, 'AutoUpdate', 'off');
set(axTrajectory, 'FontSize', traj_font_size, 'LineWidth', 1.2);

% Light-strip patch (one per direction, EW only here) — grown each frame,
% never recreated. Starts as a zero-width patch at t=0.
light_width_demo = 4; dyL = light_width_demo/2;
hLightStrip = patch(axTrajectory, [0 0 0 0], [-dyL -dyL dyL dyL], [0.8 0 0], ...
    'EdgeColor','none', 'FaceAlpha', 0.9);
uistack(hLightStrip, 'bottom');   % keep it behind the trajectory lines

% Per-car line handles, keyed by car ID — created on first sight, then
% only XData/YData are updated (this is the "only the line moves" part).
trajLineMapE = containers.Map('KeyType','double','ValueType','any');
trajLineMapW = containers.Map('KeyType','double','ValueType','any');
lightSegStartT = 0;             % start time of the currently-open light segment
lightSegPhase  = '';            % phase of the currently-open light segment

% --- Traffic flow parameters ---
spawn_gap_min = 8;            % meters between entry and nearest car

%% === Generate a traffic schedule at the SAME density as the main study ===
% Same Poisson-arrival mechanism, same lambda, and same turn-probability
% split as main_human_driving.m's light-demand stage — nothing about the
% traffic itself is toned down. What's "cut down" for the demo is only
% the run LENGTH (KKmax), so the total number of cars stays easy to
% follow on screen without lowering how densely they arrive.
% E/W only, matching the real GPR+MPC RSU configuration.

lambdaEW_axis_demo = 0.25;   % veh/s TOTAL on the EW axis — same as the main
                             % study's light-demand stage (lambdaEW_axis(1))

dirs_list = {'N','S','E','W'};
sched = struct('KKmax', KKmax, 'dt', dt);
for di = 1:4
    d = dirs_list{di};
    sched.([d '_intent'])    = false(1, KKmax);
    sched.([d '_TurnLeft'])  = false(1, KKmax);
    sched.([d '_TurnRight']) = false(1, KKmax);
    sched.([d '_Vd'])        = zeros(1, KKmax);
    sched.([d '_Th'])        = zeros(1, KKmax);
end

lamE = 0.5 * lambdaEW_axis_demo;  % split equally between the two EW approaches
lamW = 0.5 * lambdaEW_axis_demo;
lams = struct('N', 0, 'S', 0, 'E', lamE, 'W', lamW);  % N/S stay silent

for kk = 1:KKmax
    for di = 1:4
        d = dirs_list{di};
        p = 1 - exp(-lams.(d) * dt);              % Poisson step probability
        sched.([d '_intent'])(kk)    = rand < p;
        tl = rand < 0.25;                          % 25% left-turn — same as main_human_driving.m
        tr = false;
        if ~tl, tr = rand < 0.15; end               % 15% right-turn — same as main_human_driving.m
        sched.([d '_TurnLeft'])(kk)  = tl;
        sched.([d '_TurnRight'])(kk) = tr;
        sched.([d '_Vd'])(kk)        = 21 + 6*rand + 2;   % same distribution as real sim
        sched.([d '_Th'])(kk)        = 1.2 + 1.2*rand;
    end
end

fprintf('Demo schedule generated (%d timesteps, dt=%.1fs, lambdaEW=%.2f veh/s — same density as main study)\n', ...
    KKmax, dt, lambdaEW_axis_demo);
fprintf('  Total arrival intentions: E=%d  W=%d\n', sum(sched.E_intent), sum(sched.W_intent));

% Activate Each Lane — East/West only, matching the real GPR+MPC RSU config
CARN = false;
CARS = false;
CARE = true;
CARW = true;

%% === Initialize new cars===
% for I=1:30
%     % SOUTH → NORTH (drive left, so +X)
%     IDN = CarIDN + I;
%     carN = Car(IDN, -1.5, -150);
%     carN.Dir = 'N';
%     CarN(end+1) = carN;
%
%     % NORTH → SOUTH (drive left, so -X)
%     IDS = CarIDS + I;
%     carS = Car(IDS, 1.5, 150);
%     carS.Dir = 'S';
%     CarS(end+1) = carS;
%
%     % WEST → EAST (drive left, so +Y)
%     IDE = CarIDE + I;
%     carE = Car(IDE, -150, -1.5);
%     carE.Dir = 'E';
%     CarE(end+1) = carE;
%
%     % EAST → WEST (drive left, so -Y)
%     IDW = CarIDW + I;
%     carW = Car(IDW, 150, 1.5);
%     carW.Dir = 'W';
%     CarW(end+1) = carW;
% end
%
% CarIDN = IDN;
% CarIDS = IDS;
% CarIDE = IDE;
% CarIDW = IDW;

% % SOUTH → NORTH (drive left, so +X)
if CARN
    carN = Car(CarIDN, -1.5, -150);
    CarIDN = CarIDN + 1;
    carN.Dir = 'N';
    CarN(end+1) = carN;
end

% % NORTH → SOUTH (drive left, so -X)
if CARS
    carS = Car(CarIDS, 1.5, 150);
    CarIDS = CarIDS + 1;
    carS.Dir = 'S';
    CarS(end+1) = carS;
end

% WEST → EAST (drive left, so +Y)
if CARE
    carE = Car(CarIDE, -150, 1.5);
    CarIDE = CarIDE + 1;
    carE.Dir = 'E';
    CarE(end+1) = carE;
end

% % EAST → WEST (drive left, so -Y)
if CARW
    carW = Car(CarIDW, 150, -1.5);
    CarIDW = CarIDW + 1;
    carW.Dir = 'W';
    CarW(end+1) = carW;
end


% --- Fuel & idle log (one entry per completed vehicle trip) ---
FuelLog = struct('ID',{},'Dir',{},'TurnRight',{},'TurnedRight',{}, ...
    'fuel_total',{},'idle_gap_time',{},'idle_red_time',{});

for KK = 1:KKmax
    %% === Traffic Light Phases ===
    t = mod(KK*dt, Cycle);
    tsec = KK * dt;  % Current time in seconds

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

    % Detect GREEN PHASE START and run optimization
    EW_green_start = strcmp(TrafficLight.EW, 'green') && ~strcmp(prev_TrafficLight.EW, 'green');
    NS_green_start = strcmp(TrafficLight.NS, 'green') && ~strcmp(prev_TrafficLight.NS, 'green');

    % Run optimization at GREEN START (always) and also every timestep
    % during green to catch new right-turners that arrive mid-phase.
    % RunGreenPhaseOptimization now returns true only when it actually
    % re-ran (turner changed), so we only print arrival times then.
    if EW_green_start
        fprintf('\n========================================\n');
        fprintf('[t=%.1f] EW GREEN START - Running RSU optimization...\n', tsec);
        fprintf('========================================\n');
        RSU1.opt_results.EW = [];     % Force fresh optimization at phase start
        RSU1.opt_results.EW_rev = [];  % Clear E-turner result too
        RSU1.last_opt_veh_ids.EW = []; % Reset 50m-radius trigger for new phase
        RSU1.RunGreenPhaseOptimization(tsec, 'EW', TrafficLight);
        RSU1.PrintArrivalTimes(TrafficLight);
        opt_run_EW = true;
    elseif strcmp(TrafficLight.EW, 'green')
        % Mid-phase: re-run only if a new turner arrived (cheap check)
        if RSU1.RunGreenPhaseOptimization(tsec, 'EW', TrafficLight)
            fprintf('[t=%.1f] EW re-optimized (vehicle set changed)\n', tsec);
            RSU1.PrintArrivalTimes(TrafficLight);
        end
    end

    if NS_green_start
        fprintf('\n========================================\n');
        fprintf('[t=%.1f] NS GREEN START - Running RSU optimization...\n', tsec);
        fprintf('========================================\n');
        RSU1.opt_results.NS = [];     % Force fresh optimization at phase start
        RSU1.opt_results.NS_rev = [];  % Clear S-turner result too
        RSU1.last_opt_veh_ids.NS = []; % Reset 50m-radius trigger for new phase
        RSU1.RunGreenPhaseOptimization(tsec, 'NS', TrafficLight);
        RSU1.PrintArrivalTimes(TrafficLight);
        opt_run_NS = true;
    elseif strcmp(TrafficLight.NS, 'green')
        if RSU1.RunGreenPhaseOptimization(tsec, 'NS', TrafficLight)
            fprintf('[t=%.1f] NS re-optimized (vehicle set changed)\n', tsec);
            RSU1.PrintArrivalTimes(TrafficLight);
        end
    end

    % Reset optimization flag when phase ends
    if ~strcmp(TrafficLight.EW, 'green')
        opt_run_EW = false;
    end
    if ~strcmp(TrafficLight.NS, 'green')
        opt_run_NS = false;
    end

    % Update previous state for next iteration
    prev_TrafficLight = TrafficLight;

    % Reset V2X storage short
    if mod(tsec, T_reset_short) < dt
        % Reset RSU short storage
        RSU1.ResetAllShort();

        % Reset all cars' short storage
        for i = 1:length(CarN), CarN(i).ResetAllShort(); end
        for i = 1:length(CarS), CarS(i).ResetAllShort(); end
        for i = 1:length(CarE), CarE(i).ResetAllShort(); end
        for i = 1:length(CarW), CarW(i).ResetAllShort(); end

        % Optional: Print debug message
        % fprintf('t=%.1f: Reset V2X short storage\n', tsec);
    end

    %% === Plot (dual panel: intersection + live trajectory) — every step ===
    % Plotting every physics step (not every other) keeps the 8 fps output
    % smooth rather than choppy.
    [trajLineMapE, trajLineMapW, hLightStrip, lightSegStartT, lightSegPhase] = ...
        updateTrajectoryPanel(axTrajectory, tsec, TrafficLight, stop_line, CarLogE, CarLogW, ...
                               trajLineMapE, trajLineMapW, hLightStrip, lightSegStartT, lightSegPhase);
    drawDemoFrame(axIntersection, tsec, vid);

    %% === Spawn new cars (from shared traffic schedule) ===
    % Arrival intentions (sched.*_intent) are identical across all scenarios.
    % CARN/CARS/CARE/CARW flags still control which directions are active.
    % canSpawnX/Y enforces physical spacing — scenario-specific and correct.

    % % NORTH (spawn at Y=-150, moving +Y)
    if CARN && sched.N_intent(KK) && canSpawnY(CarN, -150, spawn_gap_min)
        carN = Car(CarIDN, -1.5, -150); CarIDN = CarIDN + 1;
        carN.Dir = 'N';
        carN.TurnLeft = sched.N_TurnLeft(KK);  carN.TurnRight = sched.N_TurnRight(KK);
        carN.Vd = sched.N_Vd(KK);              carN.Th = sched.N_Th(KK);
        CarN(end+1) = carN;
    end
    %
    % % SOUTH (spawn at Y=+150, moving -Y)
    if CARS && sched.S_intent(KK) && canSpawnY(CarS, 150, spawn_gap_min)
        carS = Car(CarIDS, 1.5, 150); CarIDS = CarIDS + 1;
        carS.Dir = 'S';
        carS.TurnLeft = sched.S_TurnLeft(KK);  carS.TurnRight = sched.S_TurnRight(KK);
        carS.Vd = sched.S_Vd(KK);              carS.Th = sched.S_Th(KK);
        CarS(end+1) = carS;
    end

    % EAST (spawn at X=-150, moving +X)
    if CARE && sched.E_intent(KK) && canSpawnX(CarE, -150, spawn_gap_min)
        carE = Car(CarIDE, -150, 1.5); CarIDE = CarIDE + 1;
        carE.Dir = 'E';
        carE.TurnLeft = sched.E_TurnLeft(KK);  carE.TurnRight = sched.E_TurnRight(KK);
        carE.Vd = sched.E_Vd(KK);              carE.Th = sched.E_Th(KK);
        CarE(end+1) = carE;
    end

    % % WEST (spawn at X=+150, moving -X)
    if CARW && sched.W_intent(KK) && canSpawnX(CarW, 150, spawn_gap_min)
        carW = Car(CarIDW, 150, -1.5); CarIDW = CarIDW + 1;
        carW.Dir = 'W';
        carW.TurnLeft = sched.W_TurnLeft(KK);  carW.TurnRight = sched.W_TurnRight(KK);
        carW.Vd = sched.W_Vd(KK);              carW.Th = sched.W_Th(KK);
        CarW(end+1) = carW;
    end


    %% === Compute Acceleration and movement ===
    % % N
    % for i = 1:length(CarN)
    %     % CarN(i).Lpos = i;
    %     if i > 1
    %         CarN(i).Ac = CarN(i).MPC(CarN(i), CarN(i-1));
    %     else
    %         CarN(i).Ac = 0.5 * (CarN(i).Vd - CarN(i).V);
    %     end
    % end
    %
    % % S
    % for i = 1:length(CarS)
    %     % CarS(i).Lpos = i;
    %     if i > 1
    %         CarS(i).Ac = CarS(i).MPC(CarS(i), CarS(i-1));
    %     else
    %         CarS(i).Ac = 0.5 * (CarS(i).Vd - CarS(i).V);
    %     end
    % end
    %
    % % E
    % for i = 1:length(CarE)
    %     % CarE(i).Lpos = i;
    %     if i > 1
    %         CarE(i).Ac = CarE(i).MPC(CarE(i), CarE(i-1));
    %     else
    %         CarE(i).Ac = 0.5 * (CarE(i).Vd - CarE(i).V);
    %     end
    % end
    %
    % % W
    % for i = 1:length(CarW)
    %     % CarW(i).Lpos = i;
    %     if i > 1
    %         CarW(i).Ac = CarW(i).MPC(CarW(i), CarW(i-1));
    %     else
    %         CarW(i).Ac = 0.5 * (CarW(i).Vd - CarW(i).V);
    %     end
    % end

    %% === Compute Acceleration and movement with Traffic Light Algorithm ===

    % NORTH → SOUTH
    % Pre-build stop-line dummy cars for each direction (Fix 9).
    % These are reused each timestep instead of being re-instantiated
    % inside the per-car loop, avoiding repeated rand() calls and
    % struct initialization that the Car constructor performs.
    % Only the position fields are updated inside the loop.
    dummyN = Car(-1, 0, 0); dummyN.Dir = 'N'; dummyN.V = 0;
    dummyS = Car(-1, 0, 0); dummyS.Dir = 'S'; dummyS.V = 0;
    dummyE = Car(-1, 0, 0); dummyE.Dir = 'E'; dummyE.V = 0;
    dummyW = Car(-1, 0, 0); dummyW.Dir = 'W'; dummyW.V = 0;

    % Virtual free-driving lead: placed 200 m ahead at desired speed.
    % Using MPC with a far-away lead (Fix 4) keeps the free-driving
    % acceleration produced by the same QP as car-following, rather than
    % a separate proportional gain (0.5*(Vd-V)) that produces a different
    % profile and inflates inter-configuration variance.
    freeDriveLead = Car(-2, 0, 0); freeDriveLead.V = 0; % position set below

    % Lead-car cache for each direction (Fix 8).
    % The MPC blocks below populate these; the collision prevention block
    % reuses them instead of running the same O(n²) search a second time.
    leadCarE = cell(1, length(CarE));
    leadCarW = cell(1, length(CarW));
    leadCarN = cell(1, length(CarN));
    leadCarS = cell(1, length(CarS));

    for i = 1:length(CarN)
        car = CarN(i);
        isRedOrYellow = strcmp(TrafficLight.NS, 'red') || strcmp(TrafficLight.NS, 'yellow');
        isBeforeStopLine = car.Y < -stop_line;  % NORTH cars have Y < -stop_line before crossing

        % Find the ACTUAL car ahead (closest car with Y > car.Y for NORTH direction)
        leadCar = [];
        minDist = inf;
        for j = 1:length(CarN)
            if j ~= i && CarN(j).Y > car.Y  % Car j is ahead of car i (larger Y for NORTH)
                dist = CarN(j).Y - car.Y;
                if dist < minDist
                    minDist = dist;
                    leadCar = CarN(j);
                end
            end
        end

        % Cache for collision prevention (Fix 8)
        leadCarN{i} = leadCar;

        if isRedOrYellow && isBeforeStopLine
            % Red/Yellow: stop at the line (or follow queue)
            if ~isempty(leadCar) && leadCar.Y < -stop_line + car.R0
                car.Ac = car.MPC(car, leadCar);
            else
                dummyN.Y = -stop_line + car.R0;
                car.Ac = car.MPC(car, dummyN);
            end
        else
            % Green or past stop line: follow lead or free-drive via MPC
            if ~isempty(leadCar)
                car.Ac = car.MPC(car, leadCar);
            else
                freeDriveLead.Dir = 'N';
                freeDriveLead.Y   = car.Y + 200;
                freeDriveLead.V   = car.Vd;
                car.Ac = car.MPC(car, freeDriveLead);
            end
        end
        CarN(i) = car;
    end


    % SOUTH → NORTH
    for i = 1:length(CarS)
        car = CarS(i);
        isRedOrYellow = strcmp(TrafficLight.NS, 'red') || strcmp(TrafficLight.NS, 'yellow');
        isBeforeStopLine = car.Y > stop_line;  % SOUTH cars have Y > stop_line before crossing

        % Find the ACTUAL car ahead (closest car with Y < car.Y for SOUTH direction)
        leadCar = [];
        minDist = inf;
        for j = 1:length(CarS)
            if j ~= i && CarS(j).Y < car.Y  % Car j is ahead of car i (smaller Y for SOUTH)
                dist = car.Y - CarS(j).Y;
                if dist < minDist
                    minDist = dist;
                    leadCar = CarS(j);
                end
            end
        end

        if isRedOrYellow && isBeforeStopLine
            if ~isempty(leadCar) && leadCar.Y > stop_line - car.R0
                car.Ac = car.MPC(car, leadCar);
            else
                dummyS.Y = stop_line - car.R0;
                car.Ac = car.MPC(car, dummyS);
            end
        else
            if ~isempty(leadCar)
                car.Ac = car.MPC(car, leadCar);
            else
                freeDriveLead.Dir = 'S';
                freeDriveLead.Y   = car.Y - 200;
                freeDriveLead.V   = car.Vd;
                car.Ac = car.MPC(car, freeDriveLead);
            end
        end
        leadCarS{i} = leadCar;   % cache for collision prevention (Fix 8)
        CarS(i) = car;
    end


    % WEST → EAST
    for i = 1:length(CarE)
        car = CarE(i);
        isRedOrYellow = strcmp(TrafficLight.EW, 'red') || strcmp(TrafficLight.EW, 'yellow');
        isBeforeStopLine = car.X < -stop_line;

        % Find the ACTUAL car ahead (closest car with X > car.X, same direction)
        leadCar = [];
        minDist = inf;
        for j = 1:length(CarE)
            if j ~= i && CarE(j).X > car.X  % Car j is ahead of car i
                dist = CarE(j).X - car.X;
                if dist < minDist
                    minDist = dist;
                    leadCar = CarE(j);
                end
            end
        end

        if isRedOrYellow && isBeforeStopLine
            if ~isempty(leadCar) && leadCar.X < -stop_line + car.R0
                car.Ac = car.MPC(car, leadCar);
            else
                dummyE.X = -stop_line + car.R0;
                car.Ac = car.MPC(car, dummyE);
            end
        else
            if ~isempty(leadCar)
                car.Ac = car.MPC(car, leadCar);
            else
                freeDriveLead.Dir = 'E';
                freeDriveLead.X   = car.X + 200;
                freeDriveLead.V   = car.Vd;
                car.Ac = car.MPC(car, freeDriveLead);
            end
        end

        leadCarE{i} = leadCar;   % cache for collision prevention (Fix 8)
        CarE(i) = car;
    end

    % EAST → WEST
    for i = 1:length(CarW)
        car = CarW(i);
        isRedOrYellow = strcmp(TrafficLight.EW, 'red') || strcmp(TrafficLight.EW, 'yellow');
        isBeforeStopLine = car.X > stop_line;  % WEST cars have X > stop_line before crossing

        % Find the ACTUAL car ahead (closest car with X < car.X for WEST direction)
        leadCar = [];
        minDist = inf;
        for j = 1:length(CarW)
            if j ~= i && CarW(j).X < car.X  % Car j is ahead of car i (smaller X for WEST)
                dist = car.X - CarW(j).X;
                if dist < minDist
                    minDist = dist;
                    leadCar = CarW(j);
                end
            end
        end

        if isRedOrYellow && isBeforeStopLine
            if ~isempty(leadCar) && leadCar.X > stop_line - car.R0
                car.Ac = car.MPC(car, leadCar);
            else
                dummyW.X = stop_line - car.R0;
                car.Ac = car.MPC(car, dummyW);
            end
        else
            if ~isempty(leadCar)
                car.Ac = car.MPC(car, leadCar);
            else
                freeDriveLead.Dir = 'W';
                freeDriveLead.X   = car.X - 200;
                freeDriveLead.V   = car.Vd;
                car.Ac = car.MPC(car, freeDriveLead);
            end
        end
        leadCarW{i} = leadCar;   % cache for collision prevention (Fix 8)
        CarW(i) = car;
    end

    %% === HARD COLLISION PREVENTION ===
    % Lead cars are reused from the caches populated in the MPC blocks above
    % (Fix 8) — no duplicate O(n²) search needed here.

    % --- East direction collision prevention ---
    for i = 1:length(CarE)
        car = CarE(i);
        leadCar = leadCarE{i};  % reuse cached result

        if ~isempty(leadCar)
            gap = leadCar.X - car.X;
            minSafeGap = 5.0;  % Minimum safe following distance

            % Predict positions after this timestep
            myNextX = car.X + car.V * dt + 0.5 * car.Ac * dt^2;
            leadNextX = leadCar.X + leadCar.V * dt + 0.5 * leadCar.Ac * dt^2;
            nextGap = leadNextX - myNextX;

            % If we would get too close, override acceleration
            if nextGap < minSafeGap || gap < minSafeGap
                % Calculate required deceleration to match lead car's speed
                if car.V > leadCar.V
                    % Need to slow down
                    requiredDecel = -(car.V - leadCar.V)^2 / (2 * max(gap - minSafeGap, 0.1));
                    requiredDecel = max(requiredDecel, -7.0);  % Limit to max braking
                    CarE(i).Ac = min(car.Ac, requiredDecel);
                end

                % Emergency: if gap is already too small, hard brake
                if gap < minSafeGap
                    CarE(i).Ac = -7.0;
                    fprintf('COLLISION OVERRIDE Car %d: gap=%.2f to Car %d\n', ...
                        car.ID, gap, leadCar.ID);
                end
            end
        end
    end

    % --- WEST direction collision prevention ---
    for i = 1:length(CarW)
        car = CarW(i);
        % Find the actual car ahead (closest car with smaller X)
        leadCar = [];
        minDist = inf;
        for j = 1:length(CarW)
            if j ~= i && CarW(j).X < car.X
                dist = car.X - CarW(j).X;
                if dist < minDist
                    minDist = dist;
                    leadCar = CarW(j);
                end
            end
        end
        if ~isempty(leadCar)
            gap = car.X - leadCar.X;
            minSafeGap = 5.0;  % Minimum safe following distance
            % Predict positions after this timestep
            myNextX = car.X - car.V * dt - 0.5 * car.Ac * dt^2;
            leadNextX = leadCar.X - leadCar.V * dt - 0.5 * leadCar.Ac * dt^2;
            nextGap = myNextX - leadNextX;
            % If we would get too close, override acceleration
            if nextGap < minSafeGap || gap < minSafeGap
                % Calculate required deceleration to match lead car's speed
                if car.V > leadCar.V
                    % Need to slow down
                    requiredDecel = -(car.V - leadCar.V)^2 / (2 * max(gap - minSafeGap, 0.1));
                    requiredDecel = max(requiredDecel, -7.0);  % Limit to max braking
                    CarW(i).Ac = min(car.Ac, requiredDecel);
                end
                % Emergency: if gap is already too small, hard brake
                if gap < minSafeGap
                    CarW(i).Ac = -7.0;
                    fprintf('COLLISION OVERRIDE CarW %d: gap=%.2f to Car %d\n', ...
                        car.ID, gap, leadCar.ID);
                end
            end
        end
    end

    % --- NORTH direction collision prevention ---
    for i = 1:length(CarN)
        car = CarN(i);
        % Find the actual car ahead (closest car with larger Y)
        leadCar = [];
        minDist = inf;
        for j = 1:length(CarN)
            if j ~= i && CarN(j).Y > car.Y
                dist = CarN(j).Y - car.Y;
                if dist < minDist
                    minDist = dist;
                    leadCar = CarN(j);
                end
            end
        end
        if ~isempty(leadCar)
            gap = leadCar.Y - car.Y;
            minSafeGap = 5.0;  % Minimum safe following distance
            % Predict positions after this timestep
            myNextY = car.Y + car.V * dt + 0.5 * car.Ac * dt^2;
            leadNextY = leadCar.Y + leadCar.V * dt + 0.5 * leadCar.Ac * dt^2;
            nextGap = leadNextY - myNextY;
            % If we would get too close, override acceleration
            if nextGap < minSafeGap || gap < minSafeGap
                % Calculate required deceleration to match lead car's speed
                if car.V > leadCar.V
                    % Need to slow down
                    requiredDecel = -(car.V - leadCar.V)^2 / (2 * max(gap - minSafeGap, 0.1));
                    requiredDecel = max(requiredDecel, -7.0);  % Limit to max braking
                    CarN(i).Ac = min(car.Ac, requiredDecel);
                end
                % Emergency: if gap is already too small, hard brake
                if gap < minSafeGap
                    CarN(i).Ac = -7.0;
                    fprintf('COLLISION OVERRIDE CarN %d: gap=%.2f to Car %d\n', ...
                        car.ID, gap, leadCar.ID);
                end
            end
        end
    end

    % --- SOUTH direction collision prevention ---
    for i = 1:length(CarS)
        car = CarS(i);
        % Find the actual car ahead (closest car with smaller Y)
        leadCar = [];
        minDist = inf;
        for j = 1:length(CarS)
            if j ~= i && CarS(j).Y < car.Y
                dist = car.Y - CarS(j).Y;
                if dist < minDist
                    minDist = dist;
                    leadCar = CarS(j);
                end
            end
        end
        if ~isempty(leadCar)
            gap = car.Y - leadCar.Y;
            minSafeGap = 5.0;  % Minimum safe following distance
            % Predict positions after this timestep
            myNextY = car.Y - car.V * dt - 0.5 * car.Ac * dt^2;
            leadNextY = leadCar.Y - leadCar.V * dt - 0.5 * leadCar.Ac * dt^2;
            nextGap = myNextY - leadNextY;
            % If we would get too close, override acceleration
            if nextGap < minSafeGap || gap < minSafeGap
                % Calculate required deceleration to match lead car's speed
                if car.V > leadCar.V
                    % Need to slow down
                    requiredDecel = -(car.V - leadCar.V)^2 / (2 * max(gap - minSafeGap, 0.1));
                    requiredDecel = max(requiredDecel, -7.0);  % Limit to max braking
                    CarS(i).Ac = min(car.Ac, requiredDecel);
                end
                % Emergency: if gap is already too small, hard brake
                if gap < minSafeGap
                    CarS(i).Ac = -7.0;
                    fprintf('COLLISION OVERRIDE CarS %d: gap=%.2f to Car %d\n', ...
                        car.ID, gap, leadCar.ID);
                end
            end
        end
    end

    %% Turn Left
    % W -> S
    used_dummyW_left = false;
    for i = length(CarW):-1:1
        car = CarW(i);
        if car.TurnLeft == 1 && car.TurnedLeft == 0
            % Slow down approaching the turn zone
            if car.X > turn_release && car.X <= turn_start
                % Check if car ahead (i-1) is also a left-turner not yet turned
                if i > 1 && CarW(i-1).TurnLeft == 1 && CarW(i-1).TurnedLeft == 0
                    % Follow the car ahead
                    car.Ac = car.MPC(car, CarW(i-1));
                elseif car.V > 2.0 % Only use dummy if car is moving fast
                    % No left-turner ahead, use dummy
                    if ~used_dummyW_left
                        dummy = Car(-1, 0, 0);
                        dummy.X = turn_release - car.R0;
                        dummy.Y = car.Y;
                        dummy.V = 0;
                        car.Ac = car.MPC(car, dummy);
                        used_dummyW_left = true;
                    end
                end
            end

            % Execute turn when reaching release point
            if car.X <= turn_release
                spawnY = -appear;
                if isSpawnClear(CarS, 'Y', spawnY, 5.0)
                    car.Dir = 'S';
                    car.X = 1.5;
                    car.Y = -appear;
                    car.V = 3;
                    car.Ac = 0.5 * (car.Vd - car.V);
                    car.TurnedLeft = 1;

                    insertIndex = find([CarS.Y] > car.Y, 1);
                    if isempty(insertIndex)
                        CarS(end+1) = car;
                    else
                        CarS(insertIndex+1:end+1) = CarS(insertIndex:end);
                        CarS(insertIndex) = car;
                    end
                    CarW(i) = [];
                    continue;
                else
                    % Wait - spawn blocked
                    car.V = 0;
                    car.Ac = 0;
                end
            end
            CarW(i) = car;
        end
    end

    % E -> N
    used_dummyE_left = false;
    for i = length(CarE):-1:1
        car = CarE(i);
        if car.TurnLeft == 1 && car.TurnedLeft == 0
            % Slow down approaching the turn zone
            if car.X < -turn_release && car.X >= -turn_start
                % Check if car ahead (i-1) is also a left-turner not yet turned
                if i > 1 && CarE(i-1).TurnLeft == 1 && CarE(i-1).TurnedLeft == 0
                    % Follow the car ahead
                    car.Ac = car.MPC(car, CarE(i-1));
                elseif car.V > 2.0 % Only use dummy if car is moving fast
                    % No left-turner ahead, use dummy
                    if ~used_dummyE_left
                        dummy = Car(-1, 0, 0);
                        dummy.X = -turn_release + car.R0;
                        dummy.Y = car.Y;
                        dummy.V = 0;
                        car.Ac = car.MPC(car, dummy);
                        used_dummyE_left = true;
                    end
                end
            end

            % Execute turn when reaching release point
            if car.X >= -turn_release
                spawnY = appear;
                if isSpawnClear(CarN, 'Y', spawnY, 5.0)
                    car.Dir = 'N';
                    car.X = -1.5;
                    car.Y = appear;
                    car.V = 3;
                    car.Ac = 0.5 * (car.Vd - car.V);
                    car.TurnedLeft = 1;
                    insertIndex = find([CarN.Y] < car.Y, 1);
                    if isempty(insertIndex)
                        CarN(end+1) = car;
                    else
                        CarN(insertIndex+1:end+1) = CarN(insertIndex:end);
                        CarN(insertIndex) = car;
                    end
                    CarE(i) = [];
                    continue;
                else
                    % Wait - spawn blocked
                    car.V = 0;
                    car.Ac = 0;
                end
            end
            CarE(i) = car;
        end
    end

    % S -> E
    used_dummyS_left = false;
    for i = length(CarS):-1:1
        car = CarS(i);
        if car.TurnLeft == 1 && car.TurnedLeft == 0
            % Slow down approaching the turn zone
            if car.Y > turn_release && car.Y <= turn_start
                % Check if car ahead (i-1) is also a left-turner not yet turned
                if i > 1 && CarS(i-1).TurnLeft == 1 && CarS(i-1).TurnedLeft == 0
                    % Follow the car ahead
                    car.Ac = car.MPC(car, CarS(i-1));
                elseif car.V > 2.0 % Only use dummy if car is moving fast
                    % No left-turner ahead, use dummy
                    if ~used_dummyS_left
                        dummy = Car(-1, 0, 0);
                        dummy.X = car.X;
                        dummy.Y = turn_release - car.R0;
                        dummy.V = 0;
                        car.Ac = car.MPC(car, dummy);
                        used_dummyS_left = true;
                    end
                end
            end

            % Execute turn when reaching release point
            if car.Y <= turn_release
                spawnX = appear;
                if isSpawnClear(CarE, 'X', spawnX, 5.0)
                    car.Dir = 'E';
                    car.X = appear;
                    car.Y = 1.5;
                    car.V = 3;
                    car.Ac = 0.5 * (car.Vd - car.V);
                    car.TurnedLeft = 1;
                    insertIndex = find([CarE.X] < car.X, 1);
                    if isempty(insertIndex)
                        CarE(end+1) = car;
                    else
                        CarE(insertIndex+1:end+1) = CarE(insertIndex:end);
                        CarE(insertIndex) = car;
                    end
                    CarS(i) = [];
                    continue;
                else
                    % Wait - spawn blocked
                    car.V = 0;
                    car.Ac = 0;
                end
            end
            CarS(i) = car;
        end
    end

    % N -> W
    used_dummyN_left = false;
    for i = length(CarN):-1:1
        car = CarN(i);
        if car.TurnLeft == 1 && car.TurnedLeft == 0
            % Slow down approaching the turn zone
            if car.Y < -turn_release && car.Y >= -turn_start
                % Check if car ahead (i-1) is also a left-turner not yet turned
                if i > 1 && CarN(i-1).TurnLeft == 1 && CarN(i-1).TurnedLeft == 0
                    % Follow the car ahead
                    car.Ac = car.MPC(car, CarN(i-1));
                elseif car.V > 2.0 % Only use dummy if car is moving fast
                    % No left-turner ahead, use dummy
                    if ~used_dummyN_left
                        dummy = Car(-1, 0, 0);
                        dummy.X = car.X;
                        dummy.Y = -turn_release + car.R0;
                        dummy.V = 0;
                        car.Ac = car.MPC(car, dummy);
                        used_dummyN_left = true;
                    end
                end
            end

            % Execute turn when reaching release point
            if car.Y >= -turn_release
                spawnX = -appear;
                if isSpawnClear(CarW, 'X', spawnX, 5.0)
                    car.Dir = 'W';
                    car.X = -appear;
                    car.Y = -1.5;
                    car.V = 3;
                    car.Ac = 0.5 * (car.Vd - car.V);
                    car.TurnedLeft = 1;
                    insertIndex = find([CarW.X] > car.X, 1);
                    if isempty(insertIndex)
                        CarW(end+1) = car;
                    else
                        CarW(insertIndex+1:end+1) = CarW(insertIndex:end);
                        CarW(insertIndex) = car;
                    end
                    CarN(i) = [];
                    continue;
                else
                    % Wait - spawn blocked
                    car.V = 0;
                    car.Ac = 0;
                end
            end
            CarN(i) = car;
        end
    end

    %% Turn Right
    % W -> N
    used_dummyW_right = false;
    for i = length(CarW):-1:1
        car = CarW(i);
        if car.TurnRight == 1 && car.TurnedRight == 0

            % Approaching turn zone - slow down
            if car.X > turn_wait && car.X <= turn_start
                if i > 1 && CarW(i-1).TurnRight == 1 && CarW(i-1).TurnedRight == 0
                    car.Ac = car.MPC(car, CarW(i-1));
                elseif car.V > 2.0
                    if ~used_dummyW_right
                        dummy = Car(-1, 0, 0);
                        dummy.X = turn_wait - car.R0;
                        dummy.Y = car.Y;
                        dummy.V = 0;
                        dummy.Dir = 'W';
                        car.Ac = car.MPC(car, dummy);
                        used_dummyW_right = true;
                    end
                end
            end

            % At waiting point - check RSU decision
            if car.X <= turn_wait
                car.X = turn_wait;
                car.V = 0;
                car.Ac = 0;

                % ========== RSU-BASED DECISION ==========
                x2v = car.GetLastX2V();
                canTurn = false;

                if ~isempty(x2v) && isfield(x2v, 'turn_signal')
                    if x2v.turn_signal == 1
                        % RSU says: PROCEED
                        canTurn = true;
                    elseif x2v.turn_signal == -1
                        % RSU says: WAIT
                        canTurn = false;
                    else
                        % RSU has no opinion (turn_signal == 0)
                        % Fallback to traffic light rule
                        canTurn = ~strcmp(TrafficLight.EW, 'green');
                    end
                else
                    % No RSU data - fallback to traffic light
                    canTurn = ~strcmp(TrafficLight.EW, 'green');
                end
                % =========================================

                if canTurn
                    spawnY = appear - out_eps;
                    if isSpawnClear(CarN, 'Y', spawnY, 5.0)
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
            end
            CarW(i) = car;
        end
    end


    % E -> S
    used_dummyE_right = false;
    for i = length(CarE):-1:1
        car = CarE(i);
        if car.TurnRight == 1 && car.TurnedRight == 0

            % Approaching turn zone - slow down
            if car.X < -turn_wait && car.X >= -turn_start
                if i > 1 && CarE(i-1).TurnRight == 1 && CarE(i-1).TurnedRight == 0
                    car.Ac = car.MPC(car, CarE(i-1));
                elseif car.V > 2.0
                    if ~used_dummyE_right
                        dummy = Car(-1, 0, 0);
                        dummy.X = -turn_wait + car.R0;
                        dummy.Y = car.Y;
                        dummy.V = 0;
                        dummy.Dir = 'E';
                        car.Ac = car.MPC(car, dummy);
                        used_dummyE_right = true;
                    end
                end
            end

            % At waiting point - check RSU decision
            if car.X >= -turn_wait
                car.X = -turn_wait;
                car.V = 0;
                car.Ac = 0;

                % ========== RSU-BASED DECISION ==========
                x2v = car.GetLastX2V();
                canTurn = false;

                if ~isempty(x2v) && isfield(x2v, 'turn_signal')
                    if x2v.turn_signal == 1
                        canTurn = true;
                    elseif x2v.turn_signal == -1
                        canTurn = false;
                    else
                        canTurn = ~strcmp(TrafficLight.EW, 'green');
                    end
                else
                    canTurn = ~strcmp(TrafficLight.EW, 'green');
                end
                % =========================================

                if canTurn
                    spawnY = -(appear - out_eps);
                    if isSpawnClear(CarS, 'Y', spawnY, 5.0)
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
            end
            CarE(i) = car;
        end
    end

    % S -> W
    used_dummyS_right = false;
    for i = length(CarS):-1:1
        car = CarS(i);
        if car.TurnRight == 1 && car.TurnedRight == 0

            % Approaching turn zone - slow down
            if car.Y > turn_wait && car.Y <= turn_start
                if i > 1 && CarS(i-1).TurnRight == 1 && CarS(i-1).TurnedRight == 0
                    car.Ac = car.MPC(car, CarS(i-1));
                elseif car.V > 2.0
                    if ~used_dummyS_right
                        dummy = Car(-1, 0, 0);
                        dummy.Y = turn_wait - car.R0;
                        dummy.X = car.X;
                        dummy.V = 0;
                        dummy.Dir = 'S';
                        car.Ac = car.MPC(car, dummy);
                        used_dummyS_right = true;
                    end
                end
            end

            % At waiting point - check RSU decision
            if car.Y <= turn_wait
                car.Y = turn_wait;
                car.V = 0;
                car.Ac = 0;

                % ========== RSU-BASED DECISION ==========
                x2v = car.GetLastX2V();
                canTurn = false;

                if ~isempty(x2v) && isfield(x2v, 'turn_signal')
                    if x2v.turn_signal == 1
                        canTurn = true;
                    elseif x2v.turn_signal == -1
                        canTurn = false;
                    else
                        canTurn = ~strcmp(TrafficLight.NS, 'green');
                    end
                else
                    canTurn = ~strcmp(TrafficLight.NS, 'green');
                end
                % =========================================

                if canTurn
                    spawnX = -(appear - out_eps);
                    if isSpawnClear(CarW, 'X', spawnX, 5.0)
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
            end
            CarS(i) = car;
        end
    end

    % N -> E
    used_dummyN_right = false;
    for i = length(CarN):-1:1
        car = CarN(i);
        if car.TurnRight == 1 && car.TurnedRight == 0

            % Approaching turn zone - slow down
            if car.Y < -turn_wait && car.Y >= -turn_start
                if i > 1 && CarN(i-1).TurnRight == 1 && CarN(i-1).TurnedRight == 0
                    car.Ac = car.MPC(car, CarN(i-1));
                elseif car.V > 2.0
                    if ~used_dummyN_right
                        dummy = Car(-1, 0, 0);
                        dummy.Y = -turn_wait + car.R0;
                        dummy.X = car.X;
                        dummy.V = 0;
                        dummy.Dir = 'N';
                        car.Ac = car.MPC(car, dummy);
                        used_dummyN_right = true;
                    end
                end
            end

            % At waiting point - check RSU decision
            if car.Y >= -turn_wait
                car.Y = -turn_wait;
                car.V = 0;
                car.Ac = 0;

                % ========== RSU-BASED DECISION ==========
                x2v = car.GetLastX2V();
                canTurn = false;

                if ~isempty(x2v) && isfield(x2v, 'turn_signal')
                    if x2v.turn_signal == 1
                        canTurn = true;
                    elseif x2v.turn_signal == -1
                        canTurn = false;
                    else
                        canTurn = ~strcmp(TrafficLight.NS, 'green');
                    end
                else
                    canTurn = ~strcmp(TrafficLight.NS, 'green');
                end
                % =========================================

                if canTurn
                    spawnX = appear - out_eps;
                    if isSpawnClear(CarE, 'X', spawnX, 5.0)
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
            end
            CarN(i) = car;
        end
    end

    %% Debug
    if mod(KK, 20) == 0 && strcmp(TrafficLight.EW, 'red')  % Every 10 seconds during red
        fprintf('\n\n========== TIMESTEP %.1f | EW LIGHT: RED ==========\n', KK*dt);

        if ~isempty(CarW)
            fprintf('\n--- WEST-bound cars (E direction, X increasing) ---\n');
            for i = 1:min(5, length(CarW))  % Show first 5 cars
                c = CarW(i);
                fprintf('Car %d: X=%.2f V=%.2f Ac=%.2f', c.ID, c.X, c.V, c.Ac);
                if c.X < -stop_line
                    fprintf(' (BEFORE stop line)');
                else
                    fprintf(' (PAST stop line)');
                end
                fprintf('\n');
            end
        end

        fprintf('\nStop line at X = %.2f\n', -stop_line);
        fprintf('===================================================\n\n');
    end

    %% === Update movement ===
    % Update N
    for i = 1:length(CarN)
        CarN(i).fdp(CarN(i));
        CarN(i).V = CarN(i).fdv(CarN(i));
    end

    % Update S
    for i = 1:length(CarS)
        CarS(i).fdp(CarS(i));
        CarS(i).V = CarS(i).fdv(CarS(i));
    end

    % Update E
    for i = 1:length(CarE)
        CarE(i).fdp(CarE(i));
        CarE(i).V = CarE(i).fdv(CarE(i));
    end

    % Update W
    for i = 1:length(CarW)
        CarW(i).fdp(CarW(i));
        CarW(i).V = CarW(i).fdv(CarW(i));
    end

    %% === Position Update with Collision Clamping ===
    % This is the LAST line of defense - ensures cars can NEVER pass through each other

    minSafeGap = 5.0;  % Minimum gap between cars

    % --- EAST direction position clamping ---
    for i = 1:length(CarE)
    car = CarE(i);
    nearestGap = inf; leadCar = [];
    for j = 1:length(CarE)
        if j ~= i && CarE(j).X > car.X
            g = CarE(j).X - car.X;
            if g < nearestGap
                nearestGap = g;
                leadCar = CarE(j);
            end
        end
    end
    if ~isempty(leadCar) && nearestGap < minSafeGap
        CarE(i).X = leadCar.X - minSafeGap;
        CarE(i).V = min(CarE(i).V, leadCar.V);
        % fprintf('POSITION CLAMPED CarE %d: behind Car %d, gap was %.2f\n', car.ID, leadCar.ID, nearestGap);
    end
end

    % --- WEST direction position clamping ---
    for i = 1:length(CarW)
    car = CarW(i);
    nearestGap = inf; leadCar = [];
    for j = 1:length(CarW)
        if j ~= i && CarW(j).X < car.X
            g = car.X - CarW(j).X;
            if g < nearestGap
                nearestGap = g;
                leadCar = CarW(j);
            end
        end
    end
    if ~isempty(leadCar) && nearestGap < minSafeGap
        CarW(i).X = leadCar.X + minSafeGap;
        CarW(i).V = min(CarW(i).V, leadCar.V);
        % fprintf('POSITION CLAMPED CarW %d: behind Car %d, gap was %.2f\n', car.ID, leadCar.ID, nearestGap);
    end
end

    % --- NORTH direction position clamping ---
    for i = 1:length(CarN)
    car = CarN(i);
    nearestGap = inf; leadCar = [];
    for j = 1:length(CarN)
        if j ~= i && CarN(j).Y > car.Y
            g = CarN(j).Y - car.Y;
            if g < nearestGap
                nearestGap = g;
                leadCar = CarN(j);
            end
        end
    end
    if ~isempty(leadCar) && nearestGap < minSafeGap
        CarN(i).Y = leadCar.Y - minSafeGap;
        CarN(i).V = min(CarN(i).V, leadCar.V);
        % fprintf('POSITION CLAMPED CarN %d: behind Car %d, gap was %.2f\n', car.ID, leadCar.ID, nearestGap);
    end
end

    % --- SOUTH direction position clamping ---
    for i = 1:length(CarS)
    car = CarS(i);
    nearestGap = inf; leadCar = [];
    for j = 1:length(CarS)
        if j ~= i && CarS(j).Y < car.Y
            g = car.Y - CarS(j).Y;
            if g < nearestGap
                nearestGap = g;
                leadCar = CarS(j);
            end
        end
    end
    if ~isempty(leadCar) && nearestGap < minSafeGap
        CarS(i).Y = leadCar.Y + minSafeGap;
        CarS(i).V = min(CarS(i).V, leadCar.V);
        % fprintf('POSITION CLAMPED CarS %d: behind Car %d, gap was %.2f\n', car.ID, leadCar.ID, nearestGap);
    end
end

    %% Sending data to RSU
    % NORTH cars send V2X to RSU
    for i = 1:length(CarN)
        v2x_packet = CarN(i).SendingV2X(tsec);
        RSU1.ReceivingV2X(v2x_packet);
    end

    % SOUTH cars send V2X to RSU
    for i = 1:length(CarS)
        v2x_packet = CarS(i).SendingV2X(tsec);
        RSU1.ReceivingV2X(v2x_packet);
    end

    % EAST cars send V2X to RSU
    for i = 1:length(CarE)
        v2x_packet = CarE(i).SendingV2X(tsec);
        RSU1.ReceivingV2X(v2x_packet);
    end

    % WEST cars send V2X to RSU
    for i = 1:length(CarW)
        v2x_packet = CarW(i).SendingV2X(tsec);
        RSU1.ReceivingV2X(v2x_packet);
    end

    %% RSU sends data back to car
    % Process NORTH cars
    for i = 1:length(CarN)
        car = CarN(i);

        % RSU computes response (right turn optimization if applicable)
        turn_signal = 0;
        recommended_V = NaN;
        recommended_Ac = NaN;
        priority = 0;
        message = '';

        if car.TurnRight && ~car.TurnedRight
            [vpos, turn_signal] = RSU1.RightTurnOpt(car.ID, tsec);
            if ~isempty(vpos)
                recommended_V = vpos.recommended_V;
                message = vpos.action;
            end
        end

        % Send X2V packet from RSU to car
        x2v_packet = RSU1.SendingX2V(tsec, car.ID, turn_signal, recommended_V, recommended_Ac, priority, message);
        car.ReceivingX2V(x2v_packet);
    end

    % Process SOUTH cars
    for i = 1:length(CarS)
        car = CarS(i);

        turn_signal = 0;
        recommended_V = NaN;
        recommended_Ac = NaN;
        priority = 0;
        message = '';

        if car.TurnRight && ~car.TurnedRight
            [vpos, turn_signal] = RSU1.RightTurnOpt(car.ID, tsec);
            if ~isempty(vpos)
                recommended_V = vpos.recommended_V;
                message = vpos.action;
            end
        end

        x2v_packet = RSU1.SendingX2V(tsec, car.ID, turn_signal, recommended_V, recommended_Ac, priority, message);
        car.ReceivingX2V(x2v_packet);
    end

    % Process EAST cars
    for i = 1:length(CarE)
        car = CarE(i);

        turn_signal = 0;
        recommended_V = NaN;
        recommended_Ac = NaN;
        priority = 0;
        message = '';

        if car.TurnRight && ~car.TurnedRight
            [vpos, turn_signal] = RSU1.RightTurnOpt(car.ID, tsec);
            if ~isempty(vpos)
                recommended_V = vpos.recommended_V;
                message = vpos.action;
            end
        end

        x2v_packet = RSU1.SendingX2V(tsec, car.ID, turn_signal, recommended_V, recommended_Ac, priority, message);
        car.ReceivingX2V(x2v_packet);
    end

    % Process WEST cars
    for i = 1:length(CarW)
        car = CarW(i);

        turn_signal = 0;
        recommended_V = NaN;
        recommended_Ac = NaN;
        priority = 0;
        message = '';

        if car.TurnRight && ~car.TurnedRight
            [vpos, turn_signal] = RSU1.RightTurnOpt(car.ID, tsec);
            if ~isempty(vpos)
                recommended_V = vpos.recommended_V;
                message = vpos.action;
            end
        end

        x2v_packet = RSU1.SendingX2V(tsec, car.ID, turn_signal, recommended_V, recommended_Ac, priority, message);
        car.ReceivingX2V(x2v_packet);
    end

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

    %% Remove cars from RSU tracking when out of bounds
    % NORTH - remove from RSU first, then from array
    for i = length(CarN):-1:1
        if abs(CarN(i).X) > 170 || abs(CarN(i).Y) > 170
            RSU1.RemoveVehicle(CarN(i).ID, 'N');
        end
    end

    % SOUTH - remove from RSU first
    for i = length(CarS):-1:1
        if abs(CarS(i).X) > 170 || abs(CarS(i).Y) > 170
            RSU1.RemoveVehicle(CarS(i).ID, 'S');
        end
    end

    % EAST - remove from RSU first
    for i = length(CarE):-1:1
        if abs(CarE(i).X) > 170 || abs(CarE(i).Y) > 170
            RSU1.RemoveVehicle(CarE(i).ID, 'E');
        end
    end

    % WEST - remove from RSU first
    for i = length(CarW):-1:1
        if abs(CarW(i).X) > 170 || abs(CarW(i).Y) > 170
            RSU1.RemoveVehicle(CarW(i).ID, 'W');
        end
    end


    %% === Fuel & Idle Tracking ===
    % Compute f_c per vehicle per timestep; classify idle state.
    % idle_type 'gap'  = right-turner stopped waiting for a gap
    % idle_type 'red'  = non-turning car stopped at a red/yellow light
    isRedNS = strcmp(TrafficLight.NS,'red') || strcmp(TrafficLight.NS,'yellow');
    isRedEW = strcmp(TrafficLight.EW,'red') || strcmp(TrafficLight.EW,'yellow');
    for i = 1:length(CarN)
        car = CarN(i);
        if car.TurnRight && ~car.TurnedRight && car.V < 0.5
            itype = 'gap';
        elseif ~car.TurnRight && isRedNS && car.Y < -stop_line && car.V < 0.5
            itype = 'red';
        else, itype = 'none'; end
        car.UpdateFuel(dt, itype);
    end
    for i = 1:length(CarS)
        car = CarS(i);
        if car.TurnRight && ~car.TurnedRight && car.V < 0.5
            itype = 'gap';
        elseif ~car.TurnRight && isRedNS && car.Y > stop_line && car.V < 0.5
            itype = 'red';
        else, itype = 'none'; end
        car.UpdateFuel(dt, itype);
    end
    for i = 1:length(CarE)
        car = CarE(i);
        if car.TurnRight && ~car.TurnedRight && car.V < 0.5
            itype = 'gap';
        elseif ~car.TurnRight && isRedEW && car.X < -stop_line && car.V < 0.5
            itype = 'red';
        else, itype = 'none'; end
        car.UpdateFuel(dt, itype);
    end
    for i = 1:length(CarW)
        car = CarW(i);
        if car.TurnRight && ~car.TurnedRight && car.V < 0.5
            itype = 'gap';
        elseif ~car.TurnRight && isRedEW && car.X > stop_line && car.V < 0.5
            itype = 'red';
        else, itype = 'none'; end
        car.UpdateFuel(dt, itype);
    end

    % Harvest fuel stats from vehicles that are about to leave the grid
    exitMaskN = abs([CarN.X]) > 170 | abs([CarN.Y]) > 170;
    exitMaskS = abs([CarS.X]) > 170 | abs([CarS.Y]) > 170;
    exitMaskE = abs([CarE.X]) > 170 | abs([CarE.Y]) > 170;
    exitMaskW = abs([CarW.X]) > 170 | abs([CarW.Y]) > 170;
    for idx = find(exitMaskN), FuelLog(end+1) = fuelEntry(CarN(idx)); end
    for idx = find(exitMaskS), FuelLog(end+1) = fuelEntry(CarS(idx)); end
    for idx = find(exitMaskE), FuelLog(end+1) = fuelEntry(CarE(idx)); end
    for idx = find(exitMaskW), FuelLog(end+1) = fuelEntry(CarW(idx)); end

    %% === Remove cars out of bounds ===
    % N
    CarN = CarN(abs([CarN.X]) <= 170 & abs([CarN.Y]) <= 170);
    % S
    CarS = CarS(abs([CarS.X]) <= 170 & abs([CarS.Y]) <= 170);
    % E
    CarE = CarE(abs([CarE.X]) <= 170 & abs([CarE.Y]) <= 170);
    % W
    CarW = CarW(abs([CarW.X]) <= 170 & abs([CarW.Y]) <= 170);
end

%% Print RSU
fprintf('\n\n');
RSU1.PrintSummary();

% Print V2X statistics
fprintf('\n=== V2X Communication Statistics ===\n');
fprintf('Total V2X records stored at RSU:\n');
dirs = {'N', 'S', 'E', 'W'};
for d = 1:length(dirs)
    dir = dirs{d};
    fields = fieldnames(RSU1.str_v2x_data.(dir));
    total_records = 0;
    for f = 1:length(fields)
        total_records = total_records + length(RSU1.str_v2x_data.(dir).(fields{f}).t);
    end
    fprintf('  Direction %s: %d records from %d vehicles\n', dir, total_records, length(fields));
end
fprintf('=====================================\n');


%% === Close video, save final static trajectory figure ===
close(vid);
fprintf('\nDemo video saved -> %s\n', fullfile(out_dir, 'demo_gpr_mpc_sparse.mp4'));

plot_EW_trajectories(LightLog, CarLogE, CarLogW, stop_line, ...
    fullfile(out_dir, 'demo_EW_trajectories.png'));
fprintf('Static trajectory figure saved -> %s\n', fullfile(out_dir, 'demo_EW_trajectories.png'));

%% === Helper Functions ===

function drawDemoFrame(axI, tsec, vid)
% DRAWDEMOFRAME Redraw the intersection panel for the current timestep
% and write one video frame (captures the whole figure, so it also
% includes whatever updateTrajectoryPanel has already drawn on axT).
% Cars genuinely move/spawn/despawn every frame, so this panel is
% legitimately redrawn from scratch each call — unlike the trajectory
% panel, which is built once and only extended (see updateTrajectoryPanel).

global CarN CarS CarE CarW TrafficLight
global RSUObjs

%% ---- Left panel: intersection view ----
cla(axI); hold(axI, 'on'); axis(axI, 'equal');
axis(axI, [-120 120 -120 120]);
xlabel(axI, 'X [m]'); ylabel(axI, 'Y [m]');
title(axI, sprintf('Intersection — t = %05.1f s', tsec));

lane_w = 3; road_w = 2 * lane_w;
fill(axI, [-road_w/2, -road_w/2, road_w/2, road_w/2], [-120, 120, 120, -120], [0.7 0.7 0.7], 'EdgeColor', 'none');
fill(axI, [-120, 120, 120, -120], [-road_w/2, -road_w/2, road_w/2, road_w/2], [0.7 0.7 0.7], 'EdgeColor', 'none');
for offset = [-lane_w, 0, lane_w]
    plot(axI, [-120 120], [offset offset], 'w--', 'LineWidth', 1);
    plot(axI, [offset offset], [-120 120], 'w--', 'LineWidth', 1);
end
text(axI, 0, 115, 'North', 'HorizontalAlignment','center', 'FontSize', 9);
text(axI, 0, -115, 'South', 'HorizontalAlignment','center', 'FontSize', 9);
text(axI, -115, 0, 'West', 'HorizontalAlignment','center', 'FontSize', 9);
text(axI, 115, 0, 'East', 'HorizontalAlignment','center', 'FontSize', 9);

% Traffic lights (EW is the only axis with real traffic in this demo)
r = 1.8; offset_light = 7; lw = 3;
colorMap = struct('green', [0 1 0], 'yellow', [1 1 0], 'red', [1 0 0]);
NS_color = colorMap.(TrafficLight.NS);
rectangle(axI, 'Position', [-offset_light-r, -offset_light-r, 2*r, 2*r], 'Curvature', [1 1], 'FaceColor', NS_color, 'EdgeColor', 'k', 'LineWidth', lw);
rectangle(axI, 'Position', [offset_light-r, offset_light-r, 2*r, 2*r], 'Curvature', [1 1], 'FaceColor', NS_color, 'EdgeColor', 'k', 'LineWidth', lw);
EW_color = colorMap.(TrafficLight.EW);
rectangle(axI, 'Position', [-offset_light-r, offset_light-r, 2*r, 2*r], 'Curvature', [1 1], 'FaceColor', EW_color, 'EdgeColor', 'k', 'LineWidth', lw);
rectangle(axI, 'Position', [offset_light-r, -offset_light-r, 2*r, 2*r], 'Curvature', [1 1], 'FaceColor', EW_color, 'EdgeColor', 'k', 'LineWidth', lw);

% RSU marker
if exist('RSUObjs','var') && ~isempty(RSUObjs)
    for k = 1:numel(RSUObjs)
        rsu = RSUObjs(k);
        rsu_size = 4;
        rectangle(axI, 'Position', [rsu.X-rsu_size/2, rsu.Y-rsu_size/2, rsu_size, rsu_size], ...
            'Curvature', 0.2, 'FaceColor', [0.2 0.2 0.2], 'EdgeColor', 'k', 'LineWidth', 1.5);
        text(axI, rsu.X, rsu.Y - rsu_size/2 - 2, sprintf('RSU %d', rsu.ID), ...
            'HorizontalAlignment', 'center', 'FontSize', 8, 'FontWeight', 'bold');
    end
end

% Cars — colored by direction, gray for left turn, purple for right turn
dirColorMap = struct('N', [1 0 0], 'S', [0 1 0], 'E', [0 0 1], 'W', [1 0.6 0]);
turnLColor = [0.5 0.5 0.5];
turnRColor = [0.5 0 0.5];
allDirs = {'N', 'S', 'E', 'W'};
allCars = {CarN, CarS, CarE, CarW};
carW_ = 2; carL_ = 2;
for d = 1:4
    cars = allCars{d};
    dirKey = allDirs{d};
    defaultColor = dirColorMap.(dirKey);
    for j = 1:length(cars)
        car = cars(j);
        if car.TurnLeft == 1
            clr = turnLColor;
        elseif car.TurnRight == 1
            clr = turnRColor;
        else
            clr = defaultColor;
        end
        if car.Dir == 'E' || car.Dir == 'W'
            rectangle(axI, 'Position', [car.X - carL_/2, car.Y - carW_/2, carL_, carW_], ...
                'FaceColor', clr, 'EdgeColor', 'k');
        else
            rectangle(axI, 'Position', [car.X - carW_/2, car.Y - carL_/2, carW_, carL_], ...
                'FaceColor', clr, 'EdgeColor', 'k');
        end
    end
end
hE = plot(axI, nan, nan, 's', 'MarkerFaceColor', dirColorMap.E, 'MarkerEdgeColor', 'k', 'MarkerSize', 10, 'DisplayName', 'East-bound');
hW = plot(axI, nan, nan, 's', 'MarkerFaceColor', dirColorMap.W, 'MarkerEdgeColor', 'k', 'MarkerSize', 10, 'DisplayName', 'West-bound');
hL = plot(axI, nan, nan, 's', 'MarkerFaceColor', turnLColor, 'MarkerEdgeColor', 'k', 'MarkerSize', 10, 'DisplayName', 'Left turn');
hR = plot(axI, nan, nan, 's', 'MarkerFaceColor', turnRColor, 'MarkerEdgeColor', 'k', 'MarkerSize', 10, 'DisplayName', 'Right turn');
legend(axI, [hE, hW, hL, hR], 'Location', 'southoutside', 'Orientation', 'horizontal', 'FontSize', 8);
grid(axI, 'on'); hold(axI, 'off');

drawnow;
frame = getframe(gcf);
writeVideo(vid, frame);

end

function [trajLineMapE, trajLineMapW, hLightStrip, lightSegStartT, lightSegPhase] = ...
    updateTrajectoryPanel(axT, tsec, TrafficLight, stop_line, CarLogE, CarLogW, ...
                           trajLineMapE, trajLineMapW, hLightStrip, lightSegStartT, lightSegPhase)
% UPDATETRAJECTORYPANEL Update ONLY the moving parts of the pre-built,
% paper-styled trajectory panel: extend the traffic-light strip patch and
% update each car's line XData/YData. Axes, labels, limits, legend, and
% "From East"/"From West" text are drawn once outside this function and
% are never touched here.

% --- Extend the traffic-light strip (grow the patch, don't recreate it) ---
colorMap = struct('green', [0 0.7 0], 'yellow', [0.95 0.75 0], 'red', [0.8 0 0]);
curPhase = TrafficLight.EW;
if isempty(lightSegPhase)
    lightSegPhase = curPhase;
    lightSegStartT = 0;
end
if ~strcmp(curPhase, lightSegPhase)
    % Phase changed: freeze the just-finished segment as its own patch,
    % then start a new growing patch for the new phase.
    dyL = 2;
    oldColor = colorMap.(lightSegPhase);
    patch(axT, [lightSegStartT tsec tsec lightSegStartT], [-dyL -dyL dyL dyL], oldColor, ...
        'EdgeColor','none', 'FaceAlpha', 0.9);
    uistack(findobj(axT,'Type','patch'), 'bottom');
    lightSegStartT = tsec;
    lightSegPhase  = curPhase;
end
dyL = 2;
curColor = colorMap.(lightSegPhase);
set(hLightStrip, 'XData', [lightSegStartT tsec tsec lightSegStartT], ...
                 'YData', [-dyL -dyL dyL dyL], 'FaceColor', curColor);

% --- Update each car's trajectory line (create once, then move it) ---
% Same signed-distance convention as function/plot_EW_trajectories.m:
%   East: max(0, -stop_line - X)   -> positive, shrinks to 0 at the stop line
%   West: -max(0, X - stop_line)   -> negative, shrinks to 0 at the stop line

if ~isempty(CarLogE)
    idsE = unique([CarLogE.ID]);
    for ii = 1:numel(idsE)
        id = idsE(ii);
        sel = CarLogE([CarLogE.ID] == id);
        [t, ix] = sort([sel.Time]); sel = sel(ix);
        y = max(0, (-stop_line) - [sel.X]);
        if ~isKey(trajLineMapE, id)
            if sel(1).TurnRight
                c = [0.85 0.15 0.15];
            elseif sel(1).TurnLeft
                c = [0.15 0.35 0.85];
            else
                c = [0 0 0];
            end
            h = plot(axT, t, y, 'Color', c, 'LineWidth', 1.4);
            trajLineMapE(id) = h;
        else
            set(trajLineMapE(id), 'XData', t, 'YData', y);
        end
    end
end

if ~isempty(CarLogW)
    idsW = unique([CarLogW.ID]);
    for ii = 1:numel(idsW)
        id = idsW(ii);
        sel = CarLogW([CarLogW.ID] == id);
        [t, ix] = sort([sel.Time]); sel = sel(ix);
        y = -max(0, [sel.X] - stop_line);
        if ~isKey(trajLineMapW, id)
            if sel(1).TurnRight
                c = [0.85 0.15 0.15];
            elseif sel(1).TurnLeft
                c = [0.15 0.35 0.85];
            else
                c = [0 0 0];
            end
            h = plot(axT, t, y, 'Color', c, 'LineWidth', 1.4);
            trajLineMapW(id) = h;
        else
            set(trajLineMapW(id), 'XData', t, 'YData', y);
        end
    end
end

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

function phase = phaseName(t, gNS, yNS, ar1, gEW, yEW)
% Returns a readable phase name based on t within the cycle
if t < gNS
    phase = 'NS_GREEN';
elseif t < gNS + yNS
    phase = 'NS_YELLOW';
elseif t < gNS + yNS + ar1
    phase = 'ALL_RED_1';
elseif t < gNS + yNS + ar1 + gEW
    phase = 'EW_GREEN';
elseif t < gNS + yNS + ar1 + gEW + yEW % still needs to check t < gNS + yNS + ar1 + yEW
    phase = 'EW_YELLOW';
else
    phase = 'ALL_RED_2';
end
end

function clear = isSpawnClear(carList, axis, spawnPos, safeGap)
% Check if spawn position is clear of other cars
% axis: 'X' or 'Y'
% spawnPos: the position where the car will spawn
% safeGap: minimum distance required (e.g., 5.0 meters)

clear = true;
if isempty(carList)
    return;
end

if axis == 'X'
    positions = [carList.X];
else
    positions = [carList.Y];
end

if any(abs(positions - spawnPos) < safeGap)
    clear = false;
end
end

function s = fuelEntry(car)
% Pack a car's fuel & idle counters into a FuelLog struct row.
s = struct('ID', car.ID, 'Dir', car.Dir, ...
    'TurnRight', car.TurnRight, 'TurnedRight', car.TurnedRight, ...
    'fuel_total',    car.fuel_total, ...
    'idle_gap_time', car.idle_gap_time, ...
    'idle_red_time', car.idle_red_time);
end