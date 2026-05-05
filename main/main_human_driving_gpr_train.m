%% GPR Arrival-Time Model Training
% Runs N_TRIALS independent human-driving simulations with fresh random
% traffic schedules, accumulates (dist, vel) -> tau training samples,
% then trains and saves the GPR model to simplot/gpr_arrival_model.mat.
%
% Deliberately does NOT touch traffic_schedule.mat so the other six
% scenario mains (which share that file for fair comparison) are unaffected.
%
% Run order: this file first, then main_v2v_idm_rsu_gpr.m / main_v2v_mpc_rsu_gpr.m.

clear classes %#ok<CLCLS>
close all; clear; clc;

addpath(genpath('C:/Users/monea/OneDrive/Documents/MATLAB/Traffic_Intersection/class'))
addpath(genpath('C:/Users/monea/OneDrive/Documents/MATLAB/Traffic_Intersection/function'))
addpath(genpath('C:/Users/monea/OneDrive/Documents/MATLAB/Traffic_Intersection/simplot'))
PROJECT_ROOT = 'C:/Users/monea/OneDrive/Documents/MATLAB/Traffic_Intersection';

%% ===== SETTINGS =====
N_TRIALS = 20;      % number of independent simulation runs to accumulate data
KKmax    = 3000;    % timesteps per trial (600 x 0.5s = 5 min)
dt_val   = 0.5;    % simulation timestep [s]

% Traffic light
Cycle = 120;
gNS = 54.5;  yNS = 3.5;  ar1 = 2.0;
gEW = 54.5;  yEW = 3.5;  ar2 = 2.0;

% Traffic demand (ramp stages)
DemandStages   = [0, 120, 300];
lambdaNS_axis  = [0.30, 0.45, 0.60];
lambdaEW_axis  = [0.25, 0.40, 0.55];

stop_line    = 10;    % [m] stop-line distance from centre
spawn_gap    = 8;     % [m] minimum spawn gap
turn_start   = 10;
turn_wait    = 5;
turn_release = 1.5;
out_eps      = 0.5;
appear       = 3;

% Training data accumulators
X_train = zeros(0, 2);   % [dist_to_stop_line, velocity]
y_train = zeros(0, 1);   % tau (s to stop-line crossing)

fprintf('=== GPR Training: %d trials x %d timesteps ===\n', N_TRIALS, KKmax);

%% ===== TRIAL LOOP =====
for trial = 1:N_TRIALS
    rng(trial);   % different seed each trial for diverse traffic

    fprintf('\n--- Trial %d/%d ---\n', trial, N_TRIALS);

    % Declare globals used by Car/IDM
    global CarN CarS CarE CarW TrafficLight dt KK
    dt = dt_val;

    % Fresh traffic schedule (NOT saved - private to this trial)
    sched = generateSchedule(KKmax, dt_val, DemandStages, lambdaNS_axis, lambdaEW_axis);

    % Initialise traffic
    CarN = Car.empty();  CarS = Car.empty();
    CarE = Car.empty();  CarW = Car.empty();
    CarIDN = 1000 + (trial-1)*500;
    CarIDS = 2000 + (trial-1)*500;
    CarIDE = 3000 + (trial-1)*500;
    CarIDW = 4000 + (trial-1)*500;

    % Seed one car per direction
    c = Car(CarIDN, -1.5, -150); c.Dir = 'N'; CarN(end+1) = c; CarIDN = CarIDN+1;
    c = Car(CarIDS,  1.5,  150); c.Dir = 'S'; CarS(end+1) = c; CarIDS = CarIDS+1;
    c = Car(CarIDE, -150, 1.5);  c.Dir = 'E'; CarE(end+1) = c; CarIDE = CarIDE+1;
    c = Car(CarIDW,  150,-1.5);  c.Dir = 'W'; CarW(end+1) = c; CarIDW = CarIDW+1;

    % Per-trial logs
    CarLog  = struct('Time',{},'ID',{},'Dir',{},'TurnLeft',{},'TurnRight',{},...
        'X',{},'Y',{},'V',{},'Ac',{});
    LightLog = struct('Time',{},'EW',{},'NS',{});

    % --- Simulation loop ---
    for KK = 1:KKmax
        t     = mod(KK*dt, Cycle);
        tsec  = KK * dt;

        % Traffic lights
        if     t < gNS,                         TrafficLight.NS='green';  TrafficLight.EW='red';
        elseif t < gNS+yNS,                     TrafficLight.NS='yellow'; TrafficLight.EW='red';
        elseif t < gNS+yNS+ar1,                 TrafficLight.NS='red';    TrafficLight.EW='red';
        elseif t < gNS+yNS+ar1+gEW,             TrafficLight.NS='red';    TrafficLight.EW='green';
        elseif t < gNS+yNS+ar1+gEW+yEW,         TrafficLight.NS='red';    TrafficLight.EW='yellow';
        else,                                    TrafficLight.NS='red';    TrafficLight.EW='red';
        end
        LightLog(end+1) = struct('Time',tsec,'EW',TrafficLight.EW,'NS',TrafficLight.NS);

        % Spawn
        if sched.N_intent(KK) && canSpawnY(CarN,-150,spawn_gap)
            c=Car(CarIDN,-1.5,-150); CarIDN=CarIDN+1; c.Dir='N';
            c.TurnLeft=sched.N_TurnLeft(KK); c.TurnRight=sched.N_TurnRight(KK);
            c.Vd=sched.N_Vd(KK); c.Th=sched.N_Th(KK); CarN(end+1)=c;
        end
        if sched.S_intent(KK) && canSpawnY(CarS,150,spawn_gap)
            c=Car(CarIDS,1.5,150); CarIDS=CarIDS+1; c.Dir='S';
            c.TurnLeft=sched.S_TurnLeft(KK); c.TurnRight=sched.S_TurnRight(KK);
            c.Vd=sched.S_Vd(KK); c.Th=sched.S_Th(KK); CarS(end+1)=c;
        end
        if sched.E_intent(KK) && canSpawnX(CarE,-150,spawn_gap)
            c=Car(CarIDE,-150,1.5); CarIDE=CarIDE+1; c.Dir='E';
            c.TurnLeft=sched.E_TurnLeft(KK); c.TurnRight=sched.E_TurnRight(KK);
            c.Vd=sched.E_Vd(KK); c.Th=sched.E_Th(KK); CarE(end+1)=c;
        end
        if sched.W_intent(KK) && canSpawnX(CarW,150,spawn_gap)
            c=Car(CarIDW,150,-1.5); CarIDW=CarIDW+1; c.Dir='W';
            c.TurnLeft=sched.W_TurnLeft(KK); c.TurnRight=sched.W_TurnRight(KK);
            c.Vd=sched.W_Vd(KK); c.Th=sched.W_Th(KK); CarW(end+1)=c;
        end

        % Acceleration (IDM with stop-line dummy for red)
        isRedNS = strcmp(TrafficLight.NS,'red')||strcmp(TrafficLight.NS,'yellow');
        isRedEW = strcmp(TrafficLight.EW,'red')||strcmp(TrafficLight.EW,'yellow');

        dN=Car(-1,0,0);dN.Dir='N';dN.V=0;
        dS=Car(-1,0,0);dS.Dir='S';dS.V=0;
        dE=Car(-1,0,0);dE.Dir='E';dE.V=0;
        dW=Car(-1,0,0);dW.Dir='W';dW.V=0;

        CarN = idmAccel(CarN,'N',isRedNS,stop_line,dN);
        CarS = idmAccel(CarS,'S',isRedNS,stop_line,dS);
        CarE = idmAccel(CarE,'E',isRedEW,stop_line,dE);
        CarW = idmAccel(CarW,'W',isRedEW,stop_line,dW);

        % Update position/velocity
        for i=1:length(CarN), CarN(i).fdp(CarN(i)); CarN(i).V=CarN(i).fdv(CarN(i)); end
        for i=1:length(CarS), CarS(i).fdp(CarS(i)); CarS(i).V=CarS(i).fdv(CarS(i)); end
        for i=1:length(CarE), CarE(i).fdp(CarE(i)); CarE(i).V=CarE(i).fdv(CarE(i)); end
        for i=1:length(CarW), CarW(i).fdp(CarW(i)); CarW(i).V=CarW(i).fdv(CarW(i)); end

        % Log (all cars, compact)
        for i=1:length(CarN)
            CarLog(end+1)=struct('Time',tsec,'ID',CarN(i).ID,'Dir',CarN(i).Dir,...
                'TurnLeft',CarN(i).TurnLeft,'TurnRight',CarN(i).TurnRight,...
                'X',CarN(i).X,'Y',CarN(i).Y,'V',CarN(i).V,'Ac',CarN(i).Ac);
        end
        for i=1:length(CarS)
            CarLog(end+1)=struct('Time',tsec,'ID',CarS(i).ID,'Dir',CarS(i).Dir,...
                'TurnLeft',CarS(i).TurnLeft,'TurnRight',CarS(i).TurnRight,...
                'X',CarS(i).X,'Y',CarS(i).Y,'V',CarS(i).V,'Ac',CarS(i).Ac);
        end
        for i=1:length(CarE)
            CarLog(end+1)=struct('Time',tsec,'ID',CarE(i).ID,'Dir',CarE(i).Dir,...
                'TurnLeft',CarE(i).TurnLeft,'TurnRight',CarE(i).TurnRight,...
                'X',CarE(i).X,'Y',CarE(i).Y,'V',CarE(i).V,'Ac',CarE(i).Ac);
        end
        for i=1:length(CarW)
            CarLog(end+1)=struct('Time',tsec,'ID',CarW(i).ID,'Dir',CarW(i).Dir,...
                'TurnLeft',CarW(i).TurnLeft,'TurnRight',CarW(i).TurnRight,...
                'X',CarW(i).X,'Y',CarW(i).Y,'V',CarW(i).V,'Ac',CarW(i).Ac);
        end

        % Remove out of bounds
        CarN=CarN(abs([CarN.X])<=170 & abs([CarN.Y])<=170);
        CarS=CarS(abs([CarS.X])<=170 & abs([CarS.Y])<=170);
        CarE=CarE(abs([CarE.X])<=170 & abs([CarE.Y])<=170);
        CarW=CarW(abs([CarW.X])<=170 & abs([CarW.Y])<=170);
    end % KK loop

    % --- Extract training samples from this trial ---
    [Xt, yt] = extractGPRSamples(CarLog, LightLog, stop_line);
    X_train = [X_train; Xt];
    y_train = [y_train; yt];
    fprintf('  Trial %d: +%d samples (total so far: %d)\n', trial, size(Xt,1), size(X_train,1));

end % trial loop

fprintf('\nTotal training samples: %d from %d trials\n', size(X_train,1), N_TRIALS);

%% ===== TRAIN GPR =====
MIN_SAMPLES = 100;
if size(X_train,1) >= MIN_SAMPLES

    % Subsample to cap at 2000 for training speed (fitrgp is O(N^3))
    % Full dataset is used for final refit after hyperparameter search
    MAX_FOR_OPT = 2000;
    if size(X_train,1) > MAX_FOR_OPT
        idx = randperm(size(X_train,1), MAX_FOR_OPT);
        Xo = X_train(idx,:);  yo = y_train(idx);
        fprintf('Subsampled to %d rows for hyperparameter optimisation...\n', MAX_FOR_OPT);
    else
        Xo = X_train;  yo = y_train;
    end

    fprintf('Optimising GPR hyperparameters (this may take 1-3 min)...\n');
    gpr_opt = fitrgp(Xo, yo, ...
        'KernelFunction',    'squaredexponential', ...
        'BasisFunction',     'linear', ...
        'Standardize',       true, ...
        'OptimizeHyperparameters', 'auto', ...
        'HyperparameterOptimizationOptions', struct('ShowPlots',false,'Verbose',0));

    % Refit on FULL dataset using the optimised hyperparameters
    kp = gpr_opt.KernelInformation.KernelParameters;
    fprintf('Refitting on full %d samples with fixed hyperparameters...\n', size(X_train,1));
    gpr_model = fitrgp(X_train, y_train, ...
        'KernelFunction',       'squaredexponential', ...
        'KernelParameters',     kp, ...
        'BasisFunction',        'linear', ...
        'Standardize',          true, ...
        'FitMethod',            'sr', ...   % sparse/subset-of-regressors for speed
        'PredictMethod',        'fic');     % fully independent conditional

    simplot_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'simplot');
    gpr_path    = fullfile(simplot_dir, 'gpr_arrival_model.mat');
    save(gpr_path, 'gpr_model', 'X_train', 'y_train');
    fprintf('GPR model saved -> %s\n', gpr_path);

    % Quick holdout evaluation
    n_ho    = min(500, size(X_train,1));
    idx_ho  = randperm(size(X_train,1), n_ho);
    X_ho    = X_train(idx_ho,:);
    y_true  = y_train(idx_ho);
    [y_pred, y_sd] = predict(gpr_model, X_ho);
    residuals = y_pred - y_true;
    rmse = sqrt(mean(residuals.^2));
    mae  = mean(abs(residuals));
    fprintf('Holdout (n=%d):  RMSE=%.2f s   MAE=%.2f s\n', n_ho, rmse, mae);

    %% --- Diagnostic plots ---
    out_dir = fullfile(PROJECT_ROOT, 'output');

    fig = figure('Visible','off','Color','white');
    set(fig, 'Position', [100 100 1200 380]); % slightly shorter height

    tiledlayout(fig,1,3,'TileSpacing','compact','Padding','compact');

    font_axis  = 12;
    font_label = 13;
    font_title = 13;

    %% ===== Panel 1: Predicted vs Actual =====
    
    ax1 = nexttile;

    scatter(y_true, y_pred, 12, y_sd, 'filled', 'MarkerFaceAlpha', 0.5);
    colormap(ax1, 'parula');

    cb1 = colorbar;
    cb1.Label.String = 'Std dev [s]';
    cb1.FontSize = font_axis;

    hold on;
    lims = [0, max(max(y_true), max(y_pred))*1.05];
    plot(lims, lims, 'r--', 'LineWidth', 1.5);
    hold off;

    xlabel('Actual \tau [s]', 'FontSize',font_label);
    ylabel('Predicted \tau [s]', 'FontSize',font_label);

    title(sprintf('Predicted vs Actual\nRMSE=%.2fs  MAE=%.2fs', rmse, mae), ...
        'FontSize',font_title,'FontWeight','bold');

    axis equal;
    xlim(lims); ylim(lims);

    grid on;
    box on;
    set(ax1,'FontSize',font_axis,'LineWidth',1.2);

    %% ===== Panel 2: Residual =====
    ax2 = nexttile;

    histogram(residuals, 40, 'Normalization','probability', ...
        'FaceColor',[0.20 0.60 0.86], 'EdgeColor','none', 'FaceAlpha',0.8);

    xline(0, 'r--', 'LineWidth', 1.5);

    xlabel('Residual \tau_{pred} - \tau_{true} [s]', 'FontSize',font_label);
    ylabel('Probability', 'FontSize',font_label);

    title(sprintf('Residual Distribution\nmean=%.2fs  std=%.2fs', ...
        mean(residuals), std(residuals)), ...
        'FontSize',font_title,'FontWeight','bold');

    grid on;
    box on;
    set(ax2,'FontSize',font_axis,'LineWidth',1.2);

    %% ===== Panel 3: GPR Surface =====
    ax3 = nexttile;

    d_grid  = linspace(2, 140, 50);
    v_grid  = linspace(0, 25,  50);
    [D, V]  = meshgrid(d_grid, v_grid);

    gap_rep = 8.0;
    G       = ones(size(D)) * gap_rep;

    Z = reshape(predict(gpr_model, [D(:), V(:), G(:)]), size(D));

    contourf(D, V, Z, 20, 'LineColor','none');
    colormap(ax3, 'jet');

    cb2 = colorbar;
    cb2.FontSize = font_axis;

    xlabel('Distance to stop line [m]', 'FontSize',font_label);
    ylabel('Velocity at green start [m/s]', 'FontSize',font_label);

    title(sprintf('GPR Prediction (gap=%.0fm)', gap_rep), ...
        'FontSize',font_title,'FontWeight','bold');

    grid on;
    box on;
    set(ax3,'FontSize',font_axis,'LineWidth',1.2);

    hold on;
    n_dots = min(800, size(X_train,1));
    idx_d  = randperm(size(X_train,1), n_dots);
    scatter(X_train(idx_d,1), X_train(idx_d,2), 4, 'w', ...
        'filled', 'MarkerFaceAlpha', 0.3);
    hold off;

    %% ---- IMPORTANT: REMOVE GLOBAL TITLE ----
    % (sgtitle removed)

    %% ---- Export ----
    plot_path = fullfile(out_dir, 'gpr_model_diagnostics.png');
    exportgraphics(fig, plot_path, 'Resolution', 300);
    fprintf('Diagnostic plot saved -> %s\n', plot_path);

    close(fig);
else
    warning('Only %d samples collected - need at least %d. Increase N_TRIALS.', ...
        size(X_train,1), MIN_SAMPLES);
end

fprintf('\nDone. You can now run main_v2v_idm_rsu_gpr.m or main_v2v_mpc_rsu_gpr.m\n');

%% ===== LOCAL HELPER FUNCTIONS =====

function sched = generateSchedule(KKmax, dt, stages, lamNS, lamEW)
dirs = {'N','S','E','W'};
sched.KKmax = KKmax; sched.dt = dt;
for di=1:4, d=dirs{di};
    sched.([d '_intent'])   = false(1,KKmax);
    sched.([d '_TurnLeft']) = false(1,KKmax);
    sched.([d '_TurnRight'])= false(1,KKmax);
    sched.([d '_Vd'])       = zeros(1,KKmax);
    sched.([d '_Th'])       = zeros(1,KKmax);
end
for kk=1:KKmax
    tsec=kk*dt;
    if     tsec < stages(2), k=1;
    elseif tsec < stages(3), k=2;
    else,                    k=3; end
    lams=[0.5*lamNS(k), 0.5*lamNS(k), 0.5*lamEW(k), 0.5*lamEW(k)];
    for di=1:4, d=dirs{di};
        p = 1-exp(-lams(di)*dt);
        sched.([d '_intent'])(kk)    = rand < p;
        tl = rand < 0.25;
        tr = false; if ~tl, tr = rand < 0.15; end
        sched.([d '_TurnLeft'])(kk)  = tl;
        sched.([d '_TurnRight'])(kk) = tr;
        sched.([d '_Vd'])(kk)        = 21 + 6*rand + 2;
        sched.([d '_Th'])(kk)        = 1.2 + 1.2*rand;
    end
end
end

function carList = idmAccel(carList, dir, isRed, stop_line, dummy)
for i = 1:length(carList)
    car = carList(i);
    lead = [];
    minD = inf;
    for j = 1:length(carList)
        if j==i, continue; end
        switch dir
            case 'N', d=carList(j).Y - car.Y;
            case 'S', d=car.Y - carList(j).Y;
            case 'E', d=carList(j).X - car.X;
            case 'W', d=car.X - carList(j).X;
        end
        if d>0 && d<minD, minD=d; lead=carList(j); end
    end
    isBefore = isBeforeStop(car, dir, stop_line);
    if isRed && isBefore
        if ~isempty(lead) && isBeforeStop(lead, dir, stop_line)
            car.Ac = Car.IDM(car, lead);
        else
            switch dir
                case 'N', dummy.Y = -stop_line + car.R0;
                case 'S', dummy.Y =  stop_line - car.R0;
                case 'E', dummy.X = -stop_line + car.R0;
                case 'W', dummy.X =  stop_line - car.R0;
            end
            car.Ac = Car.IDM(car, dummy);
        end
    else
        if ~isempty(lead)
            car.Ac = Car.IDM(car, lead);
        else
            car.Ac = 0.5*(car.Vd - car.V);
        end
    end
    carList(i) = car;
end
end

function b = isBeforeStop(car, dir, sl)
switch dir
    case 'N', b = car.Y < -sl;
    case 'S', b = car.Y >  sl;
    case 'E', b = car.X < -sl;
    case 'W', b = car.X >  sl;
    otherwise, b = true;
end
end

function [X, y] = extractGPRSamples(CarLog, LightLog, stop_line)
% Extract GPR training samples using GREEN-START snapshots.
%
% For each green phase start, snapshot every vehicle's (dist, vel) at that
% exact moment. tau = time from green start until the vehicle crosses.
%
% This matches exactly how the RSU uses the model: it fires at green start
% and predicts arrival times from vehicles' current states.
% Stopped vehicles (vel~0, queued at red) are naturally included.

X = zeros(0,3);  y = zeros(0,1);   % inputs: [dist, vel, gap_to_lead]
if isempty(CarLog) || isempty(LightLog), return; end

n_log = length(LightLog);

% Find green-start timesteps for EW and NS axes
green_starts_EW = [];
green_starts_NS = [];
for k = 2:n_log
    if strcmp(LightLog(k).EW,'green') && ~strcmp(LightLog(k-1).EW,'green')
        green_starts_EW(end+1) = LightLog(k).Time;
    end
    if strcmp(LightLog(k).NS,'green') && ~strcmp(LightLog(k-1).NS,'green')
        green_starts_NS(end+1) = LightLog(k).Time;
    end
end

all_times = [CarLog.Time];
all_ids   = [CarLog.ID];

% Combine EW and NS green starts into one list
axes_list  = [repmat({'EW'}, 1, length(green_starts_EW)), ...
    repmat({'NS'}, 1, length(green_starts_NS))];
times_list = [green_starts_EW, green_starts_NS];

for gi = 1:length(times_list)
    green_t = times_list(gi);
    axis    = axes_list{gi};

    % Snapshot: all vehicles logged at this green-start timestep
    snap = CarLog(all_times == green_t);

    for ai = 1:length(snap)
        vdir = snap(ai).Dir;

        % Skip turning vehicles — straight-going only
        if snap(ai).TurnRight || snap(ai).TurnLeft, continue; end

        % Direction must match this axis
        if strcmp(axis,'EW') && ~(strcmp(vdir,'E') || strcmp(vdir,'W')), continue; end
        if strcmp(axis,'NS') && ~(strcmp(vdir,'N') || strcmp(vdir,'S')), continue; end

        % Distance to stop line at green start
        switch vdir
            case 'E', dk = -stop_line - snap(ai).X;
            case 'W', dk =  snap(ai).X - stop_line;
            case 'N', dk = -stop_line - snap(ai).Y;
            case 'S', dk =  snap(ai).Y - stop_line;
            otherwise, continue;
        end
        if dk <= 0 || dk > 200, continue; end  % already past stop or out of range
        velk = snap(ai).V;                     % velocity at green start (may be 0)
        vid  = snap(ai).ID;                    % must be defined before gap loop

        % Gap to nearest lead vehicle (same direction, closer to stop line)
        % If no lead: gap = dk (free-flow condition)
        min_lead_dist = Inf;
        for aj = 1:length(snap)
            if snap(aj).ID == vid, continue; end
            if ~strcmp(snap(aj).Dir, vdir), continue; end
            if snap(aj).TurnRight || snap(aj).TurnLeft, continue; end
            switch vdir
                case 'E', dk_lead = -stop_line - snap(aj).X;
                case 'W', dk_lead =  snap(aj).X - stop_line;
                case 'N', dk_lead = -stop_line - snap(aj).Y;
                case 'S', dk_lead =  snap(aj).Y - stop_line;
                otherwise, continue;
            end
            % Lead must be between vehicle and stop line (smaller dist, still before stop)
            if dk_lead > 0 && dk_lead < dk && dk_lead < min_lead_dist
                min_lead_dist = dk_lead;
            end
        end
        if isinf(min_lead_dist)
            gap_k = dk;              % no lead vehicle: free-flow
        else
            gap_k = max(dk - min_lead_dist - 4.0, 0);  % 4m = vehicle length
        end

        % Find this vehicle's future log entries (after green_t)
        vid_mask = (all_ids == vid) & (all_times > green_t);
        vid_log  = CarLog(vid_mask);
        if isempty(vid_log), continue; end

        % Find first timestep where vehicle crossed the stop line
        t_cross = [];
        for si = 1:length(vid_log)
            switch vdir
                case 'E', d_si = -stop_line - vid_log(si).X;
                case 'W', d_si =  vid_log(si).X - stop_line;
                case 'N', d_si = -stop_line - vid_log(si).Y;
                case 'S', d_si =  vid_log(si).Y - stop_line;
            end
            if d_si <= 0
                t_cross = vid_log(si).Time;
                break;
            end
        end

        if isempty(t_cross), continue; end
        tauk = t_cross - green_t;
        if tauk <= 0 || tauk > 120, continue; end

        X(end+1,:) = [dk, velk, gap_k];
        y          = [y; tauk];
    end
end
end

function ok = canSpawnY(list, y0, gap)
if isempty(list), ok=true; return; end
ok = all(abs([list.Y]-y0) >= gap);
end

function ok = canSpawnX(list, x0, gap)
if isempty(list), ok=true; return; end
ok = all(abs([list.X]-x0) >= gap);
end