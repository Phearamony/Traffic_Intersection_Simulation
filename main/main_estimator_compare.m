%% Estimator Comparison: Naive Kinematic vs IDM Forward Sim vs GPR
%
% At every green-phase start, three estimators predict each vehicle's
% stop-line arrival time τ from its current (distance, velocity) state:
%
%   (A) Naive    — τ = dist / vel          (free-flow, no car-following)
%   (B) IDM      — forward IDM simulation  (reference [7] method)
%   (C) GPR      — trained Gaussian Process model (proposed C2)
%
% Actual τ is measured from CarLog after the simulation.
% Outputs: printed table + scatter plots + residual histogram.
%
% Prerequisites:
%   1. main_human_driving.m          → traffic_schedule.mat
%   2. main_human_driving_gpr_train.m → gpr_arrival_model.mat

clear classes %#ok<CLCLS>
close all; clear; clc;
clear global RSUObjs

addpath(genpath('C:/Users/monea/OneDrive/Documents/MATLAB/Traffic_Intersection/class'))
addpath(genpath('C:/Users/monea/OneDrive/Documents/MATLAB/Traffic_Intersection/function'))
addpath(genpath('C:/Users/monea/OneDrive/Documents/MATLAB/Traffic_Intersection/simplot'))

global CarN CarS CarE CarW TrafficLight
global dt KK

InitVals;

% ── Traffic light ─────────────────────────────────────────────────────────
Cycle = 120;
gNS = 54.5;  yNS = 3.5;  ar1 = 2.0;
gEW = 54.5;  yEW = 3.5;  ar2 = 2.0;

% ── Physical parameters ───────────────────────────────────────────────────
stop_line     = 10;
spawn_gap_min = 8;

% ── Load traffic schedule ─────────────────────────────────────────────────
sched_path = fullfile(fileparts(mfilename('fullpath')), '..', 'simplot', 'traffic_schedule.mat');
if ~exist(sched_path, 'file')
    error('traffic_schedule.mat not found. Run main_human_driving.m first.');
end
load(sched_path, 'sched');
KKmax = sched.KKmax;   % match whatever schedule was generated
dt    = sched.dt;
fprintf('Loaded traffic schedule (%d timesteps, dt=%.1fs)\n', KKmax, dt);

% ── Set up RSU (passive observer only — no turn optimisation) ──────────────
global RSUObjs
RSU1    = RSU(1, 0, 0);
RSUObjs = [RSU1];

gpr_model_path = fullfile(fileparts(mfilename('fullpath')), '..', 'simplot', 'gpr_arrival_model.mat');
if ~exist(gpr_model_path, 'file')
    error('gpr_arrival_model.mat not found. Run main_human_driving_gpr_train.m first.');
end
RSU1.LoadGPRModel(gpr_model_path);
fprintf('GPR model loaded into RSU.\n');

% ── Initialise cars ────────────────────────────────────────────────────────
CarN = Car.empty();  CarS = Car.empty();
CarE = Car.empty();  CarW = Car.empty();
CarIDN = 1000;  CarIDS = 2000;  CarIDE = 3000;  CarIDW = 4000;

CarN(end+1) = seedCar(CarIDN, -1.5, -150, 'N'); CarIDN = CarIDN+1;
CarS(end+1) = seedCar(CarIDS,  1.5,  150, 'S'); CarIDS = CarIDS+1;
CarE(end+1) = seedCar(CarIDE, -150,  1.5, 'E'); CarIDE = CarIDE+1;
CarW(end+1) = seedCar(CarIDW,  150, -1.5, 'W'); CarIDW = CarIDW+1;

% ── Logs ──────────────────────────────────────────────────────────────────
CarLog  = struct('Time',{},'ID',{},'Dir',{},'TurnRight',{},'X',{},'Y',{},'V',{});
CompLog = struct('GreenT',{},'VehicleID',{},'Dir',{}, ...
                 'tau_Naive',{},'tau_IDM',{},'tau_GPR',{});

prev_EW = 'red';
prev_NS = 'red';

fprintf('Running simulation (%d steps)...\n', KKmax);

%% ═══════════════════════════════════════════════════════════════════════════
%% MAIN SIMULATION LOOP
%% ═══════════════════════════════════════════════════════════════════════════
for KK = 1:KKmax
    t    = mod(KK*dt, Cycle);
    tsec = KK * dt;

    %% Traffic lights ────────────────────────────────────────────────────────
    if     t < gNS,                         TrafficLight.NS='green';  TrafficLight.EW='red';
    elseif t < gNS+yNS,                     TrafficLight.NS='yellow'; TrafficLight.EW='red';
    elseif t < gNS+yNS+ar1,                 TrafficLight.NS='red';    TrafficLight.EW='red';
    elseif t < gNS+yNS+ar1+gEW,             TrafficLight.NS='red';    TrafficLight.EW='green';
    elseif t < gNS+yNS+ar1+gEW+yEW,         TrafficLight.NS='red';    TrafficLight.EW='yellow';
    else,                                    TrafficLight.NS='red';    TrafficLight.EW='red';
    end

    %% V2X: all cars broadcast state to RSU ──────────────────────────────────
    for i = 1:length(CarN), RSU1.ReceivingV2X(CarN(i).SendingV2X(tsec)); end
    for i = 1:length(CarS), RSU1.ReceivingV2X(CarS(i).SendingV2X(tsec)); end
    for i = 1:length(CarE), RSU1.ReceivingV2X(CarE(i).SendingV2X(tsec)); end
    for i = 1:length(CarW), RSU1.ReceivingV2X(CarW(i).SendingV2X(tsec)); end

    %% Green-phase starts: run all three estimators ──────────────────────────
    EW_green_start = strcmp(TrafficLight.EW,'green') && ~strcmp(prev_EW,'green');
    NS_green_start = strcmp(TrafficLight.NS,'green') && ~strcmp(prev_NS,'green');

    if EW_green_start
        fprintf('[t=%.1f] EW GREEN START\n', tsec);
        CompLog = logEstimates(RSU1, 'E', tsec, TrafficLight, CompLog, stop_line);
        CompLog = logEstimates(RSU1, 'W', tsec, TrafficLight, CompLog, stop_line);
    end
    if NS_green_start
        fprintf('[t=%.1f] NS GREEN START\n', tsec);
        CompLog = logEstimates(RSU1, 'N', tsec, TrafficLight, CompLog, stop_line);
        CompLog = logEstimates(RSU1, 'S', tsec, TrafficLight, CompLog, stop_line);
    end

    prev_EW = TrafficLight.EW;
    prev_NS = TrafficLight.NS;

    if EW_green_start || NS_green_start
        RSU1.ResetAllShort();
    end

    %% Spawn cars ─────────────────────────────────────────────────────────────
    if sched.N_intent(KK) && canSpawnY(CarN,-150,spawn_gap_min)
        c=Car(CarIDN,-1.5,-150); CarIDN=CarIDN+1; c.Dir='N';
        c.TurnLeft=sched.N_TurnLeft(KK); c.TurnRight=sched.N_TurnRight(KK);
        c.Vd=sched.N_Vd(KK); c.Th=sched.N_Th(KK); CarN(end+1)=c;
    end
    if sched.S_intent(KK) && canSpawnY(CarS,150,spawn_gap_min)
        c=Car(CarIDS,1.5,150); CarIDS=CarIDS+1; c.Dir='S';
        c.TurnLeft=sched.S_TurnLeft(KK); c.TurnRight=sched.S_TurnRight(KK);
        c.Vd=sched.S_Vd(KK); c.Th=sched.S_Th(KK); CarS(end+1)=c;
    end
    if sched.E_intent(KK) && canSpawnX(CarE,-150,spawn_gap_min)
        c=Car(CarIDE,-150,1.5); CarIDE=CarIDE+1; c.Dir='E';
        c.TurnLeft=sched.E_TurnLeft(KK); c.TurnRight=sched.E_TurnRight(KK);
        c.Vd=sched.E_Vd(KK); c.Th=sched.E_Th(KK); CarE(end+1)=c;
    end
    if sched.W_intent(KK) && canSpawnX(CarW,150,spawn_gap_min)
        c=Car(CarIDW,150,-1.5); CarIDW=CarIDW+1; c.Dir='W';
        c.TurnLeft=sched.W_TurnLeft(KK); c.TurnRight=sched.W_TurnRight(KK);
        c.Vd=sched.W_Vd(KK); c.Th=sched.W_Th(KK); CarW(end+1)=c;
    end

    %% Acceleration (IDM + red-light dummy) ──────────────────────────────────
    isRedNS = strcmp(TrafficLight.NS,'red') || strcmp(TrafficLight.NS,'yellow');
    isRedEW = strcmp(TrafficLight.EW,'red') || strcmp(TrafficLight.EW,'yellow');

    used_dN=false; used_dS=false; used_dE=false; used_dW=false;
    for i=1:length(CarN)
        if isRedNS && (CarN(i).Y < -stop_line+CarN(i).R0) && ~used_dN
            dm=Car(-1,0,0); dm.Y=-stop_line+CarN(i).R0; dm.V=0; dm.Dir='N';
            CarN(i).Ac=CarN(i).IDM(CarN(i),dm); used_dN=true;
        elseif i>1, CarN(i).Ac=CarN(i).IDM(CarN(i),CarN(i-1));
        else, CarN(i).Ac=0.5*(CarN(i).Vd-CarN(i).V); end
    end
    for i=1:length(CarS)
        if isRedNS && (CarS(i).Y > stop_line-CarS(i).R0) && ~used_dS
            dm=Car(-1,0,0); dm.Y=stop_line-CarS(i).R0; dm.V=0; dm.Dir='S';
            CarS(i).Ac=CarS(i).IDM(CarS(i),dm); used_dS=true;
        elseif i>1, CarS(i).Ac=CarS(i).IDM(CarS(i),CarS(i-1));
        else, CarS(i).Ac=0.5*(CarS(i).Vd-CarS(i).V); end
    end
    for i=1:length(CarE)
        if isRedEW && (CarE(i).X < -stop_line+CarE(i).R0) && ~used_dE
            dm=Car(-1,0,0); dm.X=-stop_line+CarE(i).R0; dm.V=0; dm.Dir='E';
            CarE(i).Ac=CarE(i).IDM(CarE(i),dm); used_dE=true;
        elseif i>1, CarE(i).Ac=CarE(i).IDM(CarE(i),CarE(i-1));
        else, CarE(i).Ac=0.5*(CarE(i).Vd-CarE(i).V); end
    end
    for i=1:length(CarW)
        if isRedEW && (CarW(i).X > stop_line-CarW(i).R0) && ~used_dW
            dm=Car(-1,0,0); dm.X=stop_line-CarW(i).R0; dm.V=0; dm.Dir='W';
            CarW(i).Ac=CarW(i).IDM(CarW(i),dm); used_dW=true;
        elseif i>1, CarW(i).Ac=CarW(i).IDM(CarW(i),CarW(i-1));
        else, CarW(i).Ac=0.5*(CarW(i).Vd-CarW(i).V); end
    end

    %% Position / velocity update (second-order Euler) ───────────────────────
    for i=1:length(CarN)
        v=CarN(i).V; CarN(i).V=max(v+CarN(i).Ac*dt,0);
        CarN(i).Y=CarN(i).Y+(v*dt+0.5*CarN(i).Ac*dt^2);
    end
    for i=1:length(CarS)
        v=CarS(i).V; CarS(i).V=max(v+CarS(i).Ac*dt,0);
        CarS(i).Y=CarS(i).Y-(v*dt+0.5*CarS(i).Ac*dt^2);
    end
    for i=1:length(CarE)
        v=CarE(i).V; CarE(i).V=max(v+CarE(i).Ac*dt,0);
        CarE(i).X=CarE(i).X+(v*dt+0.5*CarE(i).Ac*dt^2);
    end
    for i=1:length(CarW)
        v=CarW(i).V; CarW(i).V=max(v+CarW(i).Ac*dt,0);
        CarW(i).X=CarW(i).X-(v*dt+0.5*CarW(i).Ac*dt^2);
    end

    %% Log all car positions ──────────────────────────────────────────────────
    for i=1:length(CarN)
        CarLog(end+1)=struct('Time',tsec,'ID',CarN(i).ID,'Dir',CarN(i).Dir, ...
            'TurnRight',CarN(i).TurnRight,'X',CarN(i).X,'Y',CarN(i).Y,'V',CarN(i).V);
    end
    for i=1:length(CarS)
        CarLog(end+1)=struct('Time',tsec,'ID',CarS(i).ID,'Dir',CarS(i).Dir, ...
            'TurnRight',CarS(i).TurnRight,'X',CarS(i).X,'Y',CarS(i).Y,'V',CarS(i).V);
    end
    for i=1:length(CarE)
        CarLog(end+1)=struct('Time',tsec,'ID',CarE(i).ID,'Dir',CarE(i).Dir, ...
            'TurnRight',CarE(i).TurnRight,'X',CarE(i).X,'Y',CarE(i).Y,'V',CarE(i).V);
    end
    for i=1:length(CarW)
        CarLog(end+1)=struct('Time',tsec,'ID',CarW(i).ID,'Dir',CarW(i).Dir, ...
            'TurnRight',CarW(i).TurnRight,'X',CarW(i).X,'Y',CarW(i).Y,'V',CarW(i).V);
    end

    %% Remove cars that left the grid ─────────────────────────────────────────
    exitN=(abs([CarN.X])>170)|(abs([CarN.Y])>170);
    exitS=(abs([CarS.X])>170)|(abs([CarS.Y])>170);
    exitE=(abs([CarE.X])>170)|(abs([CarE.Y])>170);
    exitW=(abs([CarW.X])>170)|(abs([CarW.Y])>170);
    for idx=find(exitN), RSU1.RemoveVehicle(CarN(idx).ID,'N'); end
    for idx=find(exitS), RSU1.RemoveVehicle(CarS(idx).ID,'S'); end
    for idx=find(exitE), RSU1.RemoveVehicle(CarE(idx).ID,'E'); end
    for idx=find(exitW), RSU1.RemoveVehicle(CarW(idx).ID,'W'); end
    CarN=CarN(~exitN); CarS=CarS(~exitS);
    CarE=CarE(~exitE); CarW=CarW(~exitW);
end

fprintf('\nSimulation complete. %d estimation records logged.\n', length(CompLog));

%% ═══════════════════════════════════════════════════════════════════════════
%% POST-SIMULATION: match predicted τ against actual crossing time
%%
%% Each estimator is evaluated INDEPENDENTLY — a sample is only excluded
%% from estimator X if X itself returned Inf, not because another estimator
%% failed. This prevents easy-case bias from joint-drop approaches.
%% ═══════════════════════════════════════════════════════════════════════════
all_times = [CarLog.Time];
all_ids   = [CarLog.ID];
all_X     = [CarLog.X];
all_Y     = [CarLog.Y];

% Per-estimator arrays (each may have different n)
tau_N_pred   = [];  tau_N_act   = [];
tau_IDM_pred = [];  tau_IDM_act = [];
tau_GPR_pred = [];  tau_GPR_act = [];
n_no_crossing = 0;

for k = 1:length(CompLog)
    vid     = CompLog(k).VehicleID;
    green_t = CompLog(k).GreenT;
    vdir    = CompLog(k).Dir;

    % Find actual crossing time (shared step — independent of estimators)
    mask = (all_ids == vid) & (all_times > green_t);
    if ~any(mask), n_no_crossing=n_no_crossing+1; continue; end

    fut_t = all_times(mask);
    fut_X = all_X(mask);
    fut_Y = all_Y(mask);

    t_cross = NaN;
    for si = 1:length(fut_t)
        switch vdir
            case 'E', crossed = fut_X(si) >= -stop_line;
            case 'W', crossed = fut_X(si) <=  stop_line;
            case 'N', crossed = fut_Y(si) >= -stop_line;
            case 'S', crossed = fut_Y(si) <=  stop_line;
            otherwise, crossed = false;
        end
        if crossed, t_cross = fut_t(si); break; end
    end

    if isnan(t_cross), n_no_crossing=n_no_crossing+1; continue; end
    tau_actual = t_cross - green_t;
    if tau_actual <= 0 || tau_actual > 120, n_no_crossing=n_no_crossing+1; continue; end

    % Add to each estimator independently — only skip if that estimator is Inf
    if isfinite(CompLog(k).tau_Naive)
        tau_N_pred(end+1,1) = CompLog(k).tau_Naive;
        tau_N_act(end+1,1)  = tau_actual;
    end
    if isfinite(CompLog(k).tau_IDM)
        tau_IDM_pred(end+1,1) = CompLog(k).tau_IDM;
        tau_IDM_act(end+1,1)  = tau_actual;
    end
    if isfinite(CompLog(k).tau_GPR)
        tau_GPR_pred(end+1,1) = CompLog(k).tau_GPR;
        tau_GPR_act(end+1,1)  = tau_actual;
    end
end

n_total = length(CompLog);
fprintf('Records: total=%d  no-crossing=%d\n', n_total, n_no_crossing);
fprintf('Valid samples:  Naive=%d  IDM=%d  GPR=%d\n', ...
    length(tau_N_pred), length(tau_IDM_pred), length(tau_GPR_pred));

if length(tau_GPR_pred) < 10
    warning('Too few GPR samples (%d).', length(tau_GPR_pred)); return;
end

%% ── Per-estimator residuals and metrics ─────────────────────────────────────
res_N   = tau_N_pred   - tau_N_act;
res_IDM = tau_IDM_pred - tau_IDM_act;
res_GPR = tau_GPR_pred - tau_GPR_act;

rmse_fn   = @(r) sqrt(mean(r.^2));
mae_fn    = @(r) mean(abs(r));
bias_fn   = @(r) mean(r);
within_fn = @(r,thr) 100*mean(abs(r)<=thr);
inf_pct   = @(pred) 100*(sum(isinf(pred)) / n_total);

inf_N   = inf_pct([CompLog.tau_Naive]);
inf_IDM = inf_pct([CompLog.tau_IDM]);
inf_GPR = inf_pct([CompLog.tau_GPR]);

rmse = [rmse_fn(res_N),  rmse_fn(res_IDM),  rmse_fn(res_GPR)];
mae  = [mae_fn(res_N),   mae_fn(res_IDM),   mae_fn(res_GPR)];
bias = [bias_fn(res_N),  bias_fn(res_IDM),  bias_fn(res_GPR)];
w2   = [within_fn(res_N,2), within_fn(res_IDM,2), within_fn(res_GPR,2)];
w5   = [within_fn(res_N,5), within_fn(res_IDM,5), within_fn(res_GPR,5)];
ns   = [length(tau_N_pred), length(tau_IDM_pred), length(tau_GPR_pred)];
infs = [inf_N, inf_IDM, inf_GPR];

%% ── Print table ─────────────────────────────────────────────────────────────
names = {'Naive (dist/vel)', 'IDM Forward Sim', 'GPR (proposed)'};
SEP   = repmat('-', 1, 82);
fprintf('\n%s\n', SEP);
fprintf('  Arrival-Time Estimator Comparison\n');
fprintf('%s\n', SEP);
fprintf('  %-20s  %6s  %8s  %8s  %8s  %9s  %9s  %7s\n', ...
        'Estimator','n','RMSE(s)','MAE(s)','Bias(s)','|e|<=2s(%)','|e|<=5s(%)','Inf(%)');
fprintf('%s\n', SEP);
markers = {'', '', '  <- proposed'};
for j = 1:3
    fprintf('  %-20s  %6d  %8.3f  %8.3f  %+8.3f  %9.1f  %9.1f  %6.1f%%%s\n', ...
            names{j}, ns(j), rmse(j), mae(j), bias(j), w2(j), w5(j), infs(j), markers{j});
end
fprintf('%s\n\n', SEP);

%% ── Save ────────────────────────────────────────────────────────────────────
out_dir = 'C:/Users/monea/OneDrive/Documents/MATLAB/Traffic_Intersection/output';
save(fullfile(out_dir,'estimator_comparison_results.mat'), ...
     'tau_N_pred','tau_N_act','tau_IDM_pred','tau_IDM_act', ...
     'tau_GPR_pred','tau_GPR_act', ...
     'rmse','mae','bias','w2','w5','ns','infs','n_total','n_no_crossing');

%% ═══════════════════════════════════════════════════════════════════════════
%% FIGURE 1 — Predicted vs Actual scatter (1 × 3 panels)
%% Each panel uses its own per-estimator actual array (different n allowed)
%% ═══════════════════════════════════════════════════════════════════════════
COLS = {[0.50 0.50 0.50], [0.20 0.60 0.86], [0.93 0.42 0.13]};
LBLS = {'Naive (dist/vel)', 'IDM Forward Sim', 'GPR (proposed)'};
taus_pred = {tau_N_pred,   tau_IDM_pred,   tau_GPR_pred};
taus_act  = {tau_N_act,    tau_IDM_act,    tau_GPR_act};
res_list  = {res_N, res_IDM, res_GPR};

all_vals = [tau_N_pred; tau_N_act; tau_IDM_pred; tau_IDM_act; tau_GPR_pred; tau_GPR_act];
lims = [0, max(all_vals)*1.05];

fig1 = figure('Visible','off','Color','white','Position',[50 50 1400 440]);
for j = 1:3
    subplot(1,3,j);
    scatter(taus_act{j}, taus_pred{j}, 20, COLS{j}, 'filled', 'MarkerFaceAlpha',0.5);
    hold on; plot(lims,lims,'k--','LineWidth',1.4); hold off;
    xlabel('Actual \tau [s]',    'FontSize',11);
    ylabel('Predicted \tau [s]', 'FontSize',11);
    title(sprintf('%s  (n=%d)\nRMSE=%.2fs   MAE=%.2fs   Bias=%+.2fs', ...
          LBLS{j}, ns(j), rmse(j), mae(j), bias(j)), 'FontSize',10);
    axis equal; xlim(lims); ylim(lims); grid on; box off;
end
sgtitle('Predicted vs Actual Arrival Time', 'FontSize',13,'FontWeight','bold');
exportgraphics(fig1, fullfile(out_dir,'estimator_comparison_scatter.png'), 'Resolution',150);
fprintf('Scatter plot saved  -> output/estimator_comparison_scatter.png\n');
close(fig1);

%% ═══════════════════════════════════════════════════════════════════════════
%% FIGURE 2 — Residual histogram + box plot
%% ═══════════════════════════════════════════════════════════════════════════
fig2 = figure('Visible','off','Color','white','Position',[50 50 1100 450]);
all_res = [res_N; res_IDM; res_GPR];
edges   = linspace(min(all_res)-1, max(all_res)+1, 50);

subplot(1,2,1);
for j = 1:3
    histogram(res_list{j}, edges, 'Normalization','probability', ...
        'FaceColor',COLS{j},'EdgeColor','none','FaceAlpha',0.6, ...
        'DisplayName',sprintf('%s  (n=%d)  RMSE=%.2fs',LBLS{j},ns(j),rmse(j)));
    hold on;
end
xline(0,'k--','LineWidth',1.4,'HandleVisibility','off');
hold off;
xlabel('Residual \tau_{pred} - \tau_{actual} [s]','FontSize',11);
ylabel('Probability','FontSize',11);
title('Residual Distributions','FontSize',12,'FontWeight','bold');
legend('Location','northeast','FontSize',9);
grid on; box off;

subplot(1,2,2);
% Box plot: pad shorter vectors with NaN so they all reach max length
max_n = max(ns);
res_mat = NaN(max_n, 3);
res_mat(1:ns(1),1) = res_N;
res_mat(1:ns(2),2) = res_IDM;
res_mat(1:ns(3),3) = res_GPR;
bp = boxplot(res_mat,'Labels',{'Naive','IDM','GPR'}, ...
             'Colors',cell2mat(COLS'),'Symbol','.'); 
set(findobj(gcf,'type','line'),'LineWidth',1.5);
yline(0,'k--','LineWidth',1.4);
ylabel('Residual [s]','FontSize',11);
title('Error Distribution (Box Plot)','FontSize',12,'FontWeight','bold');
grid on; box off;
y_top = max(abs(all_res))*1.15;
for j = 1:3
    text(j, y_top, sprintf('RMSE\n%.2fs',rmse(j)), ...
        'HorizontalAlignment','center','FontSize',9,'Color',COLS{j});
end

sgtitle(sprintf('Residual Analysis  |  Naive n=%d  IDM n=%d  GPR n=%d', ns(1),ns(2),ns(3)), ...
        'FontSize',13,'FontWeight','bold');
exportgraphics(fig2, fullfile(out_dir,'estimator_comparison_residuals.png'), 'Resolution',150);
fprintf('Residual plot saved → output/estimator_comparison_residuals.png\n');
fprintf('Raw data saved      → output/estimator_comparison_results.mat\n\n');
close(fig2);


%% ═══════════════════════════════════════════════════════════════════════════
%% LOCAL HELPER FUNCTIONS
%% ═══════════════════════════════════════════════════════════════════════════

function CompLog = logEstimates(RSU1, dir, tsec, trafficLight, CompLog, stop_line)
% Runs all three estimators for one direction at one green-phase start.
% Appends one row per straight-going vehicle to CompLog.

    [ids_IDM, taus_IDM] = RSU1.EstimateAllArrivalTimesIDM(dir, trafficLight);
    [ids_GPR, taus_GPR] = RSU1.EstimateAllArrivalTimesGPR(dir, trafficLight);

    all_ids = union(ids_IDM, ids_GPR);
    if isempty(all_ids), return; end

    for k = 1:length(all_ids)
        vid   = all_ids(k);
        vdata = RSU1.GetVehicleData(vid);
        if isempty(vdata), continue; end
        if vdata.TurnRight || vdata.TurnLeft, continue; end

        % ── Naive: τ = dist / vel (floor vel at 0.5 m/s to avoid div-by-zero)
        switch dir
            case 'E', dist = -stop_line - vdata.X;
            case 'W', dist =  vdata.X   - stop_line;
            case 'N', dist = -stop_line - vdata.Y;
            case 'S', dist =  vdata.Y   - stop_line;
            otherwise, dist = Inf;
        end
        if dist <= 0
            tau_Naive = 0;
        else
            tau_Naive = dist / max(vdata.V, 0.5);
        end

        % ── IDM / GPR lookup ──────────────────────────────────────────────
        idx_IDM = find(ids_IDM == vid, 1);
        idx_GPR = find(ids_GPR == vid, 1);
        tau_IDM = Inf; if ~isempty(idx_IDM), tau_IDM = taus_IDM(idx_IDM); end
        tau_GPR = Inf; if ~isempty(idx_GPR), tau_GPR = taus_GPR(idx_GPR); end

        CompLog(end+1) = struct('GreenT',tsec,'VehicleID',vid,'Dir',dir, ...  %#ok<AGROW>
            'tau_Naive',tau_Naive,'tau_IDM',tau_IDM,'tau_GPR',tau_GPR);
    end
end

function c = seedCar(id, x, y, dir)
    c = Car(id, x, y); c.Dir = dir;
end

function ok = canSpawnY(list, y0, gap)
    if isempty(list), ok=true; return; end
    ok = all(abs([list.Y]-y0) >= gap);
end

function ok = canSpawnX(list, x0, gap)
    if isempty(list), ok=true; return; end
    ok = all(abs([list.X]-x0) >= gap);
end