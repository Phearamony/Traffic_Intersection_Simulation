function plot_fuel_consumption(FuelLog, scenario_label, output_path)
% PLOT_FUEL_CONSUMPTION  Visualise per-vehicle fuel and idle-time statistics.
%
% Usage:
%   plot_fuel_consumption(FuelLog, scenario_label, output_path)
%
% Inputs:
%   FuelLog        – struct array, one entry per completed vehicle trip.
%                    Required fields: ID, TurnRight, TurnedRight,
%                                     fuel_total, idle_gap_time, idle_red_time
%   scenario_label – string shown in the figure title (e.g. 'MPC + RSU')
%   output_path    – full path to save the PNG (e.g. '…/output/fuel_mpc_rsu.png')
%
% Three panels are produced:
%   1. Mean total fuel consumption  [ml]  — right-turners vs straight/left cars
%   2. Mean idle time breakdown     [s]   — gap-waiting vs red-light waiting
%   3. Fuel box-plot distribution   [ml]  — per-vehicle spread for each group

if isempty(FuelLog)
    warning('plot_fuel_consumption: FuelLog is empty — nothing to plot.');
    return;
end

%% ---- Partition vehicles into right-turners and straight/left cars ----
% A "right-turner" is a car that intended to turn right (TurnRight==1).
% We include it regardless of whether it completed the turn (TurnedRight)
% because we care about the whole trip including any gap-waiting.
is_rt  = logical([FuelLog.TurnRight]);
is_str = ~is_rt;

fuel_rt  = [FuelLog(is_rt ).fuel_total];
fuel_str = [FuelLog(is_str).fuel_total];

gap_rt   = [FuelLog(is_rt ).idle_gap_time];
red_rt   = [FuelLog(is_rt ).idle_red_time];
gap_str  = [FuelLog(is_str).idle_gap_time];
red_str  = [FuelLog(is_str).idle_red_time];

%% ---- Helper: safe mean (returns 0 for empty) ----
smean = @(x) mean(x(~isnan(x) & isfinite(x)));

mean_fuel_rt  = smean(fuel_rt);
mean_fuel_str = smean(fuel_str);

mean_gap_rt   = smean(gap_rt);
mean_red_rt   = smean(red_rt);
mean_gap_str  = smean(gap_str);
mean_red_str  = smean(red_str);

n_rt  = sum(is_rt);
n_str = sum(is_str);

%% ---- Figure ----
fig = figure('Visible','off','Color','white');
set(fig, 'Position', [100 100 1100 400]);

%----- Panel 1: Mean total fuel -----
ax1 = subplot(1,3,1);
bar_data = [mean_fuel_rt, mean_fuel_str];
b = bar(bar_data, 0.5);
b.FaceColor = 'flat';
b.CData = [0.20 0.60 0.86; 0.93 0.49 0.19];   % blue / orange
set(ax1, 'XTickLabel', {sprintf('Right-turn\n(n=%d)',n_rt), ...
                         sprintf('Straight/Left\n(n=%d)',n_str)});
ylabel('Mean fuel consumed [ml]');
title('Total Fuel per Vehicle');
grid on; box off;
text(1, mean_fuel_rt  * 1.03, sprintf('%.1f ml', mean_fuel_rt),  ...
    'HorizontalAlignment','center','FontSize',9);
text(2, mean_fuel_str * 1.03, sprintf('%.1f ml', mean_fuel_str), ...
    'HorizontalAlignment','center','FontSize',9);
ylim([0, max(bar_data)*1.25 + 0.1]);

%----- Panel 2: Idle time breakdown -----
ax2 = subplot(1,3,2);
idle_mat = [mean_gap_rt,  mean_red_rt;
            mean_gap_str, mean_red_str];
b2 = bar(idle_mat, 'stacked');
b2(1).FaceColor = [0.85 0.33 0.10];   % red-orange  → gap waiting
b2(2).FaceColor = [0.47 0.67 0.19];   % green       → red-light waiting
set(ax2, 'XTickLabel', {sprintf('Right-turn\n(n=%d)',n_rt), ...
                         sprintf('Straight/Left\n(n=%d)',n_str)});
ylabel('Mean idle time [s]');
title('Idle Time Breakdown');
legend({'Gap waiting','Red-light wait'}, 'Location','northeast');
grid on; box off;

%----- Panel 3: Fuel distribution (box plot) -----
ax3 = subplot(1,3,3);
all_fuel    = [fuel_rt(:); fuel_str(:)];
group_label = [repmat({'Right-turn'}, numel(fuel_rt),  1); ...
               repmat({'Straight/Left'},numel(fuel_str), 1)];

if ~isempty(all_fuel)
    boxplot(all_fuel, group_label, 'Colors', [0.20 0.60 0.86; 0.93 0.49 0.19], ...
        'Symbol','o', 'OutlierSize', 4);
end
ylabel('Fuel consumed [ml]');
title('Fuel Distribution');
grid on; box off;

%----- Shared super-title -----
sgtitle(sprintf('Fuel Consumption & Idle Analysis — %s', scenario_label), ...
    'FontSize', 13, 'FontWeight', 'bold');

set(findall(gcf,'-property','FontSize'),'FontSize',12);

%% ---- Save ----
exportgraphics(fig, output_path, 'Resolution', 150);
fprintf('[plot_fuel_consumption] Saved → %s\n', output_path);
close(fig);
end