function plot_fuel_consumption(FuelLog, scenario_label, output_path)

if isempty(FuelLog)
    warning('plot_fuel_consumption: FuelLog is empty — nothing to plot.');
    return;
end

%% ---- Partition ----
is_rt  = logical([FuelLog.TurnedRight]);
is_str = ~is_rt;

fuel_rt  = [FuelLog(is_rt ).fuel_total];
fuel_str = [FuelLog(is_str).fuel_total];

gap_rt   = [FuelLog(is_rt ).idle_gap_time];
red_rt   = [FuelLog(is_rt ).idle_red_time];
gap_str  = [FuelLog(is_str).idle_gap_time];
red_str  = [FuelLog(is_str).idle_red_time];

%% ---- Safe mean ----
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

% Slightly reduced height (cleaner proportions)
set(fig, 'Position', [100 100 560 340]);

tiledlayout(fig,1,2,'TileSpacing','compact','Padding','compact');

font_axis  = 13;
font_label = 14;
font_title = 14;

%% ===== LEFT: Fuel =====
ax1 = nexttile;

bar_data = [mean_fuel_rt, mean_fuel_str];

b = bar(ax1, bar_data, 0.5);
b.FaceColor = 'flat';
b.CData = [0.20 0.60 0.86;
           0.93 0.49 0.19];

set(ax1,'FontSize',font_axis,'LineWidth',1.2);

ylabel(ax1,'Fuel [ml]','FontSize',font_label,'FontWeight','bold');

title(ax1,'Total Fuel per Vehicle','FontSize',font_title,'FontWeight','bold');

set(ax1,'XTickLabel',{sprintf('Right-turn\n(n=%d)',n_rt), ...
                     sprintf('Straight/Left\n(n=%d)',n_str)});

grid(ax1,'on');
box(ax1,'on');

ylim(ax1,[0 max(bar_data)*1.25]);

text(1, mean_fuel_rt*1.04, sprintf('%.1f',mean_fuel_rt), ...
    'HorizontalAlignment','center','FontSize',font_axis,'FontWeight','bold');

text(2, mean_fuel_str*1.04, sprintf('%.1f',mean_fuel_str), ...
    'HorizontalAlignment','center','FontSize',font_axis,'FontWeight','bold');

%% ===== RIGHT: Idle =====
ax2 = nexttile;

idle_mat = [mean_gap_rt,  mean_red_rt;
            mean_gap_str, mean_red_str];

b2 = bar(ax2, idle_mat, 'stacked', 'BarWidth',0.5);

b2(1).FaceColor = [0.85 0.33 0.10];
b2(2).FaceColor = [0.47 0.67 0.19];

set(ax2,'FontSize',font_axis,'LineWidth',1.2);

ylabel(ax2,'Idle time [s]','FontSize',font_label,'FontWeight','bold');

title(ax2,'Idle Time Breakdown','FontSize',font_title,'FontWeight','bold');

set(ax2,'XTickLabel',{sprintf('Right-turn\n(n=%d)',n_rt), ...
                     sprintf('Straight/Left\n(n=%d)',n_str)});

grid(ax2,'on');
box(ax2,'on');

legend(ax2,{'Gap wait','Redlight wait'}, ...
    'Location','best', ...
    'FontSize',font_axis-1, ...
    'Box','on');

%% ---- NO global title (important) ----

%% ---- Export ----
exportgraphics(fig, output_path, 'Resolution', 300);
fprintf('[plot_fuel_consumption] Saved → %s\n', output_path);

close(fig);

end