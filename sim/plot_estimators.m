function plot_estimators(E, outdir)
%PLOT_ESTIMATORS  Estimator comparison figures of one evaluation run.
%   estimator_comparison_scatter.png    predicted vs actual time, per estimator
%   estimator_comparison_residuals.png  error distributions and calibration
%   E rows: [t id y g idm phys gpr sd_gpr pigpr sd_pigpr pigpr_on sd_on]
y = E(:, 3);
names = {'IDM forward sim.', 'Queue physics', 'GPR', 'PI-GPR', 'PI-GPR online'};
cols = [5 6 7 9 11];
f = figure('Visible', 'off', 'Color', 'w', 'Position', [60 60 1500 330]);
lim = [0, max(y) * 1.05];
for k = 1:5
    subplot(1, 5, k);
    p = E(:, cols(k)); m = isfinite(p);
    plot(y(m), p(m), '.', 'MarkerSize', 4); hold on;
    plot(lim, lim, 'r--', 'LineWidth', 1.2);
    axis([lim lim]); grid on; box on;
    xlabel('Actual \tau [s]'); if k == 1, ylabel('Predicted \tau [s]'); end
    title(sprintf('%s\nRMSE %.2f s', names{k}, sqrt(mean((p(m) - y(m)).^2))));
end
print(f, fullfile(outdir, 'estimator_comparison_scatter.png'), '-dpng', '-r200');
close(f);

f = figure('Visible', 'off', 'Color', 'w', 'Position', [60 60 1100 380]);
subplot(1, 2, 1); hold on;
edges = -20:1:20;
for k = [1 3 4 5]
    e = E(:, cols(k)) - y; e = e(isfinite(e));
    c = histc(max(min(e, 20), -20), edges); c = c / max(sum(c), 1);
    stairs(edges, c, 'LineWidth', 1.4);
end
legend(names([1 3 4 5]), 'Location', 'northwest');
xlabel('Prediction error [s]'); ylabel('Share'); grid on; box on;
subplot(1, 2, 2); hold on;
lv = 0.1:0.1:0.9; zq = sqrt(2) * erfinv(lv);
for k = [3 4 5]
    e = abs(E(:, cols(k)) - y); s = E(:, cols(k) + 1);
    m = isfinite(e) & isfinite(s);
    plot(lv, arrayfun(@(z) mean(e(m) <= z * s(m)), zq), 'o-', 'LineWidth', 1.3);
end
plot([0 1], [0 1], 'k--');
legend([names(3:5), {'ideal'}], 'Location', 'northwest');
xlabel('Nominal interval level'); ylabel('Observed coverage'); grid on; box on;
print(f, fullfile(outdir, 'estimator_comparison_residuals.png'), '-dpng', '-r200');
close(f);
end
