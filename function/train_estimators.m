function train_estimators(seeds, n_test, kkmax)
%TRAIN_ESTIMATORS  Collect data and fit the arrival-time estimators.
%
%   train_estimators                 % seeds 1-20 (16 train, 4 test), 720 steps
%   train_estimators(1:20, 4, 720)
%
%   1. Runs C1 (human drivers, nominal population) for every seed. A
%      roadside sensor records, every 2 s and in every signal phase, the
%      features of every vehicle before its stop line; when the vehicle
%      crosses, the true remaining time y becomes a sample. The IDM
%      forward simulation is evaluated on the same snapshots.
%      Scope: snapshots during the green (or yellow) of the vehicle's
%      approach, for arrivals within that green (what the RSU plans with).
%   2. Queue physics: theta = [t_L, h_s, a_p] of est_phys by least squares
%      on a grid (training seeds only).
%   3. GPR (plain): sparse GP on [d v gap g] -> y.
%      PI-GPR: prior = IDM forward simulation (queue physics where it has no
%      answer); sparse GP on [d v gap g n vl nR prior] -> y - prior.
%   4. Test on the held-out seeds: RMSE, MAE, bias, % within 2 s / 5 s,
%      90 % interval coverage, negative log predictive density.
%
%   Writes to output/1_training/:
%     estimators.mat             the trained models (theta, gpr, pigpr), used by the RSU
%     gpr_holdout_metrics.csv    accuracy on the training and held-out seeds
%     gpr_model_diagnostics.png  held-out predictions, errors, calibration
%     estimator_samples.mat      the collected samples

if nargin < 1 || isempty(seeds), seeds = 1:20; end
if nargin < 2 || isempty(n_test), n_test = 4; end
if nargin < 3 || isempty(kkmax), kkmax = 720; end
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'sim')); addpath(fullfile(root, 'function'));
out = out_folder('training');

old = getenv('ABL_SEED'); oldk = getenv('ABL_KKMAX');
abl_env_reset();                                    % default settings for the data runs
cln = onCleanup(@() restore_env(old, oldk));        %#ok<NASGU> also after Ctrl+C
X = zeros(0, 15);
for s = seeds
    setenv('ABL_SEED', num2str(s)); setenv('ABL_KKMAX', num2str(kkmax));
    S = abl_settings(); S.driver = 'nominal'; S.sched = 'staged';
    fprintf('[train_estimators] seed %d ...\n', s);
    R = sim_run('C1', S, struct('collect', true));
    smp = R.est.samples;
    X = [X; [s * ones(size(smp, 1), 1), smp]]; %#ok<AGROW>
end
setenv('ABL_SEED', old); setenv('ABL_KKMAX', oldk);
% columns: 1 seed 2 t 3 id 4 app 5:11 [d v gap g n vl nR] 12 prior 13 tphys 14 y 15 idm
save(fullfile(out, 'estimator_samples.mat'), 'X', '-v7');
% scope: snapshots in green / yellow, arrivals within that green
X = X(X(:, 8) > 0 & X(:, 14) <= X(:, 8) + 1e-6, :);
isTest = X(:, 1) > seeds(end - n_test);
y = X(:, 14);
C = geom_constants();

% ---- physics parameters ----
apps = 'EW';
tg0 = zeros(size(y)); tend = tg0;
for r = 1:numel(y)
    [~, tg0(r), tend(r)] = signal_info(X(r, 2), apps(X(r, 4)), C);
end
best = inf; theta = [2, 2, 1.5];
for tL = 0:0.25:5
    for hs = 1.0:0.1:4.0
        for ap = [0.6 0.8 1.0 1.2 1.5]
            tp = est_phys(X(:, 5), X(:, 6), X(:, 9), X(:, 2), tg0, tend, [tL, hs, ap], C)';
            e = tp(~isTest) - y(~isTest);
            J = mean(e.^2);
            if J < best, best = J; theta = [tL, hs, ap]; end
        end
    end
end
fprintf('[train_estimators] physics: t_L = %.2f s, h_s = %.2f s, a_p = %.2f m/s2\n', theta);
tphys = est_phys(X(:, 5), X(:, 6), X(:, 9), X(:, 2), tg0, tend, theta, C)';
prior = X(:, 15); bad = ~isfinite(prior); prior(bad) = tphys(bad);
Z8 = [X(:, 5:11), prior];
Z4 = X(:, 5:8);

% ---- fit ----
tr = ~isTest;
fprintf('[train_estimators] %d training / %d test samples\n', sum(tr), sum(isTest));
o = struct('m', 400, 'n_opt', 1000, 'iter', 200, 'seed', 1);
gpr = gp_fit(Z4(tr, :), y(tr), o);
pigpr = gp_fit(Z8(tr, :), y(tr) - prior(tr), o);

% ---- calibrate the uncertainty (noise that grows with the time to arrival) ----
[mg, vg] = gp_predict(gpr, Z4);
[mp, vp] = gp_predict(pigpr, Z8); mp = mp + prior;
gpr = gp_calibrate(gpr, vg(tr), mg(tr), y(tr) - mg(tr));
pigpr = gp_calibrate(pigpr, vp(tr), mp(tr), y(tr) - mp(tr));
fprintf('[train_estimators] noise sd = %.2f + %.3f tau (GPR), %.2f + %.3f tau (PI-GPR)\n', gpr.cal, pigpr.cal);

% ---- evaluate ----
sg = gp_sd(gpr, vg, mg)'; sp = gp_sd(pigpr, vp, mp)';
names = {'idm', 'phys', 'gpr', 'pigpr'};
preds = {X(:, 15), tphys, mg, mp};
sds = {[], [], sg, sp};
fid = fopen(fullfile(out, 'gpr_holdout_metrics.csv'), 'w');
fprintf(fid, 'set,estimator,n,rmse_s,mae_s,bias_s,within2s_pct,within5s_pct,cov90_pct,nll,fail_pct\n');
sets = {'train', tr; 'test', isTest};
for k = 1:2
    m = sets{k, 2};
    for e = 1:numel(names)
        sd = sds{e}; if ~isempty(sd), sd = sd(m); end
        r = est_metrics(y(m), preds{e}(m), sd);
        fprintf(fid, '%s,%s,%d,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%.3f,%.2f\n', sets{k, 1}, names{e}, ...
            r.n, r.rmse, r.mae, r.bias, r.within2, r.within5, r.cov90, r.nll, r.fail);
        if k == 2
            fprintf('  test %-6s RMSE %.2f  MAE %.2f  within2 %.1f%%  cov90 %.1f%%\n', ...
                names{e}, r.rmse, r.mae, r.within2, r.cov90);
        end
    end
end
fclose(fid);

info = struct('seeds', seeds, 'n_test', n_test, 'kkmax', kkmax, 'n_train', sum(tr), ...
              'n_testsamples', sum(isTest), 'created', datestr(now));
save(fullfile(out, 'estimators.mat'), 'theta', 'gpr', 'pigpr', 'info', '-v7');
fprintf('[train_estimators] saved output/1_training/estimators.mat\n');

% ---- diagnostic figure ----
try
    f = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1300 400]);
    yt = y(isTest);
    subplot(1, 3, 1);
    scatter(yt, mp(isTest), 8, sp(isTest), 'filled'); hold on;
    lim = [0, max(yt) * 1.05]; plot(lim, lim, 'r--', 'LineWidth', 1.2);
    cb = colorbar; ylabel(cb, '\sigma [s]');
    xlabel('Actual \tau [s]'); ylabel('PI-GPR \tau [s]'); axis([lim lim]); grid on; box on;
    title('PI-GPR, held-out seeds');
    subplot(1, 3, 2);
    edges = -20:1:20;
    hold on;
    histogram(X(isTest, 15) - yt, edges, 'Normalization', 'probability', 'DisplayStyle', 'stairs', 'LineWidth', 1.4);
    histogram(mg(isTest) - yt, edges, 'Normalization', 'probability', 'DisplayStyle', 'stairs', 'LineWidth', 1.4);
    histogram(mp(isTest) - yt, edges, 'Normalization', 'probability', 'DisplayStyle', 'stairs', 'LineWidth', 1.4);
    legend({'IDM sim.', 'GPR', 'PI-GPR'}, 'Location', 'northwest');
    xlabel('Prediction error [s]'); ylabel('Share'); grid on; box on;
    title('Error distribution, held-out seeds');
    subplot(1, 3, 3);
    lv = 0.1:0.1:0.9; zq = sqrt(2) * erfinv(lv);
    cg = arrayfun(@(z) mean(abs(mg(isTest) - yt) <= z * sg(isTest)), zq);
    cp = arrayfun(@(z) mean(abs(mp(isTest) - yt) <= z * sp(isTest)), zq);
    plot(lv, cg, 'o-', lv, cp, 's-', [0 1], [0 1], 'k--', 'LineWidth', 1.2);
    legend({'GPR', 'PI-GPR', 'ideal'}, 'Location', 'northwest');
    xlabel('Nominal interval level'); ylabel('Observed coverage'); grid on; box on;
    title('Calibration');
    print(f, fullfile(out, 'gpr_model_diagnostics.png'), '-dpng', '-r200');
    close(f);
catch err
    fprintf(2, '[train_estimators] figure failed: %s\n', err.message);
end
end

function restore_env(old, oldk)
setenv('ABL_SEED', old); setenv('ABL_KKMAX', oldk);
end
