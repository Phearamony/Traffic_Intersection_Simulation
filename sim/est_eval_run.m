function rows = est_eval_run(S)
%EST_EVAL_RUN  Estimator comparison on one seed, appended to S.estcsv.
%
%   Human traffic (C1) for the settings in S. A roadside sensor sees every
%   vehicle; every 2 s each estimator predicts the time until every vehicle
%   before its stop line reaches it. When the vehicle crosses, the error is
%   recorded. Scored: snapshots during the green (or yellow) of the
%   vehicle's approach, for arrivals within that green.
%   Estimators: idm (IDM forward simulation), phys (queue-discharge
%   physics), gpr (plain GPR), pigpr (physics-informed GPR, offline),
%   pigpr_on (the same, learning online from every crossing in this run).
%   Sets: 'own' = all samples with a finite prediction of that estimator;
%         'common' = samples where every estimator has one.
%   Columns: study,point,seed,kkmax,estimator,set,n,rmse_s,mae_s,bias_s,
%            within2_pct,within5_pct,cov90_pct,nll,fail_pct
R = sim_run('C1', S, struct('est_eval', true));
E = R.est.evalrows;           % [t id y g idm phys gpr sd pigpr sd pigpr_on sd]
% scope: snapshots during green / yellow of the vehicle's approach, for
% arrivals within that green (what the RSU plans with; same as training)
E = E(E(:, 4) > 0 & E(:, 3) <= E(:, 4) + 1e-6, :);
names = {'idm', 'phys', 'gpr', 'pigpr', 'pigpr_on'};
mu = {E(:, 5), E(:, 6), E(:, 7), E(:, 9), E(:, 11)};
sd = {[], [], E(:, 8), E(:, 10), E(:, 12)};
y = E(:, 3);
common = all(isfinite(E(:, [5 6 7 9 11])), 2);
header = 'study,point,seed,kkmax,estimator,set,n,rmse_s,mae_s,bias_s,within2_pct,within5_pct,cov90_pct,nll,fail_pct';
p = S.estcsv;
d = fileparts(p); if ~isempty(d) && ~exist(d, 'dir'), mkdir(d); end
if exist(p, 'file')
    fid = fopen(p, 'r'); first = fgetl(fid); fclose(fid);
    if ~ischar(first) || ~strcmp(strtrim(first), header)
        [dd, nn, ee] = fileparts(p);
        movefile(p, fullfile(dd, [nn '_oldformat' ee]));
    end
end
newf = ~exist(p, 'file');
fid = fopen(p, 'a');
if newf, fprintf(fid, '%s\n', header); end
rows = {};
for k = 1:numel(names)
    for set = {'own', 'common'}
        if strcmp(set{1}, 'common'), m = common; else, m = true(size(y)); end
        s = sd{k}; if ~isempty(s), s = s(m); end
        r = est_metrics(y(m), mu{k}(m), s);
        num = sprintf('%d,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.4f,%.2f', r.n, r.rmse, r.mae, r.bias, ...
            r.within2, r.within5, r.cov90, r.nll, r.fail);
        fprintf(fid, '%s\n', strjoin({S.study, S.point, sprintf('%g', S.seed), ...
            sprintf('%d', S.KKmax), names{k}, set{1}, num}, ','));   % no empty fprintf arguments
        rows{end+1} = r; %#ok<AGROW>
    end
end
fclose(fid);
if S.plot
    try
        plot_estimators(E, S.outdir);
    catch err
        fprintf(2, '[est_eval_run] figure failed: %s\n', err.message);
    end
end
if S.plot
    save(fullfile(S.outdir, 'estimator_comparison_results.mat'), 'E', '-v7');
end
end
