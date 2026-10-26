function T = read_runs(csv_path)
%READ_RUNS  Read a results CSV with study, point and label as text.
%   Keeps the last row for each (study, point, seed, label), in case a run
%   was repeated.
opts = detectImportOptions(csv_path, 'Delimiter', ',');
txt = intersect({'study', 'point', 'label', 'estimator', 'set'}, opts.VariableNames);
opts = setvartype(opts, txt, 'string');
T = readtable(csv_path, opts);
if isempty(T), return; end
for v = txt
    x = T.(v{1}); x(ismissing(x)) = ""; T.(v{1}) = x;
end
k = strcat(T.study, '|', T.point, '|', string(T.seed));
if ismember('label', T.Properties.VariableNames), k = strcat(k, '|', T.label); end
if ismember('estimator', T.Properties.VariableNames), k = strcat(k, '|', T.estimator, '|', T.set); end
[~, ia] = unique(k, 'last');
T = T(sort(ia), :);
end
