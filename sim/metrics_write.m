function metrics_write(M, label, S)
%METRICS_WRITE  Append one row to the results CSV named by S.csv.
%   If the file has a different header (older version), it is renamed
%   *_oldformat.csv and a new file is started.
cols = {'n_rt_intent', 'n_rt_done', 'rt_completion', 'n_exited', 'n_unfinished', 'n_waiting', ...
        'fuel_rt_mL', 'fuel_str_mL', 'fuel_L100km', 'gap_rt_s', 'red_rt_s', 'red_str_s', ...
        'pet_min_s', 'pet_mean_s', 'n_conflict', 'n_collision', ...
        'delay_rt_s', 'delay_str_s', 'delay_all_s', 'n_blocked', 'entry_wait_s', ...
        'jerk_rms', 'stops_per_veh', 'qp_solves', 'qp_fail_pct', 'guard'};
header = ['study,point,seed,label,', strjoin(cols, ',')];
p = S.csv;
d = fileparts(p);
if ~isempty(d) && ~exist(d, 'dir'), mkdir(d); end
if exist(p, 'file')
    fid = fopen(p, 'r'); first = fgetl(fid); fclose(fid);
    if ~ischar(first) || ~strcmp(strtrim(first), header)
        [dd, nn, ee] = fileparts(p);
        movefile(p, fullfile(dd, [nn '_oldformat' ee]));
        fprintf(2, '[metrics] %s had an older header; renamed to %s_oldformat%s\n', p, nn, ee);
    end
end
newf = ~exist(p, 'file');
fid = fopen(p, 'a');
if fid < 0, error('metrics_write: cannot open %s', p); end
if newf, fprintf(fid, '%s\n', header); end
% build the line as one string: MATLAB's fprintf skips empty arguments
% (an empty study or point would shift every column)
vals = cellfun(@(c) sprintf('%.4f', M.(c)), cols, 'UniformOutput', false);
fprintf(fid, '%s\n', strjoin([{S.study, S.point, sprintf('%g', S.seed), label}, vals], ','));
fclose(fid);
end
