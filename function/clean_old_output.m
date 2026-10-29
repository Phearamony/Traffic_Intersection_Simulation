function clean_old_output()
%CLEAN_OLD_OUTPUT  Remove results of the older flat layout of output/.
%
%   Deletes every file and folder directly in output/ that is not part of
%   the current layout (README.md, main_all_log.txt and the folders of
%   out_folder.m), and the old simplot/ folder of the repository. Called by
%   main_train.m and main_all.m; results in the current folders are kept.

keep = {'.', '..', 'README.md', 'main_all_log.txt', '1_training', '2_main_study', '3_figures', ...
        '4_scenarios', '5_stress_tests', 'single_runs', 'demo_videos'};
out = out_folder('', '');
root = fileparts(out);
n = 0;
if exist(out, 'dir')
    d = dir(out);
    for k = 1:numel(d)
        if any(strcmp(d(k).name, keep)), continue; end
        p = fullfile(out, d(k).name);
        if d(k).isdir, rmdir(p, 's'); else, delete(p); end
        n = n + 1;
    end
end
if exist(fullfile(root, 'simplot'), 'dir')
    rmdir(fullfile(root, 'simplot'), 's'); n = n + 1;
end
if n > 0, fprintf('[clean_old_output] removed %d old item(s) from output/ and simplot/\n', n); end
end
