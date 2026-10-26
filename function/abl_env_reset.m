function abl_env_reset()
%ABL_ENV_RESET  Clear every ABL_* environment variable (back to defaults).
%   Called by the runners so a setting from one study never leaks into
%   the next run. See abl_settings.m for the list.
names = {'ABL_SEED','ABL_KKMAX','ABL_PLOT','ABL_LIVE','ABL_NOLOG','ABL_BATCH', ...
         'ABL_SCHED_ONLY','ABL_OUTDIR','ABL_CSV','ABL_EST_CSV','ABL_STUDY','ABL_POINT', ...
         'ABL_SAVELOG','ABL_SCHED','ABL_DEMAND','ABL_RT','ABL_ASYM','ABL_DRIVER', ...
         'ABL_VREC','ABL_LAT','ABL_LOSS','ABL_PEN'};
for i = 1:numel(names)
    setenv(names{i}, '');
end
end
