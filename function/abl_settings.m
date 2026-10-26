function S = abl_settings()
%ABL_SETTINGS  Run settings shared by every main_*.m script.
%
%   Values come from environment variables so that run_ablation.m can loop
%   over seeds even though each main script starts with "clear".
%   When a variable is not set, the default below is used, so running a
%   main script by hand (F5) behaves as before.
%
%   ABL_SEED   random seed for traffic schedule and driver noise   (default 1)
%   ABL_KKMAX  number of 0.5 s steps per run                        (default 720 = 360 s)
%              720 steps cover the light (0-120 s), medium (120-300 s)
%              and heavy (300 s+) demand stages.
%   ABL_PLOT   1 = save the final figures (trajectories, fuel), 0 = none (default 1)
%   ABL_LIVE   1 = live intersection animation while running   (default 1)
%   ABL_NOLOG  1 = do not append rows to the results CSVs      (default 0)
%              (make_figures.m re-runs one seed for its figures and sets this)
%   ABL_VREC   1 = vehicles follow the RSU recommended speed          (default 1)
%   ABL_BATCH  1 = called from run_ablation.m (skip clear/close)     (default 0)

S.seed  = readNum('ABL_SEED',  1);
S.KKmax = readNum('ABL_KKMAX', 720);
S.plot  = logical(readNum('ABL_PLOT', 1));
S.live  = logical(readNum('ABL_LIVE', 1));
S.nolog = logical(readNum('ABL_NOLOG', 0));
S.vrec  = logical(readNum('ABL_VREC', 1));
S.batch = logical(readNum('ABL_BATCH', 0));
end

function v = readNum(name, default)
s = getenv(name);
if isempty(s)
    v = default;
else
    v = str2double(s);
    if isnan(v), v = default; end
end
end
