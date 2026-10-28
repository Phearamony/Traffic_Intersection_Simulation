function sim_figures(R, outdir)
%SIM_FIGURES  The per-run figures, drawn with the earlier plotting functions.
%   EW_<fig>.png, EW_<fig>_ac.png, fuel_<fig>.png in outdir.
%   The engine's log is converted to the old CarLog / LightLog / FuelLog
%   formats (sim_oldlogs), so the figures look as before; the trajectory
%   figure shows the distance to the stop line, as before.
[LightLog, CarLogE, CarLogW, FuelLog] = sim_oldlogs(R);
f = R.cfg.fig;
stop_line = R.G.C.STOP;
plot_EW_trajectories(LightLog, CarLogE, CarLogW, stop_line, fullfile(outdir, ['EW_' f '.png']));
close all;
plot_EW_accelerations(LightLog, CarLogE, CarLogW, stop_line, fullfile(outdir, ['EW_' f '_ac.png']));
close all;
names = struct('human', 'Human Driving', 'idm', 'V2X + IDM', 'mpc', 'V2X + MPC', ...
    'idm_rsu', 'V2X + IDM + RSU', 'idm_rsu_gpr', 'V2X + IDM + RSU (GPR)', ...
    'mpc_rsu', 'V2X + MPC + RSU', 'mpc_rsu_gpr', 'V2X + MPC + RSU (GPR)', ...
    'proposed', 'Proposed (PI-GPR + arrival-time MPC)');
ttl = R.cfg.label; if isfield(names, f), ttl = names.(f); end
plot_fuel_consumption(FuelLog, ttl, fullfile(outdir, ['fuel_' f '.png']));
close all;
end
