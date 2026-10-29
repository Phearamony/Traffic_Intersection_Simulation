function cfg = sim_config(name)
%SIM_CONFIG  Switches that define each configuration.
%
%   Every configuration runs in the same engine (sim_run) with the same
%   roads, traffic, driver parameters, signals and metrics. Only these
%   switches differ.
%
%   ctrl    'idm' | 'mpc'      longitudinal controller of the CAVs
%   coord   'none'             all vehicles are human drivers (C1)
%           'v2v'              CAVs, no RSU; a right-turning CAV uses its own
%                              on-board / V2V gap check (earliest arrival,
%                              PET margin)
%           'rsu'              as v2v, plus the RSU slot planner
%   est     'idm' | 'gpr' | 'pigpr'   RSU arrival-time estimator
%   msg     'speed'            RSU sends a recommended speed (conference
%                              method): v = d / tau, used within 80 m of the
%                              stop line and never below 0.5 Vd
%           'time'             RSU sends a target arrival time; the MPC keeps
%                              the vehicle behind that point until then
%   unc_z   z of the uncertainty term in the slot margins (0 = fixed margins)
%   online  1 = PI-GPR learns from every observed arrival during the run
%   spat    1 = the controller knows the signal timing (V2I); 0 = only the
%           current light
%   prio    'law'   right-turn priority of the Road Traffic Act (art. 37):
%                   the RSU never delays oncoming straight or left-turning
%                   vehicles; the turner only uses gaps that exist
%           'coop'  cooperative yielding: the RSU may ask oncoming CAVs to
%                   arrive later so the turner can go first
%   label   results label;  fig  suffix of the figure names

c = struct('ctrl', 'idm', 'coord', 'none', 'est', 'idm', 'msg', 'speed', ...
           'unc_z', 0, 'online', 0, 'spat', 0, 'prio', 'law', 'label', '', 'fig', '', 'name', name);
switch upper(name)
    case 'C1'
        c.label = 'C1_human';        c.fig = 'human';
    case 'V2V_IDM'
        c.coord = 'v2v';             c.label = 'V2V_IDM';        c.fig = 'idm';
    case 'V2V_MPC'
        c.coord = 'v2v'; c.ctrl = 'mpc';
        c.label = 'V2V_MPC';         c.fig = 'mpc';
    case 'C2'
        c.coord = 'rsu';             c.est = 'idm';  c.spat = 1;
        c.label = 'C2_idm_rsu';      c.fig = 'idm_rsu';
    case 'C2G'
        c.coord = 'rsu';             c.est = 'gpr';  c.spat = 1;
        c.label = 'C2G_idm_rsu_gpr'; c.fig = 'idm_rsu_gpr';
    case 'C3I'
        c.coord = 'rsu'; c.ctrl = 'mpc'; c.est = 'idm'; c.spat = 1;
        c.label = 'C3I_mpc_rsu';     c.fig = 'mpc_rsu';
    case 'C3'
        c.coord = 'rsu'; c.ctrl = 'mpc'; c.est = 'gpr'; c.spat = 1;
        c.label = 'C3_mpc_rsu_gpr';  c.fig = 'mpc_rsu_gpr';
    case 'C4'
        c.coord = 'rsu'; c.ctrl = 'mpc'; c.est = 'pigpr'; c.spat = 1;
        c.msg = 'time'; c.unc_z = 1.645; c.online = 1;
        c.label = 'C4_proposed';     c.fig = 'proposed';
    case 'C4_NOUNC'
        c = sim_config('C4'); c.unc_z = 0;
        c.label = 'C4_no_unc';       c.fig = 'proposed_no_unc';
    case 'C4_NOONLINE'
        c = sim_config('C4'); c.online = 0;
        c.label = 'C4_no_online';    c.fig = 'proposed_no_online';
    case 'C4_SPEED'
        c = sim_config('C4'); c.msg = 'speed';
        c.label = 'C4_speed_adv';    c.fig = 'proposed_speed_adv';
    case 'C4_COOP'
        c = sim_config('C4'); c.prio = 'coop';
        c.label = 'C4_coop';         c.fig = 'proposed_coop';
    otherwise
        error('sim_config: unknown configuration %s', name);
end
c.name = upper(name);
cfg = c;
end
