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
%   msg     'speed'            RSU sends a recommended speed: v = d / tau,
%                              used within 80 m of the stop line and never
%                              below 0.5 Vd
%           'time'             RSU sends target times: the MPC keeps the
%                              vehicle behind its point until then; an IDM
%                              CAV sets its desired speed to reach the point
%                              at that time (as in [5]); a turner leaves its
%                              waiting point at its go time
%   unc_z   z of the uncertainty term in the slot margins (0 = no uncertainty)
%   online  1 = PI-GPR learns from every observed arrival during the run
%   spat    1 = the controller knows the signal timing (V2I); 0 = only the
%           current light
%   slot    'opt' joint optimizer | 'seq' successive, as in [5] | 'strict'
%           existing gaps only (rsu_plan); every rule keeps the right-turn
%           priority of the Road Traffic Act (art. 37) and the same limits
%   plan    'rolling' every step | 'once' at the start of each green ([5], CISE)
%   margin  'geom' conflict-zone geometry + PET + uncertainty |
%           'fixed' 4 s safety gap and 2 s headway ([5], CISE)
%   adv     1 = the RSU may also ask CAVs to arrive earlier (up to the limit)
%   eco     1 = CAVs that have to wait anyway get their planned arrival time
%   label   results label;  fig  suffix of the figure names
%
%   Reference methods, rebuilt in this engine with the same traffic, the
%   same legal limits and the same metrics:
%     C2  [5] Hasan et al.: IDM vehicles that track their target crossing
%         times, IDM forward simulation, fixed 4 s gap, planned once per
%         green, turners one after another
%     C3  CISE (conference paper): as C2 with GPR estimator and QP-MPC

c = struct('ctrl', 'idm', 'coord', 'none', 'est', 'idm', 'msg', 'speed', ...
           'unc_z', 0, 'online', 0, 'spat', 0, 'slot', 'opt', 'plan', 'rolling', ...
           'margin', 'geom', 'adv', 0, 'eco', 0, 'label', '', 'fig', '', 'name', name);
ref = {'coord', 'rsu'; 'spat', 1; 'msg', 'time'; 'slot', 'seq'; 'plan', 'once'; ...
       'margin', 'fixed'; 'eco', 1};
switch upper(name)
    case 'C1'
        c.label = 'C1_human';        c.fig = 'human';
    case 'V2V_IDM'
        c.coord = 'v2v';             c.label = 'V2V_IDM';        c.fig = 'idm';
    case 'V2V_MPC'
        c.coord = 'v2v'; c.ctrl = 'mpc';
        c.label = 'V2V_MPC';         c.fig = 'mpc';
    case 'C2'                                  % [5]
        c = setf(c, ref); c.est = 'idm';
        c.label = 'C2_idm_rsu';      c.fig = 'idm_rsu';
    case 'C2G'
        c = setf(c, ref); c.est = 'gpr';
        c.label = 'C2G_idm_rsu_gpr'; c.fig = 'idm_rsu_gpr';
    case 'C3I'
        c = setf(c, ref); c.ctrl = 'mpc'; c.est = 'idm';
        c.label = 'C3I_mpc_rsu';     c.fig = 'mpc_rsu';
    case 'C3'                                  % CISE
        c = setf(c, ref); c.ctrl = 'mpc'; c.est = 'gpr';
        c.label = 'C3_mpc_rsu_gpr';  c.fig = 'mpc_rsu_gpr';
    case 'C4'                                  % proposed
        c.coord = 'rsu'; c.ctrl = 'mpc'; c.est = 'pigpr'; c.spat = 1;
        c.msg = 'time'; c.unc_z = 1.645; c.online = 1; c.adv = 1; c.eco = 1;
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
    case 'C4_STRICT'
        c = sim_config('C4'); c.slot = 'strict';
        c.label = 'C4_strict';       c.fig = 'proposed_strict';
    case 'C4_SEQ'
        c = sim_config('C4'); c.slot = 'seq';
        c.label = 'C4_seq';          c.fig = 'proposed_seq';
    case 'C4_NOADV'
        c = sim_config('C4'); c.adv = 0;
        c.label = 'C4_no_adv';       c.fig = 'proposed_no_adv';
    case 'C4_ONCE'
        c = sim_config('C4'); c.plan = 'once';
        c.label = 'C4_once';         c.fig = 'proposed_once';
    otherwise
        error('sim_config: unknown configuration %s', name);
end
c.name = upper(name);
cfg = c;
end

function c = setf(c, kv)
for k = 1:size(kv, 1), c.(kv{k, 1}) = kv{k, 2}; end
end
