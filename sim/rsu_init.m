function rsu = rsu_init(cfg, G, S, opts)
%RSU_INIT  Roadside unit: what it knows, its estimators and its logs.
%
%   The RSU only knows what it received over V2X (rsu_receive): position,
%   speed, acceleration and path of each connected vehicle, with the
%   time stamp of the message. Human drivers are invisible unless
%   ABL_SENSE = 'all' (roadside sensing). In the estimator-evaluation and
%   data-collection modes (opts.est_eval / opts.collect) it sees every
%   vehicle, like a perfect roadside sensor, and plans nothing.
%
%   Estimator models (output/1_training/estimators.mat, from train_estimators):
%   theta (physics), gpr (4 inputs), pigpr (7 inputs, residual on physics).

rsu.cfg = cfg;
rsu.collect = isfield(opts, 'collect') && opts.collect;
rsu.eval = isfield(opts, 'est_eval') && opts.est_eval;
rsu.plan = strcmp(cfg.coord, 'rsu') && ~rsu.collect && ~rsu.eval;
rsu.known = struct('id', {}, 's', {}, 'v', {}, 'a', {}, 't', {}, 'p', {}, ...
                   'app', {}, 'mov', {}, 'committed', {}, 'cav', {});
rsu.snap_every = 2.0;                % [s] snapshot interval for learning / evaluation
rsu.pending = struct('id', {}, 't', {}, 'app', {}, 'g', {}, 'z', {}, 'z4', {}, 'tphys', {}, 'pred', {});
rsu.samples = zeros(0, 14);          % [t_snap id app(1=E,2=W) d v gap g n vl nR prior tphys y idm]
rsu.evalrows = zeros(0, 12);         % see rsu_crossed
rsu.lambda = 0.995;                  % forgetting factor of online learning
rsu.h_min = 1.0;                     % [s] minimum headway between stop-line arrivals
rsu.tau_safe = 4.0;                  % [s] fixed safety gap of [5] and SICE (cfg.margin = 'fixed')
if strcmp(cfg.margin, 'fixed'), rsu.h_min = 2.0; end   % h_time = 2 s in [5] and SICE
rsu.pet = 1.5;                       % [s] PET margin of a slot
% gap shaping limits (rsu_plan): oncoming CAVs only
rsu.dmax = 2.0;                      % [s] largest extra delay per vehicle (ABL_DMAX)
if isfield(S, 'dmax'), rsu.dmax = S.dmax; end
rsu.v_frac = 0.7;                    % never slower than this fraction of its speed
rsu.a_shape = 1.0;                   % [m/s2] deceleration used for the limit
rsu.d_cmd = 40;                      % [m] no new command closer to the stop line
rsu.adv_max = 1.5;                   % [s] largest gain asked of a pulled CAV (cfg.adv)
rsu.w_adv = 0.25;                    % price of asking a vehicle to arrive earlier
% gap insertion (cfg.gi): the RSU may hold an oncoming CAV at its stop line
rsu.gi_hmax = 20;                    % [s] longest hold of one oncoming CAV
rsu.gi_b = 3.0;                      % [m/s2] it must be able to stop at this deceleration
rsu.gi_gap = 10;                     % [s] green time one inserted gap costs (about Db + Da)
rsu.gi_hsat = 2.0;                   % [s] saturation headway (vehicles per green second)
rsu.gi_margin = 1.0;                 % [veh] net gain required before a gap is inserted
rsu.gi_reach = 3.0;                  % [s] turner at most this far from its waiting point
% cooperative leading right turn (cfg.clt): queued or slow oncoming CAVs may
% be held so that the front right-turner goes first
rsu.clt_dmax = 15;                   % [s] longest hold of one oncoming CAV (cfg.jtp = 0)
rsu.clt_win = 15;                    % [s] only CAVs due within this time of the turner
rsu.clt_reach = 6;                   % [s] turner at most this far from its waiting point
rsu.clt_dmax_tp = 25;                % [s] hold limit and window with the throughput price (cfg.jtp)
% throughput-priced slot cost (cfg.jtp)
rsu.tp_R = 60;                       % [s] price of a known vehicle pushed past the end of green (about one red)
rsu.tp_edge = 110;                   % [m] a lane is saturated to the edge of view when its last vehicle is this far out
rsu.h_q = 2.0;                       % [s] headway of a queue starting from rest (cfg.qd)
rsu.clt_vstand = 0.5;                 % [m/s] with cfg.qd only standing CAVs are held for a lead turn
rsu.l_s = 2.0;                       % [s] start-up time lost by a moving CAV held to a stop (cfg.qd)
rsu.gi_ids = zeros(1, 0);            % CAVs held for an inserted gap
rsu.gi_tid = zeros(1, 0);            % the turner each was held for
rsu.shp = struct('id', zeros(1, 0), 'nat', zeros(1, 0), 'tgt', zeros(1, 0), 't0', zeros(1, 0));
rsu.pul = struct('id', zeros(1, 0), 'gain', zeros(1, 0));   % pulled forward
rsu.eco = zeros(1, 0);               % CAVs that got an eco arrival time
rsu.lastCmd = zeros(1, 0);           % CAVs that got a command in the last step (for releases)
rsu.once_tg0 = -inf;                 % planning once per green: green already planned
rsu.once_ids = zeros(1, 0);          % turners already planned in this green
rsu.once_msgs = struct('id', {}, 'vadv', {}, 'tgt', {}, 'spt', {}, 'go', {}, 'pull', {}, 'hard', {});
rsu.plans = 0; rsu.noslot = 0; rsu.online_n = 0;
rsu.diag = zeros(1, 4);              % joint optimizer: [plans, >=2 turners, real choice, differs from successive ([5])]
rsu.theta = [2.0, 2.0, 1.5];
rsu.gpr = []; rsu.pigpr = []; rsu.pigpr_on = [];

needModel = rsu.eval || (rsu.plan && any(strcmp(cfg.est, {'gpr', 'pigpr'})));
mf = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'output', '1_training', 'estimators.mat');
if isfield(S, 'estfile') && ~isempty(S.estfile), mf = S.estfile; end
if exist(mf, 'file')
    E = load(mf);
    if isfield(E, 'theta'), rsu.theta = E.theta; end
    if isfield(E, 'gpr'), rsu.gpr = E.gpr; end
    if isfield(E, 'pigpr'), rsu.pigpr = E.pigpr; rsu.pigpr_on = E.pigpr; end
elseif needModel
    error('rsu_init: %s not found. Run main/main_train.m first.', mf);
end
end
