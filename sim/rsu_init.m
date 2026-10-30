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
rsu.tau_safe = 4.0;                  % [s] fixed safety gap of [5] and CISE (cfg.margin = 'fixed')
if strcmp(cfg.margin, 'fixed'), rsu.h_min = 2.0; end   % h_time = 2 s in [5] and CISE
rsu.pet = 1.5;                       % [s] PET margin of a slot
% gap shaping limits (rsu_plan): oncoming CAVs only
rsu.dmax = 2.0;                      % [s] largest extra delay per vehicle (ABL_DMAX)
if isfield(S, 'dmax'), rsu.dmax = S.dmax; end
rsu.v_frac = 0.7;                    % never slower than this fraction of its speed
rsu.a_shape = 1.0;                   % [m/s2] deceleration used for the limit
rsu.d_cmd = 40;                      % [m] no new command closer to the stop line
rsu.d_yield = 5.0;                   % [s] courtesy yield (cfg.yield): extra wait of a stopped CAV
rsu.a_max = 2.0;                     % [m/s2] fastest acceleration of a CAV (MPC bound), for entry times
rsu.adv_max = 1.5;                   % [s] largest gain asked of a pulled CAV (cfg.adv)
rsu.w_adv = 0.25;                    % price of asking a vehicle to arrive earlier
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
