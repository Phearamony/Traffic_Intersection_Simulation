function out = v2x_channel(mode, varargin)
%V2X_CHANNEL  Latency and packet loss between vehicles and the RSU.
%
%   v2x_channel('reset')              at the start of a main script
%   data = v2x_channel('up', data)    uplink, vehicle -> RSU; [] if lost
%   pkts = v2x_channel('down', car, data)   downlink, RSU -> vehicle;
%                                     cell array of packets delivered now
%
%   Model (ABL_LAT = L seconds one way, ABL_LOSS = p per message):
%   Uplink   each state message is lost with probability p. A delivered
%            message describes the vehicle L seconds ago: its timestamp is
%            t - L and its position is moved back by v*L along its lane
%            (exact for constant speed). The RSU keeps using the last
%            message it received for a vehicle.
%   Downlink each advisory is lost with probability p and is delivered
%            ceil(L/dt) simulation steps after it was sent (dt = 0.5 s, so
%            sub-step latency is rounded up to one step). A CAV ignores an
%            advisory older than Car.X2V_TIMEOUT and falls back to local
%            gap acceptance (rt_decision.m).
%   With L = 0 and p = 0 the channel is ideal and changes nothing.
%
%   Losses are drawn from their own random stream (seed 1e6 + ABL_SEED),
%   so the traffic and the driver noise are the same at every loss level.

persistent cfg rs
if isempty(cfg) || strcmp(mode, 'reset')
    S = abl_settings();
    cfg = struct('lat', max(S.lat, 0), 'loss', min(max(S.loss, 0), 1), 'dt', 0.5);
    rs = RandStream('mt19937ar', 'Seed', 1e6 + S.seed);
    if strcmp(mode, 'reset'), out = []; return; end
end

switch mode
    case 'up'
        data = varargin{1};
        if cfg.loss > 0 && rand(rs) < cfg.loss
            out = []; return;
        end
        if cfg.lat > 0
            L = cfg.lat;
            switch upper(char(data.Dir))
                case 'E', data.X = data.X - data.V * L;   % E moves +x
                case 'W', data.X = data.X + data.V * L;   % W moves -x
                case 'N', data.Y = data.Y - data.V * L;   % N moves +y
                case 'S', data.Y = data.Y + data.V * L;   % S moves -y
            end
            data.t = data.t - L;
        end
        out = data;

    case 'down'
        car = varargin{1}; data = varargin{2};
        if cfg.loss == 0 && cfg.lat == 0
            out = {data}; return;
        end
        if ~(cfg.loss > 0 && rand(rs) < cfg.loss)
            car.x2v_queue{end+1} = data;
        end
        now_t = data.t;
        delay = ceil(cfg.lat / cfg.dt - 1e-9) * cfg.dt;
        out = {}; keep = {};
        for k = 1:numel(car.x2v_queue)
            q = car.x2v_queue{k};
            if q.t + delay <= now_t + 1e-9
                out{end+1} = q; %#ok<AGROW>
            else
                keep{end+1} = q; %#ok<AGROW>
            end
        end
        car.x2v_queue = keep;
end
end
