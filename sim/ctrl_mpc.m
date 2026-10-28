function [acc, veh, info] = ctrl_mpc(veh, gap, vl, sStop, G, cfg, t, sigT, sigNow, al)
%CTRL_MPC  Quadratic-programming MPC along the vehicle's path.
%
%   [acc, veh, info] = ctrl_mpc(veh, gap, vl, sStop, G, cfg, t, sigT, sigNow, al)
%   al  acceleration of the vehicle ahead (shared over V2V); the vehicle
%       ahead is predicted with it until it would stop, then standing
%
%   Horizon N = 20 steps of dt = 0.5 s (10 s). Decision variables are the
%   accelerations u_0..u_{N-1} (piecewise constant). With s, v the current
%   position and speed:
%       v_k = v + dt sum_{i<k} u_i
%       s_k = s + k dt v + dt^2 sum_{i<k} (k - i - 1/2) u_i
%   Cost
%       J = w_v sum (v_k - vref_k)^2 + w_u sum u_k^2 + w_j sum (u_k - u_{k-1})^2
%   with w_v = 0.1, w_u = 2, w_j = 10
%   (u_{-1} = current acceleration; the last term penalises jerk).
%   vref_k = min(Vd, curve speed at the predicted position s_k), and in the
%   speed-advice configurations min(vref, max(v_adv, 0.5 Vd)) within 80 m
%   of the stop line.
%   Constraints
%     -6 <= u_k <= 2,   0 <= v_k <= curve speed at s_k
%     gap        s_k + T_c v_k <= s + gap + d_l(k) - s0_c   (T_c = 1.2 s, s0_c = 2 m)
%                d_l(k) distance the vehicle ahead drives in k steps
%     signal     s_k <= stop line while the light is red; with the signal
%                timing (spat) only for t_k before the next green start
%     turn       a right-turner without permission: s_k <= waiting point,
%                only until its RSU go time when it has one (arrival-time
%                mode), otherwise for the whole horizon
%     RSU target arrival-time mode: s_k <= stop line for t_k before the
%                target time sent by the RSU (kept until that time, even if
%                no newer message arrives, for up to 10 s after receipt)
%   If the QP has no solution the vehicle uses IDM for this step
%   (counted in info.fail, reported as qp_fail_pct).

info = struct('solves', 1, 'fail', 0);
C = G.C; P = G.P(veh.p); dt = C.DT; N = 20; L = C.VEH_L;
persistent Sv Ss Dj
if isempty(Sv) || size(Sv, 1) ~= N
    Sv = dt * tril(ones(N));
    Ss = zeros(N);
    for k = 1:N
        for i = 1:k
            Ss(k, i) = dt^2 * (k - i + 0.5);
        end
    end
    Dj = eye(N) - diag(ones(N-1, 1), -1);       % u_k - u_{k-1}
end
if nargin < 10, al = 0; end
wv = 0.1; wu = 2.0; wj = 10.0; Tc = 1.2; s0c = 2.0;
s = veh.s; v = veh.v; tk = t + (1:N)' * dt;

% predicted positions for the curve speed (previous plan, else constant speed)
if numel(veh.planS) == N
    sh = [veh.planS(2:end); veh.planS(end) + v * dt];
else
    sh = s + (1:N)' * dt * v;
end
idx = min(numel(P.s), max(1, floor(sh / C.DS) + 1));
vcur = P.vcurve(idx); vcur = vcur(:);
vref = min(veh.Vd, vcur);
fresh = (t - veh.msg_rx) <= 1.0 + 1e-9 && ~isempty(fieldnames(veh.msg));
if fresh && strcmp(cfg.msg, 'speed') && isfield(veh.msg, 'vadv') && isfinite(veh.msg.vadv)
    dz = P.s_stop - s;
    if dz > 0 && dz <= 80
        vref = min(vref, max(veh.msg.vadv, 0.5 * veh.Vd));
    end
end

Hm = 2 * (wv * (Sv' * Sv) + wu * eye(N) + wj * (Dj' * Dj));
e0 = zeros(N, 1); e0(1) = veh.a;
fv = 2 * (wv * Sv' * (v - vref) - wj * Dj' * e0);

A = [Sv; -Sv]; b = [max(vcur, 1) - v; v * ones(N, 1)];
if isfinite(gap)
    tt = (1:N)' * dt;
    if al < -1e-6 && vl > 0
        tstop = vl / -al;                         % the vehicle ahead stops
        dl = vl * min(tt, tstop) + 0.5 * al * min(tt, tstop).^2;
    else
        al_ = min(max(al, 0), 1.5);
        dl = vl * tt + 0.5 * al_ * tt.^2;
    end
    A = [A; Ss + Tc * Sv];
    b = [b; s + gap + dl - s0c - (s + tt * v) - Tc * v];
end
% stop constraints
lim = inf(N, 1);
if s < P.s_stop + 0.05 && sStop <= P.s_stop + 1e-6
    % signal stop (the engine decided this vehicle stops)
    if cfg.spat && strcmp(sigNow, 'red')
        lim(tk < sigT(1)) = P.s_stop - 0.05;
    else
        lim(:) = P.s_stop - 0.05;
    end
end
if veh.mov == 'R' && ~veh.committed && isfinite(sStop) && abs(sStop - P.s_wait) < 1e-6
    if fresh && strcmp(cfg.msg, 'time') && isfield(veh.msg, 'go') && isfinite(veh.msg.go)
        lim(tk < veh.msg.go) = min(lim(tk < veh.msg.go), P.s_wait - 0.05);
        % at the go time the on-board check decides (engine); keep the stop
        % for the first step so the vehicle never passes without permission
        lim(1) = min(lim(1), P.s_wait - 0.05);
    else
        lim(:) = min(lim, P.s_wait - 0.05);
    end
end
% any other stop point from the engine (yielding to a right-turner that is
% already turning) holds for the whole horizon
if isfinite(sStop) && sStop > s && ~(sStop <= P.s_stop + 1e-6) && ...
        ~(veh.mov == 'R' && abs(sStop - P.s_wait) < 1e-6)
    lim(:) = min(lim, sStop - 0.05);
end
% an arrival-time target is kept until its time has passed (it only ever
% holds the vehicle back), as long as it was received in the last 10 s
holdTgt = ~isempty(fieldnames(veh.msg)) && (t - veh.msg_rx) <= 10 && strcmp(cfg.msg, 'time') ...
    && isfield(veh.msg, 'tgt') && isfinite(veh.msg.tgt) && t < veh.msg.tgt;
if holdTgt && veh.mov ~= 'R' && s < veh.msg.spt
    m = tk < veh.msg.tgt;
    lim(m) = min(lim(m), veh.msg.spt - 0.05);
end
m = isfinite(lim);
if any(m)
    A = [A; Ss(m, :)];
    b = [b; lim(m) - (s + find(m) * dt * v)];
end

U0 = zeros(N, 1);
if numel(veh.U) == N, U0 = [veh.U(2:end); veh.U(end)]; end
U0 = min(max(U0, -6), 2);
[U, ok] = qp_solve(Hm, fv, A, b, -6 * ones(N, 1), 2 * ones(N, 1), U0);
if ~ok
    info.fail = 1;
    g = gap; vlead = vl;
    sHold = sStop;
    if holdTgt && veh.mov ~= 'R' && s < veh.msg.spt, sHold = min(sHold, veh.msg.spt); end
    if sHold - s < g, g = sHold - s; vlead = 0; end
    v0 = min(vref(1), veh.Vd);
    acc = ctrl_idm(v, v0, g, vlead, struct('a', 1.5, 'b', 2.5, 's0', 2.0, 'T', veh.T));
    acc = min(acc, curve_cap(P, s, v, 2.5));
    veh.U = []; veh.planS = [];
    return;
end
acc = U(1);
veh.U = U;
veh.planS = s + (1:N)' * dt * v + Ss * U;
end
