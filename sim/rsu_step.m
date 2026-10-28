function [rsu, msgs] = rsu_step(rsu, t, sig, sigT, G, cfg)
%RSU_STEP  One RSU step: learning snapshots, then slot planning.
%
%   Snapshots (every rsu.snap_every seconds): the features of every known
%   vehicle before its stop line are stored. When the vehicle later crosses
%   (rsu_crossed) the true arrival time becomes a training sample, an
%   evaluation sample, or an online update of the PI-GPR.
%   Planning (configurations with an RSU): rsu_plan for every approach that
%   has a right-turner.
msgs = struct('id', {}, 'vadv', {}, 'tgt', {}, 'spt', {}, 'go', {});
W = rsu_view(rsu, t);
snap = abs(t / rsu.snap_every - round(t / rsu.snap_every)) < 1e-9;
learn = rsu.collect || rsu.eval || (rsu.plan && cfg.online && strcmp(cfg.est, 'pigpr'));
apps = {'E', 'W'};
if snap && learn && ~isempty(W.ids)
    for a = 1:numel(apps)
        F = est_features(W, apps{a}, t, G, rsu.theta);
        k = numel(F.ids);
        if k == 0, continue; end
        pred = nan(k, 8);
        prior = nan(1, k);
        if rsu.eval
            [~, ti] = rsu_estimate(rsu, W, apps{a}, t, G, 'idm');
            [~, tp] = rsu_estimate(rsu, W, apps{a}, t, G, 'phys');
            [~, tg, sg] = rsu_estimate(rsu, W, apps{a}, t, G, 'gpr');
            [Fq, tq, sq] = rsu_estimate(rsu, W, apps{a}, t, G, 'pigpr');
            [~, to, so] = rsu_estimate(rsu, W, apps{a}, t, G, 'pigpr_on');
            pred = [ti(:), tp(:), tg(:), sg(:), tq(:), sq(:), to(:), so(:)];
            prior = Fq.prior;
        else
            % IDM prior (raw, before the minimum-headway chain)
            ti = est_idm(W, F, apps{a}, t, G);
            pred(:, 1) = ti(:);
            prior = ti; prior(~isfinite(prior)) = F.tphys(~isfinite(prior));
        end
        for q = 1:k
            rsu.pending(end+1) = struct('id', F.ids(q), 't', t, 'app', a, 'g', F.g(q), ...
                'z', [F.Zb(q, :), prior(q)], 'z4', F.Z4(q, :), 'tphys', F.tphys(q), ...
                'pred', pred(q, :));
        end
    end
end
if rsu.plan
    for a = 1:numel(apps)
        [rsu, m] = rsu_plan(rsu, W, apps{a}, t, sig, sigT, G, cfg);
        msgs = [msgs, m]; %#ok<AGROW>
    end
end
end
