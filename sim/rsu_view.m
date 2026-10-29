function W = rsu_view(rsu, t)
%RSU_VIEW  The RSU's picture of the traffic now (positions extrapolated).
%   Each known vehicle is moved forward from its last message at its
%   reported speed: s = s_msg + v_msg (t - t_msg).
K = rsu.known;
if isempty(K)
    W = struct('ids', [], 's', [], 'v', [], 'a', [], 'p', [], 'app', '', 'mov', '', 'committed', false(1, 0), 'cav', false(1, 0));
    return;
end
W.ids = [K.id];
W.v = [K.v]; W.a = [K.a];
W.s = [K.s] + W.v .* (t - [K.t]);
W.p = [K.p]; W.app = [K.app]; W.mov = [K.mov];
W.committed = logical([K.committed]);
W.cav = logical([K.cav]);
end
