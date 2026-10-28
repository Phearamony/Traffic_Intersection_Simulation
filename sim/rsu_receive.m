function rsu = rsu_receive(rsu, veh, t, lat, G)
%RSU_RECEIVE  Store a vehicle state message (uplink, with latency lat).
%   A message delivered now describes the vehicle lat seconds ago: its
%   position is moved back by v*lat along its path (exact at constant
%   speed) and its time stamp is t - lat.
k = find([rsu.known.id] == veh.id, 1);
m = struct('id', veh.id, 's', veh.s - veh.v * lat, 'v', veh.v, 'a', veh.a, 't', t - lat, ...
           'p', veh.p, 'app', veh.app, 'mov', veh.mov, 'committed', veh.committed);
if isempty(k)
    rsu.known(end+1) = m;
else
    rsu.known(k) = m;
end
end
