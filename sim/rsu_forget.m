function rsu = rsu_forget(rsu, V, DONE, t) %#ok<INUSL>
%RSU_FORGET  Drop vehicles that left the model, and stale entries.
%   A vehicle that has left (DONE) is out of range. An entry whose last
%   message is older than 5 s is dropped too (lost contact).
if isempty(rsu.known), return; end
ids = [rsu.known.id];
alive = ismember(ids, [V.id]);
fresh = (t - [rsu.known.t]) <= 5;
rsu.known = rsu.known(alive & fresh);
end
