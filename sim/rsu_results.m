function E = rsu_results(rsu)
%RSU_RESULTS  What the RSU hands back at the end of a run.
E.samples = rsu.samples;
E.evalrows = rsu.evalrows;
E.shp = rsu.shp; E.pul = rsu.pul; E.eco = rsu.eco;
E.diag = rsu.diag;
E.n_gi = 0; if isfield(rsu, 'gi_ids'), E.n_gi = numel(rsu.gi_ids); end
E.plans = rsu.plans; E.noslot = rsu.noslot; E.online_n = rsu.online_n;
end
