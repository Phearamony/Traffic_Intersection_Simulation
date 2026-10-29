function E = rsu_results(rsu)
%RSU_RESULTS  What the RSU hands back at the end of a run.
E.samples = rsu.samples;
E.evalrows = rsu.evalrows;
E.shp = rsu.shp;
E.diag = rsu.diag;
E.plans = rsu.plans; E.noslot = rsu.noslot; E.online_n = rsu.online_n;
end
