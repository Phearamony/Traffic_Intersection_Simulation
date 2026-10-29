# CISE (conference paper, code v1)

Everything that belongs to the conference paper *Learning-Based Optimal Right-Turn Coordination System for
Connected and Automated Vehicles at Intersections* (SICE Festival 2026) is in this folder, so it does not
mix with the current version. Nothing here is on the MATLAB path or used by the scripts in `main/`.

| Folder | Content |
|---|---|
| `code/` | The v1 code: `class/` (`Car.m`, `RSU.m`), `main/` (the per-configuration scripts, GPR training, estimator comparison, v1 runners), `function/` (v1 helpers), `simplot/` (`InitVals.m`, `SimPlotIntersection.m`, the v1 GPR model and traffic schedule) |
| `results/` | The full v1 run (commit `58dfe95`, "full run results"): figures, `ablation_runs.csv`, `ablation_stats.csv`, `ablation_pairs.csv`, estimator comparison, scenarios, sweeps |
| `older/` | Files from before v1: earlier `Car.m` / `RSU.m` versions, backup scripts and figures, the IDM "mimic" experiment, and `output_cise2026/` (the single-seed `ablation_summary.csv` and the short-run estimator results used while writing the paper) |

To run the v1 code again, check out commit `58dfe95` (`git checkout 58dfe95`), or copy `code/` back in
place of `sim/`, `function/` and `main/`.
