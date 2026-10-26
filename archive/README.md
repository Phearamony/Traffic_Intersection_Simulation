# Archive

Old and unused files, kept for reference. Nothing here is on the MATLAB path or used by the current scripts.

| Folder | Content |
|---|---|
| `class/` | Earlier versions of `Car.m` and `RSU.m` (`OLD_CAR.m`, `OLD_RSU.m`, `LATEST_OLD_RSU.m`) |
| `main/backup/` | Earlier versions of the main scripts |
| `main/main_v2v_idm_mimic.m` | Experimental IDM "mimic" configuration, not part of the paper ablation (still has hard-coded paths) |
| `function/backup/` | Earlier versions of the trajectory plotting functions |
| `simplot/backup/` | Earlier GPR model and traffic schedules (`traffic_schedule_3000.mat` = 3000-step schedule) |
| `output/backup/` | Earlier figures and estimator results (the long-run estimator comparison, n = 636, is `estimator_comparison_results.mat` here) |
| `output/*mimic*.png` | Figures of the mimic configuration |
| `output_cise2026/` | Results files of the CISE 2026 paper: single-seed `ablation_summary.csv` and the short-run `estimator_comparison_results.mat` (n = 51). The new code writes `ablation_runs.csv` instead. |

To use an archived file again, move it back with `git mv`.
