# Results (v2)

One folder per step, in the order `main/main_all.m` runs them:

| Folder | Written by | What is inside |
|---|---|---|
| `1_training/` | `main_train.m` | `estimators.mat` (trained model used by the RSU), `gpr_holdout_metrics.csv`, `gpr_model_diagnostics.png`, `estimator_samples.mat` |
| `2_main_study/` | `main_all.m` step 2 | `ablation_runs.csv` (one row per seed and configuration), `ablation_stats.csv`, `ablation_pairs.csv`, `estimator_runs.csv`, `ablation_errors.log` |
| `3_figures/` | `main_all.m` step 3, `main_figures.m` | `ablation_summary.png`, `ablation_paired.png`, trajectory, acceleration, fuel and estimator figures of the representative seed |
| `4_scenarios/` | `main_all.m` step 4 | `scenarioA/`, `scenarioB/`, `scenario_runs.csv` |
| `5_stress_tests/` | `main_all.m` step 5 | `sweep_runs.csv`, `sweep_<study>.png`, `sweep_stats.csv`, `sweep_limits.csv` |
| `single_runs/` | `main_one.m` | figures and `single_runs.csv` of single runs |
| `main_all_log.txt` | `main_all.m` | console log |

The paths are set in `function/out_folder.m`. The results of the conference paper (CISE, code v1) are in
`cise/results/` and are never overwritten.
