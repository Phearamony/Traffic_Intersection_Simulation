# Learning-Based Optimal Right-Turn Coordination for Connected and Automated Vehicles

MATLAB simulation for my master's research at **Gunma University (Kamal Laboratory)**. It studies how
Connected and Automated Vehicles (CAVs) can turn right efficiently at signalized intersections in
**left-hand traffic** (Japan).

📄 **Paper:** [Learning-Based Optimal Right-Turn Coordination System for Connected and Automated Vehicles at Intersections](https://drive.google.com/file/d/1eD-5lxTvvwlGY6lJCwVpVNrC4kos1YCi/view?usp=sharing)
*Phearamony Phan, Mahmudul Hasan, A. S. M. Bakibillah, Md Abdus Samad Kamal, Kou Yamada*.
SICE Festival 2026 (SICE Annual Conference), Yokohama, Japan.

![Trajectories under the proposed GPR + MPC + RSU coordination](output/demo_EW_trajectories.png)

---

## The problem

At many Japanese intersections there is no dedicated right-turn lane or protected arrow. A right-turning
car has to wait at the stop line for a gap in the oncoming traffic. While it waits, every car behind it
is blocked. The queue can spill over into the next red phase, which wastes time and fuel.

## The proposed system

A Road Side Unit (RSU) at the intersection collects vehicle states over V2X and plans a gap for each
right-turning vehicle at the start of every green phase.

1. **Bayesian GPR arrival-time estimator:** a Gaussian Process Regression model, trained offline on
   driving data, predicts when each vehicle will reach the stop line from its distance and speed `(d, v)`.
   It always returns a finite estimate (IDM forward simulation fails in 18.1% of cases).
2. **Slot-search optimizer (RSU):** finds the insertion slot in the oncoming stream that minimizes total
   squared delay, then assigns a coordinated crossing time to every vehicle.
3. **QP-MPC controller:** each CAV runs a finite-horizon quadratic-programming MPC that tracks its
   assigned crossing time while respecting acceleration and safe-gap constraints.

## Key results (CISE 2026 paper)

These are the single-seed results of the conference paper, from the code before the journal changes
(`archive/output_cise2026/ablation_summary.csv`). The journal version reruns the ablation over 20 seeds
with the fixes listed under *Journal experiments*; its results replace this table once available.

| Config | Controller | RSU estimator | Right-turn fuel [mL] | Straight fuel [mL] | Gap wait [s] |
|---|---|---|---|---|---|
| C1: human driving | IDM + noise | none | 67.59 | 34.45 | 42.32 |
| C2: reference | IDM | IDM | 60.35 | 45.53 | 24.20 |
| C2G | IDM | GPR | 60.35 | 45.53 | 24.20 |
| C3I | QP-MPC | IDM | 42.90 | 36.38 | 28.50 |
| **C3: proposed** | **QP-MPC** | **GPR** | **42.90** | **36.38** | **28.50** |

* **37% lower right-turn fuel consumption** than uncoordinated human driving.
* Most of the fuel benefit comes from pairing QP-MPC with RSU-assigned crossing times. MPC brakes once
  and smoothly, while IDM misses its slot and oscillates.
* GPR gives the same slot assignments as IDM forward simulation at a fraction of the compute cost: one
  closed-form evaluation instead of up to 120 simulated steps per vehicle.

![GPR arrival-time model diagnostics](output/gpr_model_diagnostics.png)

---

## Repository structure

```
class/        Car.m (IDM / QP-MPC vehicle), RSU.m (V2X buffer, GPR estimator, slot search)
function/     run settings, metrics, statistics and plotting functions
  abl_settings.m            seed, horizon, plot and logging switches for every script
  collect_ablation_metrics  one results row per (seed, configuration)
  compute_turn_safety       time gap between each right turn and oncoming traffic
  summarize_ablation        means, 95% CIs, paired tests
  plot_*                    trajectory, fuel and multi-seed summary figures
main/         entry points: one script per configuration, plus run_ablation and make_figures
simplot/      intersection drawing, initial values, trained GPR model, shared traffic schedule
output/       figures (EW_*, NS_*, fuel_*, estimator_*, ablation_*), results CSVs, demo video
archive/      old versions and unused files, kept for reference (see archive/README.md)
```

### Main scripts

| Script | Configuration |
|---|---|
| `main/main_human_driving.m` | Generates the shared traffic schedule for the current seed (and runs C1) |
| `main/main_human_driving_plot.m` | C1: uncoordinated human driving (IDM + noise), writes the C1 metrics |
| `main/run_ablation.m` | Runs every configuration over many seeds and summarizes the results |
| `main/make_figures.m` | Makes every journal figure after `run_ablation` |
| `main/run_scenarios.m` | Illustrative fixed scenarios for the paper |
| `main/watch_main.m` | Watch any configuration as a live animation; writes nothing to `output/` or the results CSVs |
| `main/run_sweeps.m` | Stress tests (demand, right-turn share, latency, loss, penetration, drivers, asymmetry) |
| `main/main_v2v_idm.m` | V2V-only IDM baseline |
| `main/main_v2v_idm_rsu.m` | C2: IDM + RSU (IDM estimator) |
| `main/main_v2v_idm_rsu_gpr.m` | C2G: IDM + RSU (GPR estimator) |
| `main/main_v2v_mpc.m` | V2V-only QP-MPC baseline |
| `main/main_v2v_mpc_rsu.m` | C3I: QP-MPC + RSU (IDM estimator) |
| `main/main_v2v_mpc_rsu_gpr.m` | **C3: proposed system** |
| `main/main_human_driving_gpr_train.m` | Collect data and train the GPR arrival-time model |
| `main/main_estimator_compare.m` | Compare GPR, IDM forward simulation and the naive `d/v` estimator |
| `main/demo_main.m` | Light-traffic demo with live intersection and trajectory animation |

## Getting started

Requirements: **MATLAB** (R2023a or newer recommended) with the Optimization Toolbox (`quadprog`) and the
Statistics and Machine Learning Toolbox (GPR).

```matlab
% from the repo root
cd main
demo_main          % quick visual demo of the proposed system
```

Every `main_*.m` script now finds the repo folders from its own location, so no path edits are needed.

### Watching a configuration (nothing saved)

```matlab
watch_main('C3')                       % proposed system, seed 101
watch_main('C1', 105)                  % human driving, seed 105
watch_main('C3', 1, 'ABL_SCHED', 'scenarioB')                       % illustrative scenario
watch_main('C3', 201, 'ABL_SCHED', 'const', 'ABL_PEN', '0.5')       % 50% CAVs
```

`watch_main` makes the traffic for that seed in a temporary folder, plays the live intersection
animation, and writes no results row, no figure and no `simplot/traffic_schedule.mat`.
Pressing F5 on a `main_*.m` script still works as before, but that does save figures and a results row.

## How to run everything (step by step)

All commands go in the MATLAB Command Window. Close MATLAB, the `output` folder in Explorer, and pause
OneDrive (if the repo is inside it) before pulling, so no file is locked.

**Before you start** (PowerShell, in the repo folder):

```
git pull origin main
```

Then in MATLAB:

```matlab
cd main
```

| Step | Command | What it does | Output |
|---|---|---|---|
| 0 (optional) | `watch_main('C3')` | Live animation of one configuration, nothing saved | none |
| 1 | `main_human_driving_gpr_train` | Train the GPR model once (seeds 1-16 train, 17-20 test) | `simplot/gpr_arrival_model.mat`, `output/gpr_holdout_metrics.csv`, `output/gpr_model_diagnostics.png` |
| 2 | `delete ../output/ablation_runs.csv` and `delete ../output/estimator_runs.csv` | Start the results fresh (a "file not found" warning is fine) | |
| 3 | `run_ablation(101:105)` | Quick test: 5 seeds x 7 configurations | rows in `output/ablation_runs.csv` |
| | | Check: every configuration printed a `[metrics]` line and `output/ablation_errors.log` is empty or missing. Then repeat step 2. | |
| 4 | `run_ablation(101:120, 720, true)` | Full main ablation: 20 seeds x 7 configurations + estimator comparison (about 4x step 3) | `output/ablation_runs.csv`, `ablation_stats.csv`, `ablation_pairs.csv`, `estimator_runs.csv` |
| 5 | `make_figures` | Main figures from one typical seed + multi-seed summary (adds no rows to the CSVs) | `output/EW_*.png`, `NS_*.png`, `fuel_*.png`, `estimator_comparison_*.png`, `ablation_summary.png`, `ablation_paired.png` |
| 6 | `run_scenarios` | Illustrative scenarios A and B | `output/scenarios/` |
| 7 | `run_sweeps` | All stress tests, seeds 201-210 (about 1,070 runs, the longest step). Resumable: Ctrl+C any time, run again to continue | `output/sweeps/sweep_runs.csv`, `sweep_estimator.csv` |
| 8 | `plot_sweeps` | Sweep figures and tables (only reads the CSVs, re-run any time) | `output/sweeps/sweep_<study>.png`, `sweep_stats.csv`, `sweep_limits.csv` |

Step 7 one study at a time instead:

```matlab
run_sweeps('demand')        % demand 0.1-0.8 veh/s
run_sweeps('rtshare')       % right-turn share
run_sweeps('latency')       % V2X latency
run_sweeps('loss')          % packet loss
run_sweeps('penetration')   % mixed traffic, CAV share 0-100%
run_sweeps('driver')        % aggressive / nominal / cautious drivers
run_sweeps('asym')          % unbalanced east-west demand
```

To start the sweeps over, delete the `output/sweeps/` folder.

**If something fails:** the runners do not stop on an error. They write it to `output/ablation_errors.log`
(main ablation) or `output/sweeps/sweep_errors.log` (sweeps) and continue.

**Files that hold the results** (for the manuscript tables): `output/ablation_stats.csv`,
`output/ablation_pairs.csv`, `output/estimator_runs.csv`, `output/gpr_holdout_metrics.csv`,
`output/scenarios/scenario_runs.csv`, `output/sweeps/sweep_stats.csv`, `output/sweeps/sweep_limits.csv`.

### Function inputs

| Call | Inputs | Defaults |
|---|---|---|
| `watch_main(name, seed, 'ABL_...', 'value', ...)` | `name`: `C1`, `V2V_IDM`, `V2V_MPC`, `C2`, `C2G`, `C3I`, `C3`, `HUMAN`, or a script name. `seed`: any whole number. Then any number of setting pairs (table below) | `'C3'`, `101` |
| `run_ablation(seeds, kkmax, with_estimator)` | `seeds`: list such as `101:105` or `[101 107]`. `kkmax`: steps of 0.5 s. `with_estimator`: `true`/`false` | `101:120`, `720`, `false` |
| `run_sweeps(studies, seeds)` | `studies`: one name, a cell list such as `{'latency','loss'}`, or `[]` for all | all studies, `201:210` |
| `run_scenarios(names)` | `'scenarioA'`, `'scenarioB'`, or a cell list | both |
| `make_figures(seed, kkmax)` | `seed`: seed for the trajectory figures | typical seed, `720` |
| `plot_sweeps`, `main_human_driving_gpr_train`, `demo_main` | none | |

Seeds: 1-20 GPR training/test, 101-120 main ablation, 201-210 sweeps. Watching a seed from these ranges
with the same settings shows exactly the logged run.

### Settings for `watch_main` (and hand runs)

| Setting | Values | Default |
|---|---|---|
| `ABL_SCHED` | `'staged'` (light, medium, heavy), `'const'`, `'scenarioA'`, `'scenarioB'` | `'staged'` |
| `ABL_DEMAND` | east-west flow [veh/s], used with `'const'` | `0.4` |
| `ABL_RT` | P(right turn given not left), 0 to 1 | `0.15` |
| `ABL_ASYM` | westbound share of east-west flow, 0 to 1 | `0.5` |
| `ABL_DRIVER` | `'nominal'`, `'aggressive'`, `'cautious'` | `'nominal'` |
| `ABL_PEN` | CAV share, 0 to 1; the rest are human drivers | `1` |
| `ABL_SENSE` | `'v2x'` (RSU cannot see human cars), `'all'` (RSU sees them but cannot instruct them) | `'v2x'` |
| `ABL_LAT` | one-way V2X latency [s] | `0` |
| `ABL_LOSS` | packet loss per message, 0 to 1 | `0` |
| `ABL_VREC` | `1` = CAVs follow the RSU speed advice, `0` = ignore it | `1` |
| `ABL_KKMAX` | number of 0.5 s steps | `720` (360 s) |

Examples:

```matlab
watch_main('C1', 105)
watch_main('C3', 1, 'ABL_SCHED', 'scenarioB')
watch_main('C3', 201, 'ABL_SCHED', 'const', 'ABL_DEMAND', '0.7', 'ABL_DRIVER', 'aggressive')
watch_main('C3', 201, 'ABL_SCHED', 'const', 'ABL_PEN', '0.5')    % mixed traffic
```

In mixed traffic (`ABL_PEN` below 1), human cars drive with IDM plus noise, have no V2X, ignore the RSU,
and decide right turns with the C1 gap rule. In the animation they have a thick black outline.
`ABL_PEN` has no effect on C1, where every car is already human.

Pressing F5 on a `main_*.m` script also works (seed 1, live animation), but that **does** save figures
to `output/` and adds a row to `output/ablation_runs.csv`. Use `watch_main` when you only want to look.

## Journal experiments (multi-seed ablation)

Run order, from the MATLAB command window in `main/`:

```matlab
main_human_driving_gpr_train      % 1. train GPR on seeds 1-16, test on held-out seeds 17-20
                                  %    -> simplot/gpr_arrival_model.mat, output/gpr_holdout_metrics.csv
delete ../output/ablation_runs.csv            % 2. start a fresh study (if the file exists)
delete ../output/estimator_runs.csv
run_ablation(101:120, 720, true)  % 3. 20 seeds x all configurations, 360 s each,
                                  %    plus the estimator comparison per seed
make_figures                      % 4. every simulation figure, into output/ (usual names)
```

`make_figures` re-runs one representative seed (the one whose C3 right-turner fuel is closest to the
median) with figures on and the live animation off, without adding rows to the results CSVs, then draws
the multi-seed summary (`ablation_summary.png`, mean and 95% CI per configuration) and the paired
differences (`ablation_paired.png`). `make_figures(107)` uses a seed of your choice.

`run_ablation` writes:

| File | Content |
|---|---|
| `output/ablation_runs.csv` | one row per (seed, configuration): fuel, idle time, completion, throughput, right-turn separation |
| `output/ablation_stats.csv` | per configuration and metric: n, mean, std, 95% CI |
| `output/ablation_pairs.csv` | paired comparisons over seeds: mean difference, 95% CI, Wilcoxon and paired t-test p-values |
| `output/estimator_runs.csv` | per seed and estimator: RMSE, MAE, bias, share within 2 s / 5 s, failure rate, on each estimator's own samples and on the common sample set |
| `output/ablation_errors.log` | any run that failed, with the error |
| `output/ablation_summary.png`, `output/ablation_paired.png` | multi-seed summary and paired-difference figures (from `make_figures`) |

Settings come from `function/abl_settings.m` (environment variables `ABL_*`, listed in that file and in
[How to run everything](#how-to-run-everything-step-by-step)). The runners regenerate
`simplot/traffic_schedule.mat` for every seed; for a hand run with F5, run `main_human_driving.m` first.

Changes made for the journal version:

* All configurations, including the baselines, use the east-west corridor only, so every row of the
  ablation sees the same vehicles.
* The horizon is 720 steps (360 s), so the heavy demand stage (from 300 s) is reached.
* Fuel and idle averages use complete trips only (vehicles that left the model); vehicles still inside at
  the end are counted separately.
* RSU slot times are aged since the optimisation ran, so a slot "5 s from now" counts down.
* Vehicles follow the RSU recommended speed (`Car.EffVd`, `RSU.YieldAdvisory`): right-turners slow down to
  arrive at their slot, and opposing vehicles delayed by a slot get a yield advisory.
* A right-turner is released only if no opposing through vehicle is inside the junction or predicted to
  reach its stop line within 2 s (`RSU.OpposingClear`).
* The GPR test metrics use whole held-out seeds instead of samples drawn from the training set.
* `compute_turn_safety` reports the time separation between each right turn and the nearest opposing
  through vehicle at the junction centre.

## Extra studies: illustrative scenarios and stress tests

These come on top of the main ablation and write to their own folders, so nothing is overwritten.

```matlab
cd main
run_scenarios          % 2 fixed scenarios, about 10 runs        -> output/scenarios/
run_sweeps             % 7 stress-test studies, 10 seeds each    -> output/sweeps/
run_sweeps('demand')   % or one study at a time
plot_sweeps            % redraw the sweep figures and tables at any time
```

`run_sweeps` is **resumable**: stop MATLAB at any point and call `run_sweeps` again, it skips every run
already in `output/sweeps/sweep_runs.csv`. Delete that file (and `sweep_estimator.csv`) to start over.
Seeds 201-210 are used, disjoint from GPR training (1-20) and the main ablation (101-120).

| Study | Swept setting | Points | Configurations | Runs |
|---|---|---|---|---|
| `demand` | constant east-west demand 0.1 to 0.8 veh/s | 8 | C1, C2, C3I, C3 + estimator | 400 |
| `rtshare` | right-turn share 3.75% to 52.5% of arrivals | 6 | C1, C2, C3 | 180 |
| `latency` | one-way V2X latency 0 to 2 s | 5 | C2, C3 (C1 reference) | 110 |
| `loss` | packet loss 0 to 60% per message | 5 | C2, C3 (C1 reference) | 110 |
| `penetration` | CAV share 0 to 100% (rest human drivers) | 5 | C3 (C1 reference) | 60 |
| `driver` | aggressive / nominal / cautious drivers | 3 | C1, C3I, C3 + estimator | 120 |
| `asym` | westbound share of demand 50% to 80% | 3 | C1, C2, C3 | 90 |

Definitions are in `function/sweep_definitions.m`; traffic generation in `function/make_traffic_schedule.m`;
the V2X channel model in `function/v2x_channel.m`; mixed traffic and the two-tier right-turn decision in
`Car.IsCAV` and `function/rt_decision.m`.

Outputs in `output/sweeps/`:

| File | Content |
|---|---|
| `sweep_<study>.png` | 8 metrics against the swept setting, mean and 95% CI per configuration |
| `sweep_demand_estimator.png`, `sweep_driver_estimator.png` | estimator RMSE and failure rate |
| `sweep_stats.csv` | every mean and CI behind the figures |
| `sweep_limits.csv` | first point where each system fails: completion < 90%, gap wait > 120 s, spillback to the entry, turn separation < 2 s |

Outputs in `output/scenarios/scenarioA/` and `scenarioB/`: the usual figure names (`EW_human.png`,
`EW_idm_rsu.png`, `EW_mpc_rsu_gpr.png`, `fuel_*.png`, ...) plus `scenario_overview.png` (position and speed
of every vehicle over time for C1, C2 and C3).

Behaviour changes that also apply to the main ablation:

* A right-turner without a fresh RSU advisory (lost, older than 1 s, or no plan) now decides with the C1
  gap-acceptance rule instead of waiting for the end of green (`rt_decision.m`), the same two-tier rule as
  the testbed.
* New result columns: mean delay, blocked arrivals at the entry (spillback), QP-MPC fallback rate.
  An older results CSV is renamed `*_oldformat.csv` automatically.

## Related repositories

* [Traffic_Intersection_Simulation_5G](https://github.com/Phearamony/Traffic_Intersection_Simulation_5G):
  extends this simulation to send real vehicle data over a network, as a step toward real-world testing
  over 5G.

## Citation

```bibtex
@inproceedings{phan2026rightturn,
  title     = {Learning-Based Optimal Right-Turn Coordination System for Connected and Automated Vehicles at Intersections},
  author    = {Phan, Phearamony and Hasan, Mahmudul and Bakibillah, A. S. M. and Kamal, Md Abdus Samad and Yamada, Kou},
  booktitle = {SICE Festival 2026 (SICE Annual Conference)},
  address   = {Yokohama, Japan},
  year      = {2026}
}
```

## Acknowledgment

Supported by JSPS Grants-in-Aid for Scientific Research (C) 26K07547 and the Rotary Yoneyama Memorial
Foundation Scholarship.
