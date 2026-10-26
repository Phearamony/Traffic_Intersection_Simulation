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

## Key results

A factorial ablation was run with identical stochastic traffic, signal timing and geometry for every
configuration (`output/ablation_summary.csv`):

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
class/      Car.m (IDM / QP-MPC vehicle), RSU.m (V2X buffer, GPR estimator, slot search)
function/   IDM, trajectory / acceleration / fuel plotting, ablation metrics
main/       Simulation entry points (one script per configuration)
simplot/    Intersection drawing, initial values, trained GPR model, shared traffic schedule
output/     Generated figures, videos and ablation summary
```

### Main scripts

| Script | Configuration |
|---|---|
| `main/main_human_driving.m` | Generates the shared traffic schedule for the current seed (and runs C1) |
| `main/main_human_driving_plot.m` | C1: uncoordinated human driving (IDM + noise), writes the C1 metrics |
| `main/run_ablation.m` | Runs every configuration over many seeds and summarizes the results |
| `main/make_figures.m` | Makes every journal figure after `run_ablation` |
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

Settings come from `function/abl_settings.m` (environment variables `ABL_SEED`, `ABL_KKMAX`, `ABL_PLOT`,
`ABL_LIVE`, `ABL_NOLOG`, `ABL_VREC`). Running a single script by hand still works: it uses seed 1, 720 steps and live plots.
**Run `main_human_driving.m` first** whenever you change the seed or the horizon, because it regenerates
`simplot/traffic_schedule.mat`.

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
