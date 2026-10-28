# Learning-Based Optimal Right-Turn Coordination for Connected and Automated Vehicles

MATLAB simulation for my master's research at **Gunma University (Kamal Laboratory)**. It studies how
Connected and Automated Vehicles (CAVs) can turn right efficiently and safely at signalized intersections
in **left-hand traffic** (Japan).

📄 **Conference paper:** [Learning-Based Optimal Right-Turn Coordination System for Connected and Automated Vehicles at Intersections](https://drive.google.com/file/d/1eD-5lxTvvwlGY6lJCwVpVNrC4kos1YCi/view?usp=sharing)
*Phearamony Phan, Mahmudul Hasan, A. S. M. Bakibillah, Md Abdus Samad Kamal, Kou Yamada*.
SICE Festival 2026 (SICE Annual Conference), Yokohama, Japan.

This branch is the **journal version (v2)**: a single simulation engine for every configuration, real
turning paths, a physics-informed GPR with uncertainty and online learning, and an arrival-time MPC.
The code of the conference paper is kept in `archive/v1/`.

---

## The problem

At many Japanese intersections there is no dedicated right-turn lane or protected arrow. A right-turning
car waits in the junction for a gap in the oncoming traffic, and on a single-lane approach every car
behind it waits too.

## The method (v2)

A Road Side Unit (RSU) receives the state of every connected vehicle over V2X and plans a slot for each
right-turner in the oncoming stream.

1. **Physics-informed GPR (PI-GPR) arrival-time estimator.** The prior is the IDM forward simulation of
   the queue (the conference estimator); a sparse Gaussian process learns its error from
   `[d, v, gap, signal timing, vehicles ahead, speed ahead, right-turners ahead, prior]`. It gives a
   prediction and its uncertainty, and keeps learning online from every arrival the RSU observes.
2. **Uncertainty-aware slot search.** The conference slot search (insert the turner before oncoming
   vehicle *p*, minimise the summed squared delay, exhaustive over *p*), with safety margins computed from
   the real conflict-zone geometry plus a post-encroachment margin plus `z * sqrt(sigma_turner^2 + sigma_oncoming^2)`.
3. **Arrival-time MPC.** The RSU sends target arrival times (not speeds). Each CAV solves a QP-MPC with a
   10 s horizon, jerk penalty, curve speed limits, signal timing, and the constraint "do not reach this
   point before the target time". A right-turner with a slot rolls through the turn without stopping.

All vehicles drive along real lane paths (straight, left arc, right arc), so turns are continuous and the
safety measure is the post-encroachment time (PET) at the conflict zones.

---

## Repository structure

```
sim/          the simulation engine (one engine for every configuration)
  sim_run.m        main loop: signals, arrivals, entry queue, leaders, V2X + RSU, decisions,
                   control, integration, logging, live animation
  sim_config.m     the switches that define each configuration (C1 ... C4)
  sim_main.m       run one configuration with the ABL_* settings, write metrics and figures
  geom_*.m         lane paths, arcs, conflict zones, waiting points
  ctrl_idm.m, ctrl_mpc.m, qp_solve.m      controllers
  turn_gap_ok.m    right-turn gap acceptance (human rule, on-board CAV check)
  rsu_*.m          RSU: received states, estimators, slot planning, online learning
  est_*.m, gp_*.m  estimators: IDM forward simulation, queue physics, sparse GP
  metrics_*.m      metrics of a run (fuel, delay, PET, jerk, ...) and the results CSV
  draw_intersection.m   animation frame (rotated vehicles on their paths)
function/     settings, traffic schedule, statistics and figure functions
main/         entry points and runners (see below)
simplot/      estimators.mat (trained by train_estimators)
output/       figures, results CSVs
archive/      v1 code of the conference paper and older files (see archive/README.md)
```

## Configurations

| Name | Script | Controller | Coordination | Estimator | RSU message |
|---|---|---|---|---|---|
| C1 | `main_human_driving.m` | IDM, human (noise, individual parameters) | none, human gap rule | | |
| V2V_IDM | `main_v2v_idm.m` | IDM | V2V, on-board gap check | | |
| V2V_MPC | `main_v2v_mpc.m` | MPC | V2V, on-board gap check | | |
| C2 | `main_v2v_idm_rsu.m` | IDM | RSU slots | IDM forward sim. | speed |
| C2G | `main_v2v_idm_rsu_gpr.m` | IDM | RSU slots | GPR | speed |
| C3I | `main_v2v_mpc_rsu.m` | MPC | RSU slots | IDM forward sim. | speed |
| C3 | `main_v2v_mpc_rsu_gpr.m` | MPC | RSU slots | GPR | speed (conference method) |
| **C4** | `main_proposed.m` | **MPC** | **RSU slots + uncertainty margins** | **PI-GPR, online** | **arrival time** |
| C4_NOUNC | | as C4, fixed margins | | | |
| C4_NOONLINE | | as C4, no online learning | | | |
| C4_SPEED | | as C4, speed messages | | | |

Every configuration shares the same roads, traffic, driver parameters, signal plan, spawning and metrics;
only the switches in `sim/sim_config.m` differ.

---

## How to run everything (step by step)

All commands go in the MATLAB Command Window (MATLAB with the Optimization Toolbox for `quadprog`
and the Statistics and Machine Learning Toolbox for the statistics). Close MATLAB, the `output` folder
in Explorer, and pause OneDrive (if the repo is inside it) before pulling.

```
git pull origin main
```

```matlab
cd main
run_all          % everything below, unattended (log in output/run_all_log.txt)
```

or step by step:

| Step | Command | What it does | Output |
|---|---|---|---|
| 0 (optional) | `watch_main('C4')` | Live animation of one configuration, nothing saved | none |
| 1 | `train_estimators` | Collect data (seeds 1-20, human drivers) and fit the queue physics, GPR and PI-GPR | `simplot/estimators.mat`, `output/gpr_holdout_metrics.csv`, `output/gpr_model_diagnostics.png` |
| 2 | fresh start (below) | Remove old results | |
| 3 | `run_ablation(101:105)` | Quick test: 5 seeds x 11 configurations | rows in `output/ablation_runs.csv` |
| 4 | `run_ablation(101:120, 720, true)` | Main ablation, 20 seeds + estimator comparison | `ablation_runs.csv`, `ablation_stats.csv`, `ablation_pairs.csv`, `estimator_runs.csv` |
| 5 | `make_figures` | Per-run figures of a typical seed + multi-seed summary | `EW_*.png`, `EW_*_ac.png`, `fuel_*.png`, `estimator_comparison_*.png`, `ablation_summary.png`, `ablation_paired.png` |
| 6 | `run_scenarios` | Illustrative scenarios A and B | `output/scenarios/` |
| 7 | `run_sweeps` | Stress tests, seeds 201-210 (resumable: Ctrl+C, run again to continue) | `output/sweeps/` |
| 8 | `plot_sweeps` | Sweep figures and tables | `sweep_<study>.png`, `sweep_stats.csv`, `sweep_limits.csv` |

**Fresh start** (after a code update), from `main/`:

```matlab
delete ../output/ablation_runs.csv
delete ../output/estimator_runs.csv
rmdir('../output/scenarios', 's')
rmdir('../output/sweeps', 's')
```

**Stress tests one at a time:** `run_sweeps('demand')`, `'rtshare'`, `'latency'`, `'loss'`,
`'penetration'` (mixed traffic), `'driver'` (drivers the estimators were not trained on), `'asym'`.

**If something fails:** runners do not stop on an error; they log it in `output/ablation_errors.log`
or `output/sweeps/sweep_errors.log` and continue.

### Function inputs

| Call | Inputs | Defaults |
|---|---|---|
| `watch_main(name, seed, 'ABL_...', 'value', ..., 'VIEW', m)` | `name`: C1, V2V_IDM, V2V_MPC, C2, C2G, C3I, C3, C4, C4_NOUNC, C4_NOONLINE, C4_SPEED (or an old script name). `VIEW`: half-width of the view in metres | `'C4'`, `101`, 60 m |
| `train_estimators(seeds, n_test, kkmax)` | training seeds, how many of the last ones are held out, steps | `1:20`, `4`, `720` |
| `run_ablation(seeds, kkmax, with_estimator, cfgs)` | seed list, steps, `true`/`false`, cell list of names | `101:120`, `720`, `false`, all 11 |
| `run_sweeps(studies, seeds)` | one study name, a cell list, or `[]` for all | all studies, `201:210` |
| `run_scenarios(names, cfgs)` | `'scenarioA'`, `'scenarioB'` or a cell list | both; C1, C2, C3, C4 |
| `make_figures(seed, kkmax, cfgs)` | seed of the per-run figures | typical seed, `720` |
| `run_all(fresh, train)` | `false` keeps results; `train=false` keeps the trained estimators | `true`, `true` |

### Settings (`function/abl_settings.m`)

| Setting | Values | Default |
|---|---|---|
| `ABL_SCHED` | `'staged'` (light, medium, heavy), `'const'`, `'scenarioA'`, `'scenarioB'` | `'staged'` |
| `ABL_DEMAND` | east-west flow [veh/s], with `'const'` | `0.4` |
| `ABL_RT` | P(right turn given not left), 0 to 1 | `0.15` |
| `ABL_ASYM` | westbound share of east-west flow | `0.5` |
| `ABL_DRIVER` | `'nominal'`, `'aggressive'`, `'cautious'` | `'nominal'` |
| `ABL_PEN` | CAV share, 0 to 1; the rest are human drivers | `1` |
| `ABL_SENSE` | `'v2x'` (RSU cannot see human cars), `'all'` (RSU sees them, cannot instruct them) | `'v2x'` |
| `ABL_LAT` | one-way V2X latency [s] | `0` |
| `ABL_LOSS` | packet loss per message, 0 to 1 | `0` |
| `ABL_KKMAX` | number of 0.5 s steps | `720` (360 s) |

Examples:

```matlab
watch_main('C1', 105)
watch_main('C4', 1, 'ABL_SCHED', 'scenarioB', 'VIEW', 40)
watch_main('C4', 201, 'ABL_SCHED', 'const', 'ABL_DEMAND', '0.6', 'ABL_DRIVER', 'aggressive')
watch_main('C4', 201, 'ABL_SCHED', 'const', 'ABL_PEN', '0.5')    % mixed traffic
```

In the animation, cars are drawn at their real position and heading; right-turners are purple, left-turners
grey; human drivers have a thick outline; a right-turner waiting for its gap has a yellow dot (cyan once the
RSU has given it a slot).

Running a `main_*.m` script with F5 also works (seed 1, live animation) but **does** save figures to
`output/` and add a row to `output/ablation_runs.csv` (with an empty study name, so `summarize_ablation`
ignores it once `run_ablation` rows exist). Use `watch_main` to only look.

---

## Model details

* **Geometry.** One lane per direction, lane width 3 m, stop line 10 m before the centre, entry 150 m
  before and exit 170 m after the centre. Right turns: arc of radius 6 m, passing inside the centre
  (Japanese rule), so a mutual right turn is not a conflict. Left turns: radius 4.5 m. Arc speed limited by
  a lateral acceleration of 3 m/s^2. Conflict zones from the path geometry (centre lines closer than 2.6 m).
* **Drivers.** Every vehicle draws its own desired speed, time headway, IDM acceleration, comfortable
  deceleration and standstill gap; human drivers also draw a right-turn gap margin and have acceleration
  noise. The aggressive and cautious populations shift all of these.
* **Right-turn decision.** Human: oncoming vehicles judged at constant speed, margin = 0.7 s + own gap
  margin. CAV: oncoming vehicles predicted with their current acceleration (from V2V), a vehicle at rest
  may start, a vehicle held by an RSU target arrives no earlier than its target, margin = PET 1.5 s. A
  vehicle standing at its waiting point blocks its lane, which makes mutual right turns possible. Vehicles
  yield to a turner already in the junction when they can stop comfortably.
* **Entry queue.** An arrival that cannot enter (the lane is full at the entry) waits and enters later; its
  waiting time counts in its delay. Every configuration therefore serves the same demand.
* **Metrics** (`sim/metrics_compute.m`): fuel per right-turner and per straight vehicle (complete trips),
  fuel of all vehicles per distance, gap and red waiting time, PET of right turns (minimum and mean), turns
  with PET < 1.5 s, overlapping occupancies, delay from arrival, blocked arrivals, RMS jerk, stops per
  vehicle, QP failures.
* **Estimators** are trained and scored on snapshots during the green (or yellow) of the vehicle's approach,
  for arrivals within that green, which is what the RSU plans with.

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
