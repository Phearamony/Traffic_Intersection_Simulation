# Learning-Based Optimal Right-Turn Coordination for Connected and Automated Vehicles

MATLAB simulation for my master's research at **Gunma University (Kamal Laboratory)**. It studies how
Connected and Automated Vehicles (CAVs) can turn right efficiently and safely at signalized intersections
in **left-hand traffic** (Japan).

📄 **Conference paper:** [Learning-Based Optimal Right-Turn Coordination System for Connected and Automated Vehicles at Intersections](https://drive.google.com/file/d/1eD-5lxTvvwlGY6lJCwVpVNrC4kos1YCi/view?usp=sharing)
*Phearamony Phan, Mahmudul Hasan, A. S. M. Bakibillah, Md Abdus Samad Kamal, Kou Yamada*.
SICE Festival 2026 (SICE Annual Conference), Yokohama, Japan.

This branch is the **journal version (v2)**: a single simulation engine for every configuration, real
turning paths, a physics-informed GPR with uncertainty and online learning, and an arrival-time MPC.
The code and results of the conference paper are kept in `cise/`.

---

## The problem

At many Japanese intersections there is no dedicated right-turn lane or protected arrow. A right-turning
car waits in the junction for a gap in the oncoming traffic, and on a single-lane approach every car
behind it waits too.

## The method (v2)

[5] = M. Hasan, S. Doman, A. S. M. Bakibillah, M. A. S. Kamal, K. Yamada, "An Optimal Right-Turn
Coordination System for Connected and Automated Vehicles at Urban Intersections", submitted to Computers,
Materials & Continua (version of October 21, 2025); reference [5] of the CISE paper.

This version extends the right-turn coordination of Hasan et al. [5] (one turner, IDM arrival estimate,
oncoming cars may yield without limit, planned once per green) and of the CISE paper (GPR and QP-MPC).
The road is urban: speed limit 50 km/h, drivers' desired speeds 35 to 55 km/h; CAVs keep the limit.

A Road Side Unit (RSU) receives the state of every connected vehicle over V2X and decides, every 0.5 s and
for both approaches together, when each waiting right-turner turns and what every CAV should do to make
that possible. Human drivers have no V2X: the RSU can neither see nor instruct them.

1. **Physics-informed GPR (PI-GPR) arrival-time estimator.** The prior is the IDM forward simulation of
   the queue (the estimator of [5]); a sparse Gaussian process learns its error from
   `[d, v, gap, signal timing, vehicles ahead, speed ahead, right-turners ahead, prior]`. Its uncertainty
   grows with the time to arrival (`sigma = sqrt(GP variance + (c1 tau)^2)`, fitted on the training data),
   and it keeps learning online from every arrival the RSU observes.
2. **Joint gap-shaping slot optimizer** (`sim/rsu_plan.m`). The slot search of [5] (insert the turner
   before oncoming vehicle *p*), extended to every waiting turner of both approaches at once. For each
   candidate assignment the RSU computes the smallest legal changes that make it work and keeps the one
   with the least total delay. The right-turner always yields (Road Traffic Act, art. 37): its on-board
   check only lets it turn into a gap that exists. To make a gap the RSU may ask oncoming CAVs to arrive a
   little later (at most 2 s, never below 0.7 of their speed, about 1 m/s^2, no new command within 40 m of
   the stop line) or, for the car before the gap, a little earlier (up to the speed limit). Safety margins
   come from the conflict-zone geometry, a post-encroachment margin and `z * sigma` of the vehicles the RSU
   does not control.
3. **Arrival-time MPC and eco approach.** The RSU sends target times (not speeds) to every CAV it plans
   for: the turner (when to go), oncoming CAVs (shaped or pulled forward), and CAVs that have to wait
   anyway (behind a waiting turner), so they slow early instead of stopping late. Each CAV solves a QP-MPC
   with a 10 s horizon, jerk penalty, curve speed limits, signal timing and the constraint "do not reach
   this point before the target time". A held CAV cannot arrive early, so the RSU needs smaller margins for
   the vehicles it commands.

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
function/     settings, traffic schedule, runners (study, sweeps, scenarios, training),
              statistics and figure functions
main/         the scripts you run (F5), see below
simplot/      estimators.mat (written by main_train.m)
output/       results of this version
cise/         everything of the conference paper: v1 code, its full results, older files
              (see cise/README.md); new runs never write there
```

## Configurations

| Name | Controller | Coordination | Estimator | RSU message |
|---|---|---|---|---|
| C1 | IDM, human (noise, individual parameters) | none, human gap rule | | |
| V2V_MPC | MPC | V2V, on-board gap check | | |
| C2 | IDM tracking the target time | **method of [5]**: slot search once per green, 4 s gap, turners one after another | IDM forward sim. | target times |
| C3 | MPC | **CISE method**: as C2 | GPR | target times |
| **C4** | **MPC** | **joint slot optimizer every 0.5 s, gap shaping (later / earlier), uncertainty margins, eco approach** | **PI-GPR, online** | **target times** |
| C4_NOUNC | as C4, no uncertainty term in the margins | | | |
| C4_NOONLINE | as C4, no online learning | | | |
| C4_SPEED | as C4, speed advice instead of target times | | | |
| C4_STRICT | as C4, existing gaps only (no oncoming CAV is asked to change) | | | |
| C4_SEQ | as C4, turners one after another (the successive search of [5]) | | | |
| C4_NOADV | as C4, oncoming CAVs only asked to arrive later, never earlier | | | |
| C4_ONCE | as C4, planned once per green (as [5]) | | | |

Set the configuration by name in `main_one.m` or `main_animation.m`.

Also available (not in the default study): V2V_IDM, C2G (C2 with GPR), C3I (C3 with the IDM estimator),
the factorial of the CISE paper.

Right-turn priority: every configuration follows the Road Traffic Act (art. 37). The right-turner yields:
its on-board check only lets it turn into a gap that exists. Every RSU configuration, including C2 and C3,
uses the same limits on how much an oncoming CAV may be asked to change (`ABL_DMAX` = 2 s and the comfort
limits above). [5] itself had no such limit; with it, the comparison is fair.

Every configuration shares the same roads, traffic, driver parameters, signal plan, spawning and metrics;
only the switches in `sim/sim_config.m` differ.

---

## How to run

Requirements: MATLAB with the Optimization Toolbox (`quadprog`) and the Statistics and Machine Learning
Toolbox (statistics). Close MATLAB and pause OneDrive (if the repo is inside it) before pulling:

```
git pull origin main
```

Then open a script in `main/`, change the settings block at the top if you want, and press **F5**:

| Script | What it does | Time (MATLAB, rough) |
|---|---|---|
| `main_animation.m` | Live animation of one configuration. Nothing is saved | a few minutes |
| `main_train.m` | Train the estimators (GPR, PI-GPR) on seeds 1-20. **Run it first, and again after any change to the traffic model** | 15-30 min |
| `main_one.m` | Run one configuration, with or without animation, and save its figures | a few minutes |
| `main_all.m` | Everything for the paper: training, main study (12 configurations x 20 seeds), figures, scenarios, stress tests | about a day; switch parts off in its settings |
| `main_figures.m` | Redraw the figures from results that already exist | 10-20 min |

After `git pull`, start with `main_train.m`, look at one run with `main_animation.m`, then start
`main_all.m` (with `DO_TRAIN = false` if you have just trained).

Results go to `output/`:

| File | Content |
|---|---|
| `ablation_runs.csv` | one row per seed and configuration (all metrics) |
| `ablation_stats.csv`, `ablation_pairs.csv` | means with 95% CI, paired comparisons with C4 |
| `ablation_summary.png`, `ablation_paired.png` | the study figures |
| `EW_<fig>.png`, `EW_<fig>_ac.png`, `fuel_<fig>.png` | trajectories, accelerations, fuel of one run |
| `estimator_runs.csv`, `estimator_comparison_*.png`, `gpr_holdout_metrics.csv`, `gpr_model_diagnostics.png` | estimator accuracy |
| `scenarios/`, `sweeps/` | scenarios A and B, stress tests |
| `run_all_log.txt`, `ablation_errors.log` | console log and failed runs (a failed run does not stop the study) |

The functions behind the scripts (in `function/`) can also be called directly, for example
`run_ablation(101:105)` for a quick 5-seed study, `run_sweeps('penetration')` for one stress test (resumable:
stop with Ctrl+C and call it again), or `watch_main('C4', 1, 'ABL_SCHED', 'scenarioB', 'VIEW', 40)`.

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
| `ABL_DMAX` | largest delay the RSU may ask of one oncoming CAV [s] | `2` |

In the animation, cars are drawn at their real position and heading; right-turners are purple, left-turners
gray; human drivers have a thick outline; a right-turner waiting for its gap has a yellow dot (cyan once the
RSU has given it a slot).

---

## Model details

* **Geometry.** One lane per direction, lane width 3 m, stop line 10 m before the center, entry 150 m
  before and exit 170 m after the center. Right turns: arc of radius 6 m, passing inside the center
  (Japanese rule), so a mutual right turn is not a conflict. Left turns: radius 4.5 m. Arc speed limited by
  a lateral acceleration of 3 m/s^2. Conflict zones from the path geometry (center lines closer than 2.6 m).
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
