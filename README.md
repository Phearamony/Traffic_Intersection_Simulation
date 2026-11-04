# Learning-Based Optimal Right-Turn Coordination for Connected and Automated Vehicles

MATLAB simulation for my master's research at **Gunma University (Kamal Laboratory)**. It studies how
Connected and Automated Vehicles (CAVs) can turn right efficiently and safely at signalized intersections
in **left-hand traffic** (Japan).

📄 **Conference paper:** [Learning-Based Optimal Right-Turn Coordination System for Connected and Automated Vehicles at Intersections](https://drive.google.com/file/d/1h1G11o44SiFWYravJFsgIG5Vy24xO_ma/view?usp=drivesdk), Proceedings of the SICE Festival 2026 with Annual Conference, Yokohama, Japan, Sep. 14-17, 2026, pp. 665-670.
*Phearamony Phan, Mahmudul Hasan, A. S. M. Bakibillah, Md Abdus Samad Kamal, Kou Yamada*.

This branch is the **journal version (v2)**: a single simulation engine for every configuration, real
turning paths, a physics-informed GPR with uncertainty and online learning, an arrival-time MPC, and a
**cooperative leading right turn** inside the RSU slot optimizer (the proposed method, `C4_CLTP`).
The code and results of the conference paper are kept in `sice/`.

### Results at a glance

20 traffic seeds (101-120), 360 s each, every configuration on the same traffic (MATLAB, `output/2_main_study/`).

| Configuration | Trips per run | Mean delay [s] | Fuel [L/100 km] | Right-turner gap wait [s] | Turn conflicts |
|---|---|---|---|---|---|
| C1 human drivers | 61.6 | 75.9 | 13.51 | 12.5 | 0.4 |
| C2 method of [5] | 60.7 | 78.4 | 13.63 | 11.0 | 0 |
| C3 SICE paper | 64.5 | 72.7 | 12.43 | 12.0 | 0 |
| V2V_MPC (no RSU) | 68.1 | 69.8 | 11.90 | 10.4 | 0 |
| C4 (without lead turn) | 68.6 | 69.7 | 11.82 | 11.2 | 0 |
| **C4_CLTP (proposed)** | **77.2** | **59.7** | **10.67** | **1.1** | **0** |
| C4_LT (standing holds only) | 71.0 | 66.2 | 11.48 | 7.1 | 0 |
| C4_LTR (C4_LT, rule decides) | 71.6 | 65.5 | 11.38 | 7.6 | 0 |
| C4_CLTR (C4_CLTP, rule decides) | 75.5 | 58.9 | 10.81 | 2.6 | 0 |

C4_CLTP versus C4: +8.6 trips per run (+12.5%, Wilcoxon p = 0.0003), 10.0 s less delay, 9.8% less fuel.
Versus V2V_MPC: +9.1 trips (p = 0.0002). Versus the SICE method C3: +12.7 trips (p = 0.0001).
Allowing only standing CAVs to be held (C4_LT) fixes the two hand-made scenarios but gives up most of the
gain (-6.2 trips against C4_CLTP, p = 0.0009). A local rule on the proposed holds (C4_CLTR: hold when 2 or more vehicles wait behind the
turner) reaches most of the gain: +6.9 trips over C4, 1.7 fewer than C4_CLTP (p = 0.06, not significant); the
cost completes more right turns (59% against 52%, p = 0.001). Stress tests (`output/5_stress_tests/`): the gain holds for
demand 0.2-0.8 veh/s, every driver population, uneven demand, latency up to 1.5 s (1 s is borderline, p = 0.053) and packet loss up to
40%; it needs at least 75% CAVs and fades above about 25% right-turners.

---

## The problem

At many Japanese intersections there is no dedicated right-turn lane or protected arrow. A right-turning
car waits in the junction for a gap in the oncoming traffic, and on a single-lane approach every car
behind it waits too.

## The method (v2)

[5] = M. Hasan, S. Doman, A. S. M. Bakibillah, M. A. S. Kamal, K. Yamada, "An Optimal Right-Turn
Coordination System for Connected and Automated Vehicles at Urban Intersections", Computers, Materials &
Continua, vol. 86, no. 1, pp. 1-17, 2026, doi: 10.32604/cmc.2025.070222; reference [5] of the SICE paper.

This version extends the right-turn coordination of Hasan et al. [5] (one turner, IDM arrival estimate,
oncoming cars may yield without limit, planned once per green) and of the SICE paper (GPR and QP-MPC).
The road is urban: speed limit 50 km/h, drivers' desired speeds 35 to 55 km/h; CAVs keep the limit.

A Road Side Unit (RSU) receives the state of every connected vehicle over V2X and decides, every 0.5 s and
for both approaches together, when each waiting right-turner turns and what every CAV should do to make
that possible. Human drivers have no V2X: the RSU can neither see nor instruct them.

1. **Physics-informed GPR (PI-GPR) arrival-time estimator.** The prior is the IDM forward simulation of
   the queue (the estimator of [5]); a sparse Gaussian process learns its error from
   `[d, v, gap, signal timing, vehicles ahead, speed ahead, right-turners ahead, prior]`. Its uncertainty
   grows with the time to arrival (`sigma = sqrt(latent GP variance + (c0 + c1 tau)^2)`, c0 and c1 fitted on the training data),
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
4. **Cooperative leading right turn** (`C4_CLTP`, the proposed method). When the oncoming side is a
   queue at or rolling into its stop line, no gap can be made by slowing distant cars. The slot optimizer
   may then hold the head CAVs of that queue (up to 25 s) so that the front right-turner goes first. Whether
   and how long to hold is decided by a throughput-priced cost: vehicle-seconds of delay, 60 s for each car
   pushed past the end of the yellow, and the unseen tail of a queue that reaches beyond the RSU's view.
   A hold is released as soon as the plan no longer puts its turner first. The turner still yields: it
   only turns into the gap the held car really leaves.

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
output/       results of this version, one folder per step (see below and output/README.md)
sice/         everything of the conference paper: v1 code, its full results, older files
              (see sice/README.md); new runs never write there
```

## Configurations

| Name | Controller | Coordination | Estimator | RSU message |
|---|---|---|---|---|
| C1 | IDM, human (noise, individual parameters) | none, human gap rule | | |
| V2V_MPC | MPC | V2V, on-board gap check | | |
| C2 | IDM tracking the target time | **method of [5]**: slot search once per green, 4 s gap, turners one after another | IDM forward sim. | target times |
| C3 | MPC | **SICE paper method**: as C2 | GPR | target times |
| C4 | MPC | joint slot optimizer every 0.5 s, gap shaping (later / earlier), uncertainty margins, eco approach | PI-GPR, online | target times |
| C4_NOUNC | as C4, no uncertainty term in the margins | | | |
| C4_NOONLINE | as C4, no online learning | | | |
| C4_SPEED | as C4, speed advice instead of target times | | | |
| C4_STRICT | as C4, existing gaps only (no oncoming CAV is asked to change) | | | |
| C4_SEQ | as C4, turners one after another (the successive search of [5]) | | | |
| C4_NOADV | as C4, oncoming CAVs only asked to arrive later, never earlier | | | |
| C4_ONCE | as C4, planned once per green (as [5]) | | | |
| C4_CLT | as C4 + cooperative leading right turn: queued or slow oncoming CAVs near the stop line become decision variables and may be held (up to 15 s) so the front right-turner goes first | | | |
| **C4_CLTP** | **proposed: as C4_CLT, with a throughput-priced slot cost (vehicle-seconds of delay, a price per vehicle pushed past the end of green, and the unseen queue tail); the cost decides the hold, up to 25 s** | | | |
| C4_LT | as C4_CLTP, but only CAVs already standing in their queue may be held, and in the plan the vehicles behind a vehicle that stands follow at the 2 s queue headway | | | |
| C4_LTR | as C4_LT, but a local rule decides the hold (hold whenever 2 or more vehicles wait behind the turner), not the cost | | | |
| C4_CLTR | as C4_CLTP, but the same local rule decides the hold (the optimizer-versus-rule test of the proposed method) | | | |

Set the configuration by name in `main_one.m` or `main_animation.m`.

Also available (not in the default study): V2V_IDM, C2G (C2 with GPR), C3I (C3 with the IDM estimator),
the factorial of the SICE paper.
C4_GI (gap insertion) is the first version of the lead turn, kept for reference.

Lead-turn holds (C4_CLT, C4_CLTP, C4_LT, C4_LTR, C4_CLTR): a held CAV is released as soon as the plan no longer puts its turner
before it, when the turner commits, or at the hold limit. A held target is never renewed by itself. The
right-turner still yields (art. 37): it only turns into the gap the held CAV leaves.

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
git pull origin lead-turn
```

Then open a script in `main/`, change the settings block at the top if you want, and press **F5**:

| Script | What it does | Time (MATLAB, rough) |
|---|---|---|
| `main_animation.m` | Live animation of the configurations you list (default C1, C2, C3, C4; `'ALL'` for all twelve), one after another on the same traffic. Close the window to skip to the next. Nothing is saved | a few minutes each |
| `main_train.m` | Train the estimators (GPR, PI-GPR) on seeds 1-20. **Run it first, and again after any change to the traffic model** | 15-30 min |
| `main_one.m` | Run one configuration, with or without animation, and save its figures | a few minutes |
| `main_all.m` | Everything for the paper: training, main study (17 configurations x 20 seeds), figures, scenarios, stress tests | about a day; switch parts off in its settings |
| `main_figures.m` | Redraw the figures from results that already exist | 10-20 min |

After `git pull`, start with `main_train.m`, look at one run with `main_animation.m`, then start
`main_all.m` (with `DO_TRAIN = false` if you have just trained).

Results go to `output/`, one folder per step in the order `main_all.m` runs them:

| Folder | Written by | Content |
|---|---|---|
| `1_training/` | `main_train.m` | `estimators.mat` (the trained model the RSU uses), `gpr_holdout_metrics.csv` (accuracy on held-out seeds), `gpr_model_diagnostics.png` |
| `2_main_study/` | `main_all.m` step 2 | `ablation_runs.csv` (one row per seed and configuration), `ablation_stats.csv` (means, 95% CI), `ablation_pairs.csv` (paired comparisons), `estimator_runs.csv`, `ablation_errors.log` |
| `3_figures/` | `main_all.m` step 3, `main_figures.m` | `ablation_summary.png`, `ablation_paired.png`, and for the representative seed `EW_<fig>.png` (trajectories), `EW_<fig>_ac.png` (accelerations), `fuel_<fig>.png`, `estimator_comparison_*.png` |
| `4_scenarios/` | `main_all.m` step 4 | `scenarioA/`, `scenarioB/` (figures and logs of each configuration), `scenario_runs.csv` |
| `5_stress_tests/` | `main_all.m` step 5 | `sweep_runs.csv`, `sweep_<study>.png`, `sweep_stats.csv`, `sweep_limits.csv` |
| `single_runs/` | `main_one.m` | figures and `single_runs.csv` of single runs, kept apart from the study |
| `main_all_log.txt` | `main_all.m` | the console of the whole run |

`main_all.m` with `FRESH = true` deletes the old results of the steps it is about to run (the trained model is
kept when `DO_TRAIN = false`). Every path is set in one place, `function/out_folder.m`.

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
  booktitle = {Proceedings of the SICE Festival 2026 with Annual Conference},
  address   = {Yokohama, Japan},
  pages     = {665--670},
  year      = {2026}
}
```

## Acknowledgment

Supported by JSPS Grants-in-Aid for Scientific Research (C) 26K07547 and the Rotary Yoneyama Memorial
Foundation Scholarship.
