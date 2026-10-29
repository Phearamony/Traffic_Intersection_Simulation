# Review guide: traffic simulation v2 (code and math)

Code state: branch `v2-engine`, commit `e3d786a` (final code of this round), which follows `979b6c2` (safe-side oncoming bounds), `b3dfe15` (second review round), `70d8d01` ("Fixes from the code review") and `0b64b16` (CAV gap check keeps PET after a vehicle has just cleared; once-per-green planning also plans turners that come into range later). All line numbers refer to `e3d786a`.

## How to use this guide

Read it with the code open next to it, one part at a time, in the order given (it follows the data flow of one time step, then goes deeper into each module). Every equation is followed by its location as `file.m:lines`; open that range and check the equation term by term against the code. Each part ends with "Things to check": assumptions and approximations you should be able to defend in the paper. Items fixed by `70d8d01`, `b3dfe15`, `979b6c2` or `e3d786a` are marked "Fixed in ..." in one line, so you can confirm the fix. Section 10 collects the open questions that could change the conclusions.

Notation used throughout:

- $\Delta t = 0.5$ s (simulation step, `C.DT`), $L = 4$ m (vehicle length), $s$ = path coordinate of the vehicle front (m), $s_{stop} = 140$ m (stop line on every path).
- $T(d, v_0, a, v_{max})$ = `travel_time` (`sim/travel_time.m:3-12`): time to cover $d$ from speed $v_0$ accelerating at $a$ up to $v_{max}$:

$$
v_0 \leftarrow \min(\max(v_0,0), v_{max}),\quad t_{acc} = \frac{v_{max}-v_0}{a},\quad d_{acc} = v_0 t_{acc} + \tfrac12 a t_{acc}^2
$$
$$
T = \begin{cases} 0 & d \le 0\\ d/\max(v_0, 0.1) & a \le 10^{-6}\\ \dfrac{-v_0+\sqrt{v_0^2+2ad}}{a} & d \le d_{acc}\\ t_{acc} + \dfrac{d-d_{acc}}{v_{max}} & \text{otherwise}\end{cases}
$$

The $a \le 10^{-6}$ branch uses the unclipped $v_0$ (line 4 runs before line 5).

Numbers marked "(computed)" were obtained by re-implementing `geom_build` and `travel_time` outside MATLAB; confirm them with one MATLAB call (`G = geom_build(); G.P(G.idx.ER).s_wait`, `G.oppZ{G.idx.ER}`).

---

## 1. Overview and data flow of one time step

### 1.1 Configuration switches: `sim/sim_config.m`

Every configuration runs in the same engine; only these fields differ (defaults at `sim/sim_config.m:44-46`):

| field | values | meaning |
|---|---|---|
| `ctrl` | `idm`, `mpc` | controller of the CAVs |
| `coord` | `none`, `v2v`, `rsu` | `none` = all human (C1); `v2v` = CAVs with on-board gap check; `rsu` = plus RSU planner |
| `est` | `idm`, `gpr`, `pigpr` | RSU arrival estimator |
| `msg` | `speed`, `time` | speed advice or target times |
| `unc_z` | number | $z$ of the uncertainty term in slot margins |
| `online` | 0/1 | PI-GPR online learning |
| `spat` | 0/1 | MPC knows the signal timing |
| `slot` | `opt`, `seq`, `strict` | joint, successive ([5]), existing gaps only |
| `plan` | `rolling`, `once` | every step or once per green |
| `margin` | `geom`, `fixed` | geometric margins or 4 s / 2 s |
| `adv` | 0/1 | RSU may ask CAVs to arrive earlier |
| `eco` | 0/1 | CAVs that must wait get their planned time |

The reference set `ref` (`sim/sim_config.m:47-48`) is `coord=rsu, spat=1, msg=time, slot=seq, plan=once, margin=fixed, eco=1`. Configurations (`sim/sim_config.m:49-96`):

| name | settings |
|---|---|
| C1 | defaults: `coord=none` (all human) |
| V2V_IDM | `coord=v2v` |
| V2V_MPC | `coord=v2v, ctrl=mpc` (note `spat=0`) |
| C2 | `ref`, `est=idm` (so `ctrl=idm`, `unc_z=0`, `adv=0` from the defaults) |
| C2G | `ref`, `est=gpr` |
| C3I | `ref`, `ctrl=mpc`, `est=idm` |
| C3 | `ref`, `ctrl=mpc`, `est=gpr` |
| C4 | `coord=rsu, ctrl=mpc, est=pigpr, spat=1, msg=time, unc_z=1.645, online=1, adv=1, eco=1` (slot `opt`, plan `rolling`, margin `geom` from the defaults) |
| C4_NOUNC / NOONLINE / SPEED / STRICT / SEQ / NOADV / ONCE | C4 with `unc_z=0` / `online=0` / `msg=speed` / `slot=strict` / `slot=seq` / `adv=0` / `plan=once` |

### 1.2 Run settings: `function/abl_settings.m`

All settings come from `ABL_*` environment variables with defaults (`function/abl_settings.m:44-67`): seed 1, `KKmax` 720 (360 s), schedule `staged`, demand 0.40 veh/s (only for `const`), right-turn share `rt` 0.15 (P(right | not left)), `asym` 0.5, driver `nominal`, latency 0 s, loss 0, penetration 1, sense `v2x`, `dmax` 2 s. The latency comment (`function/abl_settings.m:30-32`) now states the model: the RSU extrapolates a received state at its speed, and downlink messages arrive after the latency rounded up to whole 0.5 s steps. `abl_env_reset` clears every variable (`function/abl_env_reset.m:5-12`).

### 1.3 Time convention (since 70d8d01)

`sim/sim_run.m:80-82`: in step `KK`, decisions use the state at time $t = (KK-1)\Delta t$; integration moves it to $t_1 = t + \Delta t$. Everything that decides uses $t$: signals (line 85), arrivals and spawn times (lines 105, 117), RSU uplink time stamps (line 148, `sim/rsu_receive.m:7`), snapshots (`sim/rsu_step.m:11, 36`), `t_now` (line 176), MPC prediction times $t_k = t + k\Delta t$ (`sim/ctrl_mpc.m:51`), go and target comparisons. Everything that records the result of the step uses $t_1$ or interpolates inside $[t, t_1]$: the log (line 355), stop-line crossing $t_{stop} = t + \Delta t\,(s_{stop} - s)/(s^+ - s)$ (line 343), exit time (line 347), guard log (line 308), animation (lines 357-363). The run covers decisions at $0, 0.5, \ldots, 359.5$ s and log times $0.5, \ldots, 360$ s; `R.t_end = KKmax*dt` = 360 s (line 379).

So the training labels $y = t_{stop} - t_{snap}$, the IDM forward simulation, the RSU's go times, the MPC and the metrics now share one clock. Fixed in 70d8d01 (was: logs and crossings 0.5 s earlier than decisions).

### 1.4 One time step: `sim/sim_run.m`

Initialization:

- Traffic schedule with stages at 0, 120, 300 s, NS axis rates 0.30/0.45/0.60 veh/s and EW axis rates 0.25/0.40/0.55 veh/s (`sim/sim_run.m:43-44`). Only E and W are simulated (line 45); NS draws exist but no NS vehicle is created.
- Separate random streams for human acceleration noise (seed $3\cdot10^5+$seed) and channel loss (seed $10^6+$seed) (lines 46-47).
- One straight vehicle per approach at $t=0$, $v=12$ m/s, CAV unless C1 or `pen = 0` (lines 57-66).
- RSU for `coord=rsu`, or for data collection / estimator evaluation (lines 69-72). Channel: latency, loss, sense (line 73).

Loop over `KK = 1..KKmax` (line 79):

1. **Signals** (`sim/sim_run.m:83-87`, `sim/signal_info.m:9-20`). Fixed plan, cycle 120 s. EW green $[60, 114.5)$, yellow $[114.5, 118)$, red otherwise (`sim/geom_constants.m:22-24`). `sigT = [tg0, tend]`: start of the current (or next) green and end of its yellow.
2. **Arrivals** (`sim/sim_run.m:89-107`). A scheduled arrival of index `KK` creates a vehicle in the entry queue with movement, $V_d$, $T$, $a$, $b$, $s_0$, $h_m$. It is a CAV if `coord ~= none` and $U < $ `pen` (line 103). A CAV's desired speed is capped: $V_d \leftarrow \min(V_d, V_{lim})$, $V_{lim} = 50/3.6 = 13.89$ m/s (line 104).
3. **Entry** (`sim/sim_run.m:109-130`). The first queued vehicle enters when the smallest $s$ of any vehicle of that approach is $\ge$ `SPAWN_GAP` $= 8$ m (line 115). Entry speed $V_d$, or $\min(V_d, v_{ahead})$ if that vehicle is within 60 m (lines 119-124). Queue idling fuel at entry: $F \leftarrow f(0,0)\,(t_{spawn}-t_{arr})$ (line 127).
4. **Leaders** (`sim/sim_run.m:137`, `sim/find_leaders.m:31-70`). Bumper gap $g_i$, leader speed $v_{l,i}$, index, acceleration $a_{l,i}$. Approach lane: same approach, leader rear before $\min(s_{div,i}, s_{div,j})$, $g = s_j - L - s_i$ (`find_leaders.m:34-43`). Exit lane: $w = s - s_{out} + w_{out}$ (line 28), $g = w_j - L - w_i$ (lines 62-63); before a merge, $g = (s_j - MZ_{ji}) - L - (s_i - MZ_{ij})$ (lines 56-60). An uncommitted right-turner is never a leader at a merge (line 55).
5. **V2X and RSU** (`sim/sim_run.m:139-173`).
   - Uplink: every CAV (and every vehicle if `sense='all'` or in collect/eval mode) sends its state, dropped with probability `loss` (lines 142-149). `rsu_receive` stores $s - v\,\ell$ with time stamp $t-\ell$ ($\ell$ = latency) (`sim/rsu_receive.m:7`).
   - `rsu_forget` drops vehicles that left and entries older than 5 s (`sim/rsu_forget.m:6-9`).
   - `rsu_step` (line 151): snapshots for learning and `rsu_plan` (parts 5 and 6).
   - Downlink: each message dropped with probability `loss`; delivered at $t + \lceil \ell/\Delta t \rceil \Delta t$ (lines 159-160). On delivery the CAV's `msg` is replaced and `msg_rx = t` (lines 162-172).
6. **Decisions and stop points** (`sim/sim_run.m:175-237`).
   - Signal stop: a vehicle before its stop line (+0.05 m) stops at $s_{stop}$ if red, or if yellow and $s_{stop} - s \ge v^2/(2\cdot3)$ (lines 178-186).
   - Right-turners within 40 m before the stop line and not committed run the gap check (part 4) (lines 187-211); stop point $s_{wait}$ if not ok (line 209).
   - Yielding rule for straight and left vehicles (lines 218-237), part 4.
7. **Control** (`sim/sim_run.m:239-283`). MPC for CAVs when `ctrl=mpc` (part 7); IDM otherwise (part 3). For IDM the stop point becomes a standing leader: if $s_{stop,i} - s_i < g_i$ then $g = s_{stop,i} - s_i$, $v_l = 0$ (lines 243-245).
8. **Integration** (`sim/sim_run.m:285-319`):
$$
v^+ = \max(v + a\Delta t, 0),\qquad
\Delta s = \begin{cases} \dfrac{v^2}{-2a} & v + a\Delta t < 0,\ a<0\\[4pt] \tfrac12 (v + v^+)\Delta t & \text{otherwise}\end{cases}
$$
   Overlap guard, twice (lines 298-312): leaders recomputed at the new positions; if the gap is $< 0.3$ m, $s_i^+ \leftarrow \max(s_i, s_i^+ - (0.3 - g))$ and $v_i^+ \leftarrow \min(v_i^+, v_j^+)$, counted in `count.guard`. Stop guard (lines 313-318): a vehicle that would pass its stop point is placed at $\max(s, s_{stop,i} - 0.05)$ with $v^+=0$, counted in `count.stopguard`. Realized acceleration $a = (v^+ - v)/\Delta t$ (line 319).

   State update (lines 321-351): squared jerk $((a - a_{prev})/\Delta t)^2$ (line 325); fuel $f(v^+, a)\Delta t$ (line 327); a stop is counted when $v^+ < 0.5$ after having been $> 1.0$ (lines 329-341); idle time split into `idle_gap` (right-turner past its stop line, not committed), `idle_red` (before the stop line, not green), `idle_other`; stop-line crossing and exit times (lines 342-350); `rsu_crossed` on crossing (line 344).
9. **Log** `[t1 id s v a path isCAV]` (`sim/sim_run.m:353-356`).

At the end, vehicles still in the entry queue get idle fuel up to 360 s and `entered=false` (`sim/sim_run.m:370-374`).

Helpers: `fresh` = message received within 1 s (`sim/sim_run.m:414-416`); `held` = within 10 s (`sim/sim_run.m:418-421`).

### Things to check (part 1)

- Fixed in b3dfe15: the schedule now uses the decision clock, `tsec = (kk-1)*dt` (`function/make_traffic_schedule.m:104`) and scenario index `kk = round(t/dt) + 1` (`function/make_traffic_schedule.m:150`), so a scenario arrival "at 44 s" joins the queue at $t = 44$ s; the header comment `sim/sim_run.m:12` matches.
- Uplink latency is dead reckoning: `rsu_receive` stores $s - v\ell$ at $t-\ell$ and `rsu_view` extrapolates $s - v\ell + v\ell = s$ (`sim/rsu_view.m:12`). The RSU sees the exact current position and the current $v$, $a$ and `committed`. Only the downlink delay (rounded up to whole steps) and losses degrade it. The comment now says so (`function/abl_settings.m:30-32`) and the sweep uses whole steps ($[0, 0.5, 1, 1.5, 2]$ s, `function/sweep_definitions.m:34`). The paper should describe the latency study as delay of the RSU commands, not of the RSU's information.
- Entry with an 8 m spawn gap at up to $V_d$ (lines 115-124) leaves 4 m of free space; check how often the IDM brakes hard or the overlap guard fires right after entry.
- The overlap guard and the stop guard move vehicles non-physically. `count.stopguard` is not reported in the metrics (`sim/metrics_compute.m:94` reports only `guard`).
- Human noise comes from one stream in vehicle order (lines 276-280), so the same human vehicle gets different noise in different configurations once the vehicle set differs (mixed traffic).
- Only the E-W corridor exists. The right turn also crosses NS paths (zones exist, part 2) but no NS vehicle is simulated; this is valid only because the signal separates them.

---

## 2. Geometry

### 2.1 Constants: `sim/geom_constants.m:3-24`

$\Delta s = 0.25$ m path sampling; entry 150 m before center, exit 170 m after; stop line 10 m before center; lane center offset 1.5 m (3 m lanes); right-turn radius $R_R = 6$ m; left-turn radius $R_L = 4.5$ m; lateral acceleration limit $a_{lat} = 3$ m/s²; $V_{max} = 30$ m/s; $V_{lim} = 13.89$ m/s; $V_{des,0} = 12.8$ m/s (RSU IDM); conflict distance $D_{conf} = 2.6$ m; merge zone length 6 m; waiting gap 0.5 m; $L = 4$ m, width 1.8 m; spawn gap 8 m; $\Delta t = 0.5$ s.

### 2.2 Paths: `sim/geom_build.m`

Paths are built for approach E (travelling $+x$ at $y=+1.5$, left-hand traffic) and rotated by 0, 90, 180, -90 degrees for E, N, W, S (`sim/geom_build.m:39-80`).

- Straight (`sim/geom_build.m:142-152`): $x = -150 + s$, $y = 1.5$, $s \in [0, 320]$, $\kappa = 0$, $s_{div} = 320$, $s_{out} = 150$, $w_{out} = 0$.
- Turn (`sim/geom_build.m:154-194`): line, quarter circle, line. Right: arc start $x_0 = 1.5 - R_R = -4.5$, center $(-4.5, -4.5)$, angle $\pi/2 \to 0$ clockwise. Left: $x_0 = -1.5 - R_L = -6$, center $(-6, 6)$, angle $-\pi/2 \to 0$. With $L_1 = x_0 + 150$ (approach line) and $L_a = R\pi/2$ (arc):
$$
\text{arc: } \theta = \theta_0 \pm \frac{s - L_1}{R},\ x = c_x + R\cos\theta,\ y = c_y + R\sin\theta,\ \kappa = 1/R
$$
  $s_{div} = L_1$ (leaves the approach lane at the arc start), $s_{out} = L_1 + L_a$, $w_{out} = |y_{end}|$.
- (computed) ER: $L_1 = 145.5$, arc to 154.92, length 320.25 m, $w_{out} = 4.5$. EL: $L_1 = 144$, arc to 151.07, length 315 m, $w_{out} = 6$.
- Curve speed (`sim/geom_build.m:73-74`): $v_{curve}(s) = \min(\sqrt{a_{lat}/\max(|\kappa|, 10^{-6})}, V_{max})$: right arc $\sqrt{3\cdot6} = 4.243$ m/s, left arc $\sqrt{3\cdot4.5} = 3.674$ m/s, 30 m/s on lines. Curvature jumps from 0 to $1/R$ (no transition curve).
- Exit direction (`sim/geom_build.m:219-228`): straight keeps its direction, left is counter-clockwise, right is clockwise in the order E, N, W, S. ER and WL both exit to S (a merge); ER and WS cross.
- Pose at $s$ by linear interpolation (`sim/geom_pose.m:3-9`).

### 2.3 Conflict and merge zones (`sim/geom_build.m:82-94`, `196-217`)

For every pair of paths from different approaches, except opposite right turns (they pass each other on the inside, lines 87-89), sample points with $|x|,|y| < 15$ m are compared; close pairs have $\|p_i(s) - p_j(s')\| < D_{conf}$.

- Different exit roads ("cross"): $z_p = [\min s_i, \max s_i]$, $z_q = [\min s_j, \max s_j]$ over the close pairs.
- Same exit road ("merge"): $z_p = [\min s_i, \min s_i + 6]$, $z_q = [\min s_j, \min s_j + 6]$.

A vehicle occupies a zone from the time its front reaches $z_0$ until its rear passes $z_1$ (front at $z_1 + L$).

Waiting point (`sim/geom_build.m:99-107`): $s_{wait} = z_0(\text{turn vs opposite straight}) - 0.5$.

Rows of each right turn against the opposite approach (`sim/geom_build.m:111-125`): for turn path $k$ and every opposite straight or left path $q$, one row $[q,\ z_{0k},\ z_{1k},\ z_{0q},\ z_{1q}]$. Merge-zone starts `MZ(p,q)` (lines 129-135).

(computed) Zones of ER (WR is symmetric):

| pair | type | zone on ER [m] | zone on other [m] |
|---|---|---|---|
| ER / WS | cross | [147.75, 154.50] | [146.50, 152.75] |
| ER / WL | merge | [151.75, 157.75] | [146.50, 152.50] |
| ER / NS | cross | [146.00, 152.50] | [147.25, 153.50] |
| ER / NR | cross | [148.00, 153.50] | [147.00, 152.25] |
| ER / SR | cross | [147.00, 152.25] | [148.00, 153.50] |
| ER / SS | merge | [149.25, 155.25] | [148.50, 154.50] |

The last four are perpendicular movements, separated by the signal and never populated. So $s_{wait} = 147.25$ m on ER (7.25 m past the stop line), and `oppZ{ER}` holds the rows for WS and WL. The WL arc starts at $s = 144$, so its zone with ER lies on its arc.

### Things to check (part 2)

- $D_{conf} = 2.6$ m between path center points is the only collision geometry. The body is 4 m x 1.8 m and $s$ is the front; the rear is covered by $z_1 + L$, but at oblique angles the corners can overlap at center distances above 2.6 m. Check the zone against a body-overlap test once for ER/WS and the merge.
- The merge zone is a fixed 6 m from first contact; afterwards car following takes over (`find_leaders`). Check that a 6 m zone with PET 1.5 s is the merge safety definition you want.
- No transition curve: the curve speed drops from 30 to 4.24 m/s at one sample. `curve_cap` and the MPC use look-ahead, but jerk at the arc start depends on it.
- Fixed in 70d8d01: the header no longer documents a field `s_ff` (`sim/geom_build.m:33`); `t_ff_unit` is still created as NaN and unused (line 76).

---

## 3. Drivers and traffic

### 3.1 Schedule: `function/make_traffic_schedule.m`

The global stream is seeded with `rng(S.seed)` (`sim/sim_run.m:43`). For each step index $k$ and approach N, S, E, W (in this order) (`function/make_traffic_schedule.m:103-124`):

- Rates: staged $\lambda_{EW} \in \{0.25, 0.40, 0.55\}$ for $(k-1)\Delta t < 120$, $< 300$, else (decision time of step $k$, line 104); `const`: $\lambda_{EW}$ = `demand`, $\lambda_{NS} = 0.45$ (lines 105-110). Per approach: $\lambda_N = \lambda_S = \lambda_{NS}/2$, $\lambda_E = (1-\text{asym})\lambda_{EW}$, $\lambda_W = \text{asym}\,\lambda_{EW}$ (line 111).
- Arrival: Bernoulli with $p = 1 - e^{-\lambda\Delta t}$ (lines 114-115), at most one per approach per step.
- Movement: left with $P = 0.25$, otherwise right with $P = $ `rt` (lines 116-118). Default right share $0.75 \times 0.15 = 11.25\%$.
- Desired speed and headway (lines 96-101, 121-122): nominal $V_d \sim U[11.1, 14.4]$ m/s, $T \sim U[1.2, 2.4]$ s; aggressive $U[12.5, 15.3]$, $U[0.8, 1.6]$; cautious $U[9.7, 12.5]$, $U[1.8, 3.0]$.
- CAV draw from its own stream (seed $5\cdot10^5$+seed), CAV if $U < $ `pen`, so fleets are nested across penetration rates (lines 127-130).
- IDM parameters and gap margin from another stream (seed $7\cdot10^5$+seed) (lines 133-145): nominal $a \sim U[1.0, 2.0]$, $b \sim U[1.5, 3.0]$, $s_0 \sim U[1.5, 2.5]$, $h_m \sim U[1.5, 3.0]$; other populations in the same lines.
- Scenarios A and B (lines 77-93, 148-159): fixed arrival indices `round(t/dt) + 1` (so the arrival is decided at $t$), $V_d = 13$, $T = 1.5$, $U = 0$ (all CAVs), other parameters at defaults (1.5, 2.5, 2.0, 2.0).

### 3.2 IDM: `sim/ctrl_idm.m:9-18`

$$
s^* = s_0 + \max\!\Big(vT + \frac{v(v - v_l)}{2\sqrt{ab}},\ 0\Big),\qquad
a_{IDM} = a\Big[1 - \Big(\frac{v}{\max(v_0,0.1)}\Big)^4 - \Big(\frac{s^*}{\max(g, 0.1)}\Big)^2\Big]
$$
clipped to $[-8, a]$; the interaction term is 0 when $g = \infty$.

How the engine calls it (`sim/sim_run.m:250-281`):

- $v_0 = V_d$, capped by the curve speed at the current position: $v_0 \leftarrow \min(v_0, v_{curve}(s))$ (line 269).
- Humans use their own $a, b, s_0, T$. CAVs under IDM use $a = 1.5$, $b = 2.5$, $s_0 = 2$ and their own drawn $T$ (lines 270-273).
- IDM CAV with an RSU target time (`msg=time`, message held, $t < t_{gt}$, $s < s_{pt}$) (lines 252-261): $v_t = (s_{pt} - s)/\max(t_{gt} - t, 0.5)$; pulled: $v_0 \leftarrow \min(\max(v_0, v_t), V_{lim})$; otherwise $v_0 \leftarrow \min(v_0, \max(v_t, 1))$.
- IDM CAV with speed advice (`fresh`, finite `vadv`, $0 < s_{stop} - s \le 80$): $v_0 \leftarrow \min(v_0, \max(v_{adv}, 0.5V_d))$ (lines 262-268).

### 3.3 Curve speed cap: `sim/curve_cap.m:8-20`

Over samples within the next 60 m with $v_{curve} < v$ and distance $d > 0.5$ m:
$$
b_{req} = \max \frac{v^2 - v_{curve}^2}{2d},\qquad a \leftarrow \min(a,\ -b_{req}) \text{ if } b_{req} > 0.5b
$$
(applied at `sim/sim_run.m:275`, and in the MPC fallback `sim/ctrl_mpc.m:149` with $b = 2.5$).

### 3.4 Human noise: `sim/sim_run.m:276-280`

$$
a \leftarrow a\,(1 + \epsilon_1/30) + 0.05\,\epsilon_2,\qquad \epsilon_{1,2} \sim N(0,1)
$$
Applied after the curve cap, so noise can exceed the cap and the IDM bounds.

### Things to check (part 3)

- IDM CAVs (C2) keep the drawn headway $T \sim U[1.2, 2.4]$ s (`sim/sim_run.m:272`), while MPC CAVs use a fixed $T_c = 1.2$ s in a hard constraint (`sim/ctrl_mpc.m:50`). C2 vs C3/C4 therefore also compares mean headways of about 1.8 s and 1.2 s, which changes discharge capacity independently of coordination.
- Human $V_d$ goes up to 14.4 m/s (52 km/h); CAVs are capped at 13.89 m/s.
- The IDM target-time rule ([5]) only changes $v_0$; IDM approaches $v_0$ gradually, so the realized arrival can differ from the target. Measure the tracking error of C2 targets in the log.
- The noise is white per step; check that it matches the driver model described in the paper.

---

## 4. Right-turn decision

### 4.1 Gap check: `sim/turn_gap_ok.m`

Called for every uncommitted right-turner with $s \ge s_{stop} - 40$ (`sim/sim_run.m:187`), as `mode='cav'` with margin $m = 1.5$ s for CAVs (line 189) and `mode='human'` with $m = 0.7 + h_m$ for humans (line 197). $PET_{min} = 1.5$ s (`sim/turn_gap_ok.m:27`). It uses the true states of all vehicles (V2V or on-board sensing), not the RSU view.

Turner $i$, arc speed $v_t$ (line 35). For every opposite row $r = [q, z_{0k}, z_{1k}, z_{0q}, z_{1q}]$ (lines 52-56):
$$
t_{enter} = T(z_{0k} - s_i, v_i, 1.5, v_t),\qquad t_{clear} = T(z_{1k} + L - s_i, v_i, 1.5, v_t)
$$

Opposing waiting turner (lines 42-51): among opposite right-turners that are uncommitted, $v < 0.5$ and past their stop line, the one with the largest $s$ ($s_{block}$); its time to leave the shared lane from rest:
$$
t_{vac} = T(s_{div}^{o} + L - s_{block}, 0, 1.5, v_t^{o})
$$

Vehicles $j$ on path $q$, sorted by $s$ descending, $t_{prev} = -\infty$ (lines 57-60). For each:

1. Already cleared ($s_j - L > z_{1q}$, lines 62-68): ignored, except in `cav` mode the turn is refused if
$$
\frac{s_j - L - z_{1q}}{\max(v_j, 1)} + t_{enter} < PET_{min}
$$
   (time since its rear left, at current speed, plus the turner's time to enter). This is the PET-after-clear rule of `0b64b16`; the header (lines 17-19) describes it.
2. Blocked flag: $s_j < s_{block}$ (line 69).
3. Signal (lines 70-76): if $s_j < s_{stop}^q + 0.05$ and red, or yellow with $s_{stop}^q - s_j \ge v_j^2/6$, the vehicle is ignored.
4. Arrival time at $z_{0q}$ (lines 77-91):
   - inside the zone: $t_{arr} = 0$;
   - `cav`: $a_j' = \max(a_j, 0)$, and $a_j' \ge 1.5$ if $v_j < 2$; $t_{arr} = T(z_{0q} - s_j, v_j, a_j', \max(V_{d,j}, v_j))$. If $j$ holds a hard RSU target (`hard`, finite `tgt`, $s_j < s_{pt}$, $t < t_{gt}$, received within 10 s) (lines 84-88):
$$
t_{arr} \leftarrow \max\!\Big(t_{arr},\ (t_{gt} - t) + \frac{z_{0q} - s_{pt}}{\max(v_j, 6)}\Big)
$$
   - `human`: $t_{arr} = (z_{0q} - s_j)/\max(v_j, 0.1)$ (line 90).
5. Blocked vehicles (lines 92-99): ignored if they can still stop with a 0.5 m reserve, $z_{0q} - 0.5 - s_j \ge v_j^2/(2\cdot 4)$ (line 97); otherwise
$$
t_{arr} \leftarrow \max\big(t_{arr},\ t_{vac} + T(z_{0q} - s_j, v_j, 2.0, \max(V_{d,j}, 1))\big)
$$
   (line 98).
6. Chain (lines 101-102): $t_{arr} \leftarrow \max(t_{arr}, t_{prev} + 1.0)$, $t_{prev} \leftarrow t_{arr}$.
7. Passes before (lines 104-105):
$$
t_{out} = t_{arr} + \frac{z_{1q} - \max(s_j, z_{0q}) + L}{\max(v_j, 3)},\qquad \text{ignore } j \text{ if } t_{out} + PET_{min} \le t_{enter}
$$
8. Test (lines 106-108): refuse if $t_{arr} < t_{clear} + m$.

The turn is allowed only if no vehicle on any row refuses it.

### 4.2 Go-time hold and commitment: `sim/sim_run.m:187-211`

- A CAV with a positive check, in an RSU configuration with `msg=time`, holding a message (10 s) with finite `go`, is refused while $t < go - 0.25$ (lines 192-195). With decisions on the 0.5 s grid, the turner leaves at the grid time nearest to `go` (at most 0.25 s early). The on-board check keeps the final say.
- `committed` when ok and $s \ge s_{wait} - 0.3$ (line 204), or when $s > s_{wait} + 0.05$ without permission ("could not stop", lines 205-207).
- Not committed and not ok: stop point $\min(s_{stop,i}, s_{wait})$ (lines 208-210).

### 4.3 Yielding rule: `sim/sim_run.m:214-237`

For each straight or left vehicle $i$ and each opposing right turn $k$ with a zone on its path (rows from `oppTurnRows`, `sim/sim_run.m:394-412`, $[k, z_{0i}, z_{1i}, z_{0k}, z_{1k}]$): if $s_i < z_{0i}$ and a turner on $k$ is committed, not yet clear ($s_u - L < z_{1k}$) and past $s_{wait}^k - 0.5$, then $i$ stops at $z_{0i} - 0.5$ if
$$
z_{0i} - s_i \ge \frac{v_i^2}{2\cdot 4}
$$
(lines 232-233). The gap check (step 5 above) asks for the same 0.5 m reserve and is therefore slightly stricter; the comment (`sim/turn_gap_ok.m:93-96`) now says so. Fixed in 70d8d01 (comment).

### Things to check (part 4)

- Human and CAV use the same turner kinematics ($1.5$ m/s² to $v_t$) for $t_{clear}$, but a human turner drives with its own $a \in [1, 2]$ and noise. A human with $a = 1$ clears later than its own check assumes.
- `cav` mode uses the true $a_j$ and the true $V_{d,j}$ of human drivers. If the paper says "V2V or on-board sensing", a human's $V_d$ is not observable.
- The chain $t_{prev} + 1.0$ links vehicles on the same path $q$ only; a left-turner behind a straight vehicle in the same lane is not chained.
- The PET-after-clear check exists only in `cav` mode.
- The yielding rule makes oncoming vehicles brake for a committed turner. This avoids collisions after a wrong turn decision but moves the cost into oncoming delay, so PET and collision counts understate unsafe decisions. Log how often the rule fires per configuration.
- A turner that "could not stop" commits regardless of the gap (line 206). Count these.

---

## 5. Estimators and the RSU's knowledge

### 5.1 What the RSU knows

`rsu_init` (`sim/rsu_init.m:14-58`) loads `simplot/estimators.mat` (theta, gpr, pigpr); `pigpr_on` is a copy that learns online. Default `theta = [2.0, 2.0, 1.5]` if there is no file.

| field | value | line |
|---|---|---|
| snapshot interval | 2 s | 20 |
| forgetting factor $\lambda$ | 0.995 | 24 |
| $h_{min}$ | 1.0 s (2.0 s if `margin=fixed`) | 25, 27 |
| $\tau_{safe}$ | 4.0 s | 26 |
| PET margin | 1.5 s | 28 |
| $d_{max}$ | `S.dmax` (2 s) | 30-31 |
| $v_{frac}$ | 0.7 | 32 |
| $a_{shape}$ | 1.0 m/s² | 33 |
| $d_{cmd}$ | 40 m | 34 |
| $adv_{max}$ | 1.5 s | 35 |
| $w_{adv}$ | 0.25 | 36 |

View: `rsu_receive` (`sim/rsu_receive.m:6-13`), `rsu_forget` (`sim/rsu_forget.m:5-9`), `rsu_view` (`sim/rsu_view.m:10-15`): $\hat s = s_{msg} + v_{msg}(t - t_{msg})$; $v$, $a$, path, `committed`, `cav` as last received. Humans are invisible unless `sense='all'`; in collect/eval mode every vehicle is seen (`sim/sim_run.m:143`).

### 5.2 Features: `sim/est_features.m:30-61`

For approach $a$, vehicles with $\hat s < s_{stop}$, closest first. "Ahead" vehicles: same approach, $\hat s < s_{stop} + 10$ (center), not committed (line 34). For vehicle $q$:

- $d = s_{stop} - s$, $v$;
- $n$ = ahead vehicles with larger $s$; $n_R$ = those that are right-turners (lines 48-50);
- $gap = \max(s_a - L - s, 0)$ to the closest ahead vehicle, else $gap = d$; $v_l$ its speed, else $v_l = V_{lim}$ (lines 43, 51-57). Fixed in 70d8d01 (comment at line 18 now says $V_{lim}$);
- $g$ = `signal_feature` (`function/signal_feature.m:15-26`): EW green start 60, yellow end 118; $g = 118 - t_c$ during green or yellow, $g = -\mathrm{mod}(60 - t_c, 120)$ during red ($t_c = t \bmod 120$);
- $\tau_{phys}$ from `est_phys`.

Inputs: $Z_4 = [d, v, gap, g]$ (plain GPR), $Z_b = [d, v, gap, g, n, v_l, n_R]$ (PI-GPR, plus the prior).

### 5.3 IDM forward simulation: `sim/est_idm.m:18-48`

All known vehicles of the approach before the stop line, nominal IDM $a=1.5$, $b=2.5$, $s_0=2$, $T=1.5$, $v_0 = 12.8$ m/s, step 0.5 s, up to 240 steps (120 s). The first vehicle has no leader. During red, or yellow when $s_{stop} - s \ge v^2/6$, a vehicle with no one between it and the line follows a standing obstacle whose rear is at the stop line (lines 30-35). $s^+ = s + \tfrac12(v + v^+)\Delta t$ (line 42). Crossing time $\tau = (k-1)\Delta t + \Delta t\,(s_{stop} - s)/(s^+ - s)$ (line 44); $\infty$ if not reached. This is the same clock as the labels since 70d8d01. Vehicles past the stop line (including a waiting right-turner) are not simulated.

### 5.4 Queue-discharge physics: `sim/est_phys.m:21-34`

With $\theta = [t_L, h_s, a_p]$, current time $t$, current or next green start $t_{g0}$, end of its yellow $t_{end}$:
$$
\tau_{kin} = T(d, v, a_p, V_{des,0}),\quad \tau_q = \max(t_{g0} - t + t_L, 0) + n h_s,\quad \tau = \max(\tau_{kin}, \tau_q)
$$
If $t + \tau > t_{end}$ (left over):
$$
n_{served} = \Big\lfloor \frac{\max(t_{end} - \max(t_{g0}, t) - t_L, 0)}{h_s}\Big\rfloor,\quad n_{left} = \max(n - n_{served}, 0),\quad \tau = \max(t_{g0} + 120 - t + t_L + n_{left}h_s,\ \tau_{kin})
$$
`travel_time_vec` (lines 37-45) is the vectorized $T$ without the $a = 0$ branch.

### 5.5 Arrival estimate used by the RSU: `sim/rsu_estimate.m:23-56`

- `idm`: $\hat\tau = \tau_{IDM}$, $\sigma = 0$ (lines 28-29).
- `phys`: $\hat\tau = \tau_{phys}$, $\sigma = 0$.
- `gpr`: $(\mu, s^2) = $ GP$(Z_4)$, $\hat\tau = \mu$, $\sigma$ = `gp_sd`$(s^2, \hat\tau)$; vehicles with $t + \tau_{phys} > t_{end}$ use $\tau_{phys}$ (lines 32-37).
- `pigpr`, `pigpr_on`: prior $p = \tau_{IDM}$, or $\tau_{phys}$ where $\tau_{IDM} = \infty$; $(\mu_r, s^2) = $ GP$([Z_b, p])$; $\hat\tau = p + \mu_r$; $\sigma$ = `gp_sd`$(s^2, \hat\tau)$; vehicles with $t + p > t_{end}$ use $p$ (lines 38-49). Left-over vehicles keep the GP's $\sigma$ at their out-of-scope inputs.
- Then for every estimator (lines 53-56): $\hat\tau \leftarrow \max(\hat\tau, \Delta t)$, and $\hat\tau_q \leftarrow \max(\hat\tau_q, \hat\tau_{q-1} + h_{min})$, closest first.

### 5.6 Sparse GP: `sim/gp_fit.m`, `gp_predict.m`, `gp_update.m`, `gp_refresh.m`

Standardization (`sim/gp_fit.m:30-34`): $\tilde x = (x - \bar x)/s_x$, $\tilde y = (y - \bar y)/s_y$.

ARD squared-exponential kernel (`sim/gp_fit.m:75-81`):
$$
k(x, x') = \sigma_f^2 \exp\Big(-\tfrac12 \sum_{d=1}^D \frac{(\tilde x_d - \tilde x'_d)^2}{\ell_d^2}\Big)
$$

Hyperparameters $h = [\log\ell_{1..D}, \log\sigma_f, \log\sigma_n]$, start $[0,\ldots,0, 0, \log 0.3]$ (line 40), fitted with `fminunc` on a random subset of $n_{opt}$ points (lines 37-51) by minimizing (lines 83-97)
$$
\mathcal{L}(h) = \tfrac12 \tilde y^\top K^{-1}\tilde y + \sum_i \log L_{ii} + \tfrac n2 \log 2\pi + \tfrac12 \sum_d \Big(\frac{\log \ell_d}{1.5}\Big)^2,\quad K = K_f + (\sigma_n^2 + 10^{-8})I
$$
a MAP estimate (the header, lines 13-15, now says so; fixed in 70d8d01), with analytic gradient (lines 98-107), $W = \alpha\alpha^\top - K^{-1}$, $\alpha = K^{-1}\tilde y$, $D_d = ((\tilde x_d - \tilde x_d')/\ell_d)^2$:
$$
\frac{\partial\mathcal L}{\partial \log\ell_d} = -\tfrac12 \mathrm{tr}\big(W (K_f \odot D_d)\big) + \frac{\log\ell_d}{1.5^2},\quad
\frac{\partial\mathcal L}{\partial \log\sigma_f} = -\tfrac12\mathrm{tr}(W\, 2K_f),\quad
\frac{\partial\mathcal L}{\partial \log\sigma_n} = -\tfrac12 \mathrm{tr}(W)\, 2\sigma_n^2
$$
Afterwards $\sigma_n \ge 10^{-3}$ (line 53).

DTC on $m$ inducing points (a different random subset, lines 58-69):
$$
A = K_{mm} + 10^{-6}\sigma_f^2 I + \sigma_n^{-2}K_{mn}K_{nm},\qquad b = \sigma_n^{-2}K_{mn}\tilde y
$$
(`K_mm` already contains the jitter, line 61.) `gp_refresh` (`sim/gp_refresh.m:3-12`) stores $w = A^{-1}b$, $A^{-1}$, $K_{mm}^{-1}$ by Cholesky.

Prediction (`sim/gp_predict.m:4-15`), $k_m = k(Z, x)$:
$$
\mu(x) = s_y\, k_m^\top w + \bar y,\qquad
s^2(x) = s_y^2\Big[\max\big(\sigma_f^2 - k_m^\top (K_{mm}^{-1} - A^{-1})k_m,\ 0\big) + \sigma_n^2\Big]
$$
(standard DTC mean and variance plus noise).

Online update with forgetting (`sim/gp_update.m:12-22`), for a batch of $k$ points:
$$
A \leftarrow K_{mm} + \lambda (A - K_{mm}) + \sigma_n^{-2}K_{mk}K_{km},\qquad b \leftarrow \lambda b + \sigma_n^{-2} K_{mk}\tilde y_k
$$
Hyperparameters, inducing points and standardization stay fixed. The header (lines 8-9) now states a low-rank update with the forgetting factor applied once per call. Fixed in 70d8d01 (comment).

### 5.7 Horizon-dependent noise: `sim/gp_sd.m`, `sim/gp_calibrate.m`

$$
\sigma(\hat\tau) = \sqrt{\max\big(s^2 - (s_y\sigma_n)^2, 0\big) + \big(c_0 + c_1\max(\hat\tau, 0)\big)^2}
$$
(`sim/gp_sd.m:11-16`): latent GP variance plus a noise linear in the predicted time. $c_0, c_1 > 0$ by maximum likelihood with `fminsearch` on $\log c$ from $[\log 0.5, \log 0.1]$ (`sim/gp_calibrate.m:8-19`):
$$
\min_{c_0,c_1>0} \sum_i \tfrac12\log v_i + \tfrac12 \frac{r_i^2}{v_i},\qquad v_i = \sigma^2_{lat,i} + (c_0 + c_1\tau_i)^2
$$
`README.md:40` now writes the same formula. Fixed in 70d8d01 (README and `gp_calibrate.m:7`).

### 5.8 Training: `function/train_estimators.m`

1. Data (lines 35-46): C1 (all human, nominal drivers, staged schedule) on seeds 1-20 with `collect=true`; the RSU sees every vehicle. Rows `[seed t id app d v gap g n vl nR prior tphys y idm]` (comment line 45); `prior` and `idm` are the raw IDM forward simulation; $y = t_{stop} - t_{snap}$.
2. Scope (line 48): snapshots with $g > 0$ and $y \le g$.
3. Split (line 49): test = seeds 17-20, train = 1-16.
4. Physics grid (lines 53-71): $t_L \in \{0, 0.25, \ldots, 5\}$, $h_s \in \{1.0, 1.1, \ldots, 4.0\}$, $a_p \in \{0.6, 0.8, 1.0, 1.2, 1.5\}$, minimizing $\frac1N\sum(\tau_{phys} - y)^2$ on training rows.
5. Prior (line 72): IDM, $\tau_{phys}$ where IDM is infinite.
6. Fit (lines 79-81): $m = 400$, $n_{opt} = 1000$, 200 iterations. GPR on $Z_4 \to y$; PI-GPR on $[Z_b, p] \to y - p$.
7. Calibration (lines 83-88) on the training rows.
8. Test metrics (lines 90-111, `sim/est_metrics.m:9-25`): RMSE, MAE, bias, $|e| < 2$ s and $< 5$ s, 90% coverage $|e| \le 1.645\sigma$, mean NLL.

Because the clock changed in 70d8d01, the estimators must be retrained before any study run.

### 5.9 Snapshots and online learning: `sim/rsu_step.m`, `sim/rsu_crossed.m`

- Snapshot when $t$ is a multiple of 2 s and learning is active (collect, eval, or `plan & online & est=pigpr`) (`sim/rsu_step.m:11-14`). Features from the RSU view and the raw IDM prior (fallback $\tau_{phys}$) (lines 29-34) are stored as pending (lines 35-39).
- On stop-line crossing (`sim/rsu_crossed.m:11-47`): every pending snapshot of the vehicle with $0 < y \le 120$ becomes a sample. Online (lines 29-46): in planning mode, a vehicle is skipped if the RSU delayed it, i.e. its shaping record has $t_{gt} - nat \ge 0.3$ s, or pulled it forward (lines 33-38, b3dfe15); holds at the natural time and eco vehicles are learned from. Otherwise snapshots with $g > 0$ and $y \le g$ update `pigpr_on` with residual targets $y - p$ in one `gp_update` call (line 42), and `rsu.pigpr = rsu.pigpr_on` (line 45).
- Planning (`sim/rsu_step.m:42-44`): `rsu_plan`, both approaches together.

### Things to check (part 5)

- **Training population vs. use.** The estimators are trained on C1 human traffic only (`function/train_estimators.m:40`) and used in C3/C4, where vehicles are MPC CAVs with $T_c = 1.2$ s, capped speed and RSU holds. The online update (C4 only) has to absorb the shift within one 360 s run. Report estimator error inside C4 runs, not only on C1 seeds.
- **Selection in online learning.** Since b3dfe15 the RSU skips only vehicles it delayed by 0.3 s or more or pulled forward (`sim/rsu_crossed.m:33-38`); in 70d8d01 it also skipped eco and held vehicles, which removed most blocked-queue arrivals (fixed). Eco and held arrivals are still shaped by the MPC's arrival-time hold, so the learner partly learns controlled arrivals. Check `rsu.online_n` per run.
- Calibration of $c_0, c_1$ uses in-sample residuals (`function/train_estimators.m:86-87`); check the 90% coverage on the test seeds.
- With $\lambda = 0.995$ applied once per crossing vehicle, the half-life is about 138 crossings.
- In mixed traffic with `sense='v2x'`, $n$, $gap$, $v_l$, $n_R$ omit human vehicles, unlike training.
- `est_idm` ignores the waiting right-turner (by design, `sim/est_idm.m:12`); for C2 only the lane-order constraint in `rsu_plan` covers it.
- The `idm` estimator in `est_eval_run` includes the $h_{min}$ chain (via `sim/rsu_estimate.m:53-56`), while `idm` in `train_estimators` is the raw IDM (`function/train_estimators.m:93`). Same name, different estimator; label them differently in the paper tables.
- `est_phys` subtracts $t_L$ also when the queue is already discharging (`sim/est_phys.m:30`).

---

## 6. The slot optimizer: `sim/rsu_plan.m`

This is the core of the paper. Reference [5] (Hasan et al.): a waiting right-turner is inserted before oncoming vehicle $p$ (slot search), quadratic delay cost, $\tau_{safe} = 4$ s, $h_{time} = 2$ s, IDM arrival estimate, planned once per green, turners one after another. C2 rebuilds it with `slot=seq, plan=once, margin=fixed, est=idm, ctrl=idm`. C4 uses `slot=opt, plan=rolling, margin=geom, est=pigpr, ctrl=mpc, adv=1`.

Setup (`sim/rsu_plan.m:89-97`): $h = h_{min}$ (1 s, or 2 s with fixed margins), $z$ = `unc_z`, `ctl` = (`msg=time` and `ctrl=mpc`), `strict`, `once`, `fixedM`, `adv` = `cfg.adv` and not strict, $s_{ctl} = 0.5$ s (one control step), $K = 2$ turners per approach.

### 6.1 Lanes and vehicle times (`sim/rsu_plan.m:116-188`)

For each approach $a$ (opposite $o$): right-turn path $aR$, arc speed $v_t = 4.243$ m/s, and `rowsO` = zones of the opposite turner $oR$ (line 121). Estimates $(\hat\tau, \sigma)$ from `rsu_estimate` with `cfg.est` (line 122).

Each lane is a list ordered by $s$ descending with fields `id, mov, cav, fixed, d, v, s, lo, sd, turn`:

- Past right-turner ($s \ge s_{stop}$, not committed, $s \le s_{wait} + 0.05$) (lines 128-131):
$$
lo = T(s_{wait} - s, v, 1.5, v_t),\quad \sigma = 0.3,\quad d = 0,\quad turn = true
$$
- Past straight or left vehicle whose rear is still before $z_1$ of its zone with the opposite turner (lines 132-137): fixed,
$$
lo = -\frac{s - s_{stop}}{\max(v, 1)}\quad (\text{approximate time since it crossed}),\quad \sigma = 0
$$
- Vehicle before the stop line with finite $\hat\tau$ (lines 140-146): $lo = \hat\tau$; an uncommitted right-turner is planned at its waiting point:
$$
lo = \hat\tau + \frac{s_{wait} - s_{stop}}{v_t} = \hat\tau + 1.709\ \text{s (computed)}
$$
- Shaped before (record in `rsu.shp`, lines 149-157): $lo \leftarrow \max(lo, t_{gt} - t)$ (a given target is never taken back), $nat = nat_{stored} - t$.
- Lane order for non-fixed vehicles (lines 158-160): $lo_i \leftarrow \max(lo_i, lo_{i-1} + h)$.
- Natural time (lines 163-164): $nat = lo$ for unshaped vehicles; $nat \leftarrow \min(nat, lo)$.

### 6.2 Shaping limits (`sim/rsu_plan.m:165-173`)

Eligible: not fixed, not a turner, not R, CAV, not strict, $d \ge d_{cmd} = 40$ m, $v \ge 5$ m/s.
$$
r = 1 - \sqrt{\max\Big(0,\ 1 - \frac{a_{shape}\, d}{v^2}\Big)},\qquad r \leftarrow \min(r, 1 - v_{frac}) = \min(r, 0.3)
$$
$$
cap = \min\Big(d_{max},\ \frac{d}{2v}\Big(\frac{1}{1-r} - 1\Big)\Big),\qquad hi = \max(lo,\ nat + cap)
$$
$r$ is the fractional speed loss when decelerating at $a_{shape}$ over $d/2$ ($v_{end}^2 = v^2 - a_{shape}d$); the cap is the extra time to drive $d/2$ at $(1-r)v$ (it ignores the time lost while decelerating, so it is a lower estimate of what that deceleration produces). For non-eligible vehicles $hi = lo$. (computed) At $v = 13.89$ m/s: $d = 40$: 0.18 s; 60: 0.44 s; 80: 0.89 s; 100: 1.54 s; $\ge 130$: 2.0 s.

### 6.3 Pull-forward (`sim/rsu_plan.m:174-183`)

With `adv`, same eligibility, $t_{fast} = T(d, v, 1.0, V_{lim})$:
$$
lo_{adv} = lo - \max\big(0,\ \min(adv_{max},\ d/v - t_{fast})\big),\quad adv_{max} = 1.5\ \text{s}
$$
The gain is measured against constant current speed $d/v$, not against $lo$. (computed) $d=100$, $v=11$: 1.5 s; $d=60$, $v=11$: 0.83 s; $d=100$, $v=12.5$: 0.73 s.

### 6.4 Who may be held or commanded (`sim/rsu_plan.m:184-186`)

$$
hold = ctl \wedge cav \wedge \neg fixed \wedge mov \ne R \wedge \neg strict,\qquad cmd = hold \vee (hi > lo + 10^{-6})
$$
`hold` has no distance condition: a CAV within 40 m or queued can still be held at its natural time (no delay). The docstring says "allowed at any distance" (`sim/rsu_plan.m:46`).

### 6.5 Planned turners and margins (`sim/rsu_plan.m:190-242`)

Only approaches with a green light (line 195). Candidates: turners with $s \ge s_{stop} - 120$, the first $K$ (lines 200-201; $K = 50$ in `once` mode, line 112). For turner $k$:

- $\sigma_k = \hat\sigma_k$, or $s_{ctl} = 0.5$ s when `ctl` (line 203).
- Start speed $v_{t0} = v_t$ if `ctl`, $v > 1$ and $s < s_{wait} - 1$ (rolling turn), else 0 (lines 204-205).
- For each vehicle $m$ of the opposite lane with a zone row (straight or left) (lines 208-233). Its speed at the stop line is not known, so both oncoming times are bounded on the safe side (979b6c2, lines 211-225): it enters the zone at the earliest, at full speed $v_{cap}$, and leaves at the latest, from its present speed, or from rest if it is slow now or predicted to stop first (e3d786a), accelerating at 1.5 m/s² up to $v_{cap}$:
$$
v_{cap} = \begin{cases} V_{lim} = 13.89 & m \text{ straight}\\ v_{curve}^{oL} = 3.674 & m \text{ left-turner}\end{cases},\qquad v_{0m} = \begin{cases} 0 & \min(v_m, v_{cap}) < \min(5,\ 0.8\,v_{cap}) \ \text{ or } \ lo_m > d_m/\max(v_m, 0.1) + 1.5\\ \min(v_m, v_{cap}) & \text{otherwise}\end{cases}
$$
$$
d_{ent,m} = \frac{z_{0q} - s_{stop}}{v_{cap}},\quad d_{exit,m} = T(z_{1q} + L - s_{stop}, v_{0m}, 1.5, v_{cap})
$$
$$
d_{ent,k} = T(z_{0k} - s_{wait}, v_{t0}, 1.5, v_t),\quad t_{clr,k} = T(z_{1k} + L - s_{wait}, v_{t0}, 1.5, v_t),\qquad u = z\sqrt{\sigma_k^2 + \sigma_m^2}
$$
$$
D^b_{km} = d_{exit,m} - d_{ent,k} + PET + u,\qquad
D^a_{km} = t_{clr,k} - d_{ent,m} + PET + u,\qquad
D^{aC}_{km} = t_{clr,k} - d_{ent,m} + PET + z\sqrt{\sigma_k^2 + s_{ctl}^2}
$$
  (lines 222-231). With `margin=fixed`: $D^b = D^a = D^{aC} = \tau_{safe} = 4$ s (line 232). For a left-turner the speed threshold is $0.8 \times 3.674 = 2.94$ m/s, so a left-turner already moving at its arc speed keeps it. The second condition (line 223) treats a vehicle whose predicted stop-line time is more than 1.5 s later than its free travel at the present speed as one that will stop first (queue, or behind a waiting turner of its own lane). Past vehicles have $d_m = 0$ and negative $lo_m$, so only the speed test applies to them.
- Derivation: the turner leaves $s_{wait}$ at $\tau_k$, enters at $\tau_k + d_{ent,k}$, clears at $\tau_k + t_{clr,k}$; $m$ crosses its stop line at $\tau_m$, enters at $\tau_m + d_{ent,m}$, clears at $\tau_m + d_{exit,m}$. Turner after $m$: $\tau_k + d_{ent,k} \ge \tau_m + d_{exit,m} + PET + u$, i.e. $\tau_k \ge \tau_m + D^b$. Turner before $m$: $\tau_m + d_{ent,m} \ge \tau_k + t_{clr,k} + PET + u$, i.e. $\tau_m \ge \tau_k + D^a$.
- Stored per turner (lines 234-237): conflicting indices `cidx` (in lane order), $D^b, D^a, D^{aC}$, $t_{clr}$ of the farthest zone, $t_{end}$ = yellow end $- t$, $s_{pt} = s_{wait}$. In `once` mode the planned turners `[T.id]` and all in-range turners are added to `once_ids` (line 241).

(computed) Values for ER/WR with $PET = 1.5$ s and $u = 0$; "state now" is the oncoming vehicle's present speed and whether it is late:

| oncoming $m$ | turner start | state now | $d_{ent,m}$ | $d_{exit,m}$ | $D^b$ [s] | $D^a$ [s] |
|---|---|---|---|---|---|---|
| straight (WS) | rest | 13.89 m/s, not late | 0.47 | 1.21 | 1.89 | 5.10 |
| straight (WS) | rest | 6 m/s, not late | 0.47 | 2.19 | 2.87 | 5.10 |
| straight (WS) | rest | below 5 m/s, or late | 0.47 | 4.73 | 5.41 | 5.10 |
| left (WL, merge) | rest | 2.94 m/s or more, not late | 1.77 | 4.49 | 3.54 | 4.56 |
| left (WL, merge) | rest | below 2.94 m/s, or late | 1.77 | 5.72 | 4.77 | 4.56 |
| straight (WS) | rolling $v_t$ | 13.89 m/s, not late | 0.47 | 1.21 | 2.59 | 3.68 |
| straight (WS) | rolling $v_t$ | below 5 m/s, or late | 0.47 | 4.73 | 6.11 | 3.68 |
| left (WL, merge) | rolling $v_t$ | 2.94 m/s or more, not late | 1.77 | 4.49 | 4.93 | 3.15 |
| left (WL, merge) | rolling $v_t$ | below 2.94 m/s, or late | 1.77 | 5.72 | 6.15 | 3.15 |

("late" = $lo_m > d_m/v_m + 1.5$ s.)

$t_{clr}$ (farthest zone): 4.83 s from rest, 3.42 s rolling. The held-vehicle term $z\sqrt{2}\,s_{ctl} = 1.645\sqrt{0.5} = 1.16$ s for $z = 1.645$. $D^a$ no longer depends on $v_m$. The fixed 4 s of [5] is below the geometric $D^a$ for a turner from rest (4.56 to 5.10 s).

### 6.6 Constraints and the fixed-point evaluation (`evalSlots`, `sim/rsu_plan.m:399-477`)

A slot vector $sl$ gives each planned turner $k$ a slot $p_k \in \{1, \ldots, |cidx_k| + 1\}$: after the first $p_k - 1$ conflicting vehicles of the opposite lane and before the rest ($p_k = 0$: not planned). $c_j$ is the $j$-th conflicting vehicle.

Start: $\tau = lo$ in both lanes. With `pull`, every CAV before a planned turner's slot with $lo_{adv} < \tau - 0.3$ starts at $lo_{adv}$ and is flagged `pulled` (lines 405-416, threshold at line 411).

Iterate up to 30 times (lines 417-451), only increasing times:

1. Lane order (non-fixed): $\tau_i \leftarrow \max(\tau_i, \tau_{i-1} + h)$ (lines 419-426).
2. Before the slot: $\tau_k \leftarrow \max\big(\tau_k, \max_{j < p_k} (\tau_{c_j} + D^b_{k c_j})\big)$ (lines 431-436).
3. After the slot, $j \ge p_k$, $m = c_j$ (lines 437-448):
$$
\tau_m \leftarrow \max(\tau_m,\ \tau_k + M_{km}),\qquad M_{km} = \begin{cases} D^{aC}_{km} & hold_m, \text{ or } m \text{ right behind a held, holdable vehicle } (\tau_m \le \tau_{m-1} + h)\\ D^a_{km} & \text{otherwise}\end{cases}
$$
   If $\tau_m$ was raised and $m$ is commandable, $m$ is marked `held`; if it was not raised but $hold_m$ and $lo_m < \tau_k + D^a_{km}$, it is marked `held` too (only safe because it is held).

`ok` = no change in the last iteration (line 452); a cycle ends after 30 iterations as infeasible.

Feasibility (lines 453-472), vehicle $i$ with time $x_i$:

- $x_i < lo_i - 0.01$: feasible only if `pulled`, $x_i \ge lo_{adv,i} - 0.01$ and $x_i \le lo_i - 0.29$, i.e. at least 0.3 s earlier than $lo$ within the 0.01 s tolerance (line 460, new in b3dfe15).
- $|x_i - lo_i| \le 0.01$: feasible (flag `pulled` cleared).
- $x_i > lo_i + 0.01$: infeasible if fixed; feasible if a turner; feasible if pushed by the vehicle ahead ($x_i \le x_{i-1} + h + 0.01$, line 469); feasible if commandable and $x_i \le hi_i + 0.01$ (line 470); otherwise infeasible.

A human after the slot can only be later than $lo$ by following the vehicle ahead; this is how "the turner yields to vehicles the RSU cannot instruct" is enforced.

Cost (lines 473-475), over all non-fixed vehicles of both approaches:
$$
J = \sum_{i} \max(x_i - nat_i, 0)^2 + w_{adv}\min(x_i - nat_i, 0)^2,\qquad w_{adv} = 0.25
$$

`evalBest` (lines 387-397) evaluates a slot vector without and with pulling and keeps the feasible one with lower $J$.

### 6.7 Search rules (`sim/rsu_plan.m:244-309`)

- Candidate slots (lines 245-254): for each turner alone, the feasible slots; if none, only the last slot.
- `opt` and `strict` (lines 258-276): exhaustive over the product of the candidate sets, skipping vectors where two turners of one lane are out of lane order (`laneOrderOk`, lines 479-489); keep the feasible vector with the smallest $J$. With more than 5000 combinations the successive result is used instead, and the original candidate sets are kept as `S0` (lines 261-264).
- `seq` (lines 277-280, `seqSlots` lines 369-385): turners sorted by $nat_k$; each takes its lowest-$J$ candidate given the slots fixed so far (later turners unplanned); ties keep the earlier slot; if none is feasible, its last candidate. If the final vector is infeasible, nothing is planned.
- `strict`: no shaping, no hold, no pull: existing gaps only.
- No-slot turners (lines 294-309): after the choice, a turner with $\tau_k + t_{clr} > t_{end}$ gets $sl_k = 0$ and the plan is re-evaluated. If the re-evaluated plan is feasible it replaces the old one; otherwise the function returns without any message this step, not even turner go messages (lines 306-309, 979b6c2); in `once` mode it also sets `once_tg0 = -inf`, so the next step replans (line 307, e3d786a).

### 6.8 Planning modes (`sim/rsu_plan.m:98-113`)

- `rolling`: replans every step.
- `once`: when neither E nor W is green, the stored messages and `once_ids` are cleared (lines 102-104). During green it plans when the green starts or when a turner in $[s_{stop} - 120, s_{stop} + 8]$ is not yet in `once_ids` (lines 106-111), with $K = 50$; otherwise it repeats the stored messages (line 109). After planning, the planned turners and all in-range turners enter `once_ids` (line 241). After an inconsistent plan, `once_tg0 = -inf` forces a new plan at the next step (line 307).

### 6.9 Messages (`sim/rsu_plan.m:311-365`)

- Turner (lines 312-322): no message if it has no slot this green (`noslot`, counted). Else `go` = `tgt` = $t + \tau_k$, `spt` = $s_{wait}$, `vadv` = $(s_{wait} - s)/\tau_k$ if $\tau_k > 0.5$, `pull` = `hard` = false.
- Other CAVs (not fixed, not turners, not R) (lines 324-364); `keep` = has a shaping record whose target has not passed (lines 328-329):
  - Pulled and $x_i \le lo_i - 0.29$ (the same threshold as the feasibility rule, 979b6c2): record the gain $lo - x$ in `rsu.pul`, send `tgt` = $t + x_i$, `spt` = $s_{stop}$, `pull` = true, `hard` = false (lines 330-341).
  - Else if commandable and (`held` or `keep`): $t_m = \min(x_i, hi_i)$. New record: `nat` = $t + nat_i$, `tgt` = $t + t_m$, `t0` = $t$. Existing record: $t_{gt} \leftarrow \max(t_{gt}, \min(t + t_m, nat + d_{max}))$, $t_m \leftarrow \max(t_m, t_{gt} - t)$ (lines 342-351).
  - Else if `eco`, `msg=time` and $x_i > lo_i + 0.5$: eco, $t_m = x_i$ (lines 352-355).
  - Message: `tgt` = $t + t_m$ (NaN with speed messages), `spt` = $s_{stop}$, `vadv` = $d/\max(t_m, 0.5)$, `hard` = `ctl` (lines 359-362).
- In `once` mode the message list is stored and repeated (line 365).

### 6.10 Diagnostics counter (`sim/rsu_plan.m:283-293`)

`rsu.plans` +1 per plan (line 283). For `opt` only, computed before the no-slot re-evaluation and with the original candidate sets (`S0`) when the fallback was used (line 287): `rsu.diag = [plans, plans with nT >= 2, plans where some turner had more than one candidate slot, plans where the successive search would choose a different slot vector]`. When the fallback is used, `best.sl` is the successive result, so `diag(4)` is 0 for those plans by construction. It compares slot vectors only. Fixed in b3dfe15 (was inflated by no-slot zeros).

### 6.11 What is exact and what is approximate

Exact, given the inputs:

- For a fixed slot vector and pull flag, the times are the least fixed point of a monotone system, so each vehicle gets the earliest time consistent with lane order, $D^b$ and $D^a$/$D^{aC}$.
- `opt` (up to 5000 combinations): the minimum of $J$ over all lane-order-consistent combinations of the candidate slots, with pull or no pull for the whole vector.

Approximate or heuristic:

- **Candidate pruning** (lines 245-254). The comment "a joint assignment only adds delays" (line 245) is true for the constraints, but feasibility is not monotone: a vehicle that is infeasible alone (pushed beyond $hi$ by $D^a$) can become feasible jointly as "pushed by the one ahead" (line 469) when another turner delays the vehicle ahead. `opt` is exhaustive over candidate slots, not over all slots.
- **Pull is all or nothing**: every eligible CAV with gain $\ge 0.3$ s before every slot starts at $lo_{adv}$; the amount is not optimized, and a pulled vehicle pushed back to less than 0.3 s gain makes the whole pulled variant infeasible (line 460) instead of un-pulling only that vehicle.
- **Kinematics in the margins**: oncoming vehicles enter at $v_{cap}$ and leave from their present speed (or from rest when slow or predicted to stop first) with 1.5 m/s² up to $v_{cap}$, both bounds on the safe side; turners use 1.5 m/s² from rest or roll at $v_t$.
- **One time per vehicle** (stop-line time; waiting-point time for turners); lane order compares a turner's waiting-point time with its follower's stop-line time.
- **Above 5000 combinations** the joint search is replaced by the successive one.
- $K = 2$ turners per approach in rolling mode; humans invisible with `sense=v2x`; the estimator's $\hat\tau$, $\hat\sigma$; the constant $s_{ctl}$.

### Things to check (part 6)

- Fixed in 70d8d01: left-turners at their arc speed; pull threshold 0.3 s at the start of `evalSlots`; no-slot turners removed and the plan re-evaluated; $s_{ctl} = 0.5$ s.
- Fixed in b3dfe15: a pulled vehicle must gain at least 0.3 s (line 460); diagnostics before the re-evaluation and with `S0` (lines 284-293); `once_ids` includes all in-range turners (line 241).
- Fixed in 979b6c2: oncoming times bounded on the safe side (lines 211-225). Entry at full speed means a vehicle that is stopped now but will cross moving no longer shrinks $D^a$ (2.62 s in b3dfe15, now 5.10 s for a turner from rest); exit from rest below 5 m/s.
- Fixed in 979b6c2: when the no-slot re-evaluation is infeasible the function returns with no messages (lines 306-309), so turners are not released on a plan whose holds are not sent.
- Fixed in 979b6c2: the pull message threshold is $x_i \le lo_i - 0.29$ (line 330), the same as the feasibility rule (line 460).
- Fixed in e3d786a: a vehicle predicted to stop first ($lo_m > d_m/v_m + 1.5$ s) leaves from rest in $D^b$ (line 223); a left-turner moving at its arc speed is no longer forced to start from rest (threshold $\min(5, 0.8\,v_{cap})$); in `once` mode an inconsistent plan sets `once_tg0 = -inf`, so the next step replans (line 307).
- **The "will stop first" test** (line 223) compares $lo_m$, which includes lane order, shaping targets and, for left-turners, the slowdown for the curve, with $d_m/v_m$ at the present speed. Shaped vehicles and left-turners far away can therefore be classed as "late" and get the from-rest exit time. This is on the safe side; check how often it fires.
- **Inconsistent plans still count.** `rsu.plans` and `rsu.diag` (lines 283-293) are incremented before the consistency check, so plans that send nothing (lines 306-309) are counted too.
- **Fixed-margin baselines.** With `margin=fixed`, $D^a = 4$ s is below the geometric $D^a$ for a turner from rest; C2/C3 plans can be geometrically unsafe and rely on the on-board check. Check how [5] defines the 4 s (it may refer to other reference points).
- **$s_{ctl}$ vs. what the MPC guarantees.** The hold allows crossing up to one step (0.5 s) before the target, and the turner leaves up to 0.25 s before `go` (part 4.2). Measure the actual tracking error once.
- `lo` of a past fixed vehicle uses $\max(v, 1)$ and its current speed.
- Records in `rsu.shp` and `rsu.pul` are written before transmission; with packet loss, `n_shaped` and `n_pulled` count commands that never arrived, and a plan that relied on a `held` vehicle ($D^{aC}$) is unsafe if that message is lost.
- In `once` mode, an in-range turner that could not be planned (for example an infinite IDM estimate) is now marked as handled and is only planned when another new turner triggers a replan.
- `seq` orders turners by $nat_k$; check that "order of arrival" in [5] means arrival at the waiting point.
- The reference configurations also send eco times (`eco = 1` in `ref`, `sim/sim_config.m:48`) and replan when a new turner comes into range (lines 106-111). Confirm both are part of [5] and the CISE method, or state the deviations. [5] also had no shaping limit; C2 uses $d_{max}$, $v_{frac}$, $a_{shape}$, $d_{cmd}$ (deliberate, per the README).

---

## 7. The MPC: `sim/ctrl_mpc.m`, `sim/qp_solve.m`

### 7.1 Prediction model (`sim/ctrl_mpc.m:37-48`)

$N = 20$, $\Delta t = 0.5$ s (10 s horizon), decision variables $u_1, \ldots, u_N$ (piecewise-constant acceleration), 1-based:
$$
v_k = v + \Delta t \sum_{i=1}^{k} u_i \;\Rightarrow\; S_v = \Delta t\,\mathrm{tril}(\mathbf 1)
$$
$$
s_k = s + k\Delta t\, v + \Delta t^2 \sum_{i=1}^{k} \big(k - i + \tfrac12\big) u_i \;\Rightarrow\; (S_s)_{ki} = \Delta t^2 (k - i + \tfrac12),\ i \le k
$$
Both are exact for piecewise-constant $u$ (the header, lines 11-12, writes the same with 0-based $i$). $D_j = I - \text{subdiag}(1)$ gives $u_k - u_{k-1}$. Prediction times $t_k = t + k\Delta t$ (line 51).

### 7.2 Cost (`sim/ctrl_mpc.m:50`, `76-78`)

$$
J = w_v \sum_k (v_k - v_{ref,k})^2 + w_u \sum_k u_k^2 + w_j \sum_k (u_k - u_{k-1})^2,\quad w_v = 0.1,\ w_u = 2,\ w_j = 10,\ u_0 = a_{current}
$$
In QP form $\tfrac12 U^\top H U + f^\top U$:
$$
H = 2(w_v S_v^\top S_v + w_u I + w_j D_j^\top D_j),\qquad f = 2\big(w_v S_v^\top (v\mathbf 1 - v_{ref}) - w_j D_j^\top e_0\big),\ e_0 = [a, 0, \ldots, 0]^\top
$$
(The "jerk" term is $\Delta u$, not $\Delta u/\Delta t$.)

Reference speed (lines 53-74): predicted positions from the previous plan shifted by one step (last entry extrapolated with the current $v$), else $s + k\Delta t v$; $v_{curve,k}$ at those positions; $v_{ref} = \min(V_d, v_{curve})$; speed messages (`msg=speed`, fresh, $0 < s_{stop} - s \le 80$): $v_{ref} \leftarrow \min(v_{ref}, \max(v_{adv}, 0.5V_d))$; pulled (message within 10 s, `pull`, $t < t_{gt}$, $s < s_{pt}$): $v_{ref} = \min(V_{lim}, v_{curve})$.

### 7.3 Constraints

- Bounds $-6 \le u_k \le 2$ (line 140).
- Speed (line 80): $0 \le v_k \le \max(v_{curve,k}, 1)$. No hard speed limit on straight road ($v_{curve} = 30$ m/s); $V_{lim}$ enters only through $v_{ref}$.
- Gap to the leader (lines 81-96), $T_c = 1.2$ s, $s_{0c} = 2$ m:
$$
s_k + T_c v_k \le s + g + d_l(k\Delta t) - s_{0c}
$$
  braking leader ($a_l < 0$, $v_l > 0$, lines 83-85): $d_l(t) = v_l \min(t, t_s) + \tfrac12 a_l \min(t, t_s)^2$, $t_s = v_l/(-a_l)$. Otherwise (lines 86-92, cap new in 70d8d01), with $\bar a_l = \min(\max(a_l, 0), 1.5)$, $v_{cap} = \max(v_l, V_{lim})$, $t_c = (v_{cap} - v_l)/\max(\bar a_l, 10^{-6})$:
$$
d_l(t) = \begin{cases} v_l t + \tfrac12 \bar a_l t^2 & t \le t_c\\ v_l t_c + \tfrac12 \bar a_l t_c^2 + v_{cap}(t - t_c) & t > t_c\end{cases}
$$
- Signal (lines 98-106), when the engine set the stop at the stop line ($s_{stop,i} \le s_{stop}$, vehicle before the line): with `spat` and red, $s_k \le s_{stop} - 0.05$ only for $t_k < t_{g0}$ (next green start); otherwise for the whole horizon.
- Turn hold (lines 107-116), uncommitted right-turner whose engine stop point is $s_{wait}$: with a fresh (1 s) `time` message with finite `go`, $s_k \le s_{wait} - 0.05$ for $t_k < go$ and always for $k = 1$; otherwise for the whole horizon.
- Other stop points (yielding, lines 119-122): $s_k \le s_{stop,i} - 0.05$ for the whole horizon.
- Arrival-time hold (lines 125-130): a non-R CAV with a `time` message received within 10 s, finite `tgt`, $t < t_{gt}$, $s < s_{pt}$: $s_k \le s_{pt} - 0.05$ for $t_k < t_{gt}$. This applies to shaped, eco and pulled messages alike.
- Position limits as $S_s(m,:)\,U \le lim_m - (s + k\Delta t\, v)$ (lines 131-135).

### 7.4 Solve and fallback

Warm start from the shifted previous solution (lines 137-139). `qp_solve` (`sim/qp_solve.m:5-24`): `quadprog` (200 iterations, tolerances 1e-7), ok if `exitflag > 0` and finite. On failure (lines 141-152): IDM with $a=1.5$, $b=2.5$, $s_0=2$, own $T$, $v_0 = \min(v_{ref,1}, V_d)$, the stop point (and the target point if held) as standing obstacle, curve cap with $b = 2.5$; counted in `qpfail`. Applied acceleration $u_1$; the plan $\hat s = s + k\Delta t v + S_s U$ is stored (lines 153-155).

### Things to check (part 7)

- Fixed in 70d8d01: the accelerating leader is no longer extrapolated beyond $\max(v_l, V_{lim})$ (lines 88-92).
- **Discrete hold.** The hold is imposed only at grid times $t_k < t_{gt}$, so a held vehicle can cross its point up to 0.5 s before the target (now reflected in $s_{ctl} = 0.5$ s).
- No terminal constraint: a vehicle can plan into a state where it needs more than $-6$ m/s² after the horizon; the QP then becomes infeasible and the IDM fallback and stop guard take over. Report `qp_fail_pct` and the stop-guard count per configuration.
- $v_{curve}$ is evaluated at the previous plan's positions (a standard linearization); check jerk at the arc start.
- The speed limit is soft. Check the maximum CAV speed in the logs, especially for pulled vehicles.
- The turn hold uses `fresh` (1 s) for `go`, the arrival-time hold and the engine's go hold use 10 s. With message loss the turner falls back to the whole-horizon hold (safe).
- $T_c = 1.2$ s for every MPC CAV vs. the drawn $T$ for IDM CAVs (part 3).

---

## 8. Metrics: `sim/metrics_compute.m`, `sim/metrics_write.m`, `sim/fuel_rate.m`

### 8.1 Vehicle sets (`sim/metrics_compute.m:57-69`)

arrived = every scheduled vehicle (also still in the entry queue); entered; exited (complete trips); `rt` = exited right-turners; `str` = exited straight vehicles (left-turners are in neither). `n_rt_intent`, `n_rt_done`, `rt_completion` = their ratio, `n_exited`, `n_unfinished`, `n_waiting`.

### 8.2 Fuel (`sim/fuel_rate.m:4-9`, `sim/metrics_compute.m:70-72`)

$$
f(v, a) = \max\Big(b_0 + b_1 v + b_2 v^2 + b_3 v^3 + \max(a,0)(c_0 + c_1 v + c_2 v^2),\ b_0\Big)\ \text{[mL/s]}
$$
$b_0 = 0.156$, $b_1 = 2.450\cdot10^{-2}$, $b_2 = -7.415\cdot10^{-4}$, $b_3 = 5.975\cdot10^{-5}$, $c_0 = 0.07224$, $c_1 = 9.681\cdot10^{-2}$, $c_2 = 1.075\cdot10^{-3}$. Integrated as $f(v^+, a)\Delta t$ per step, plus $f(0,0)(t_{spawn} - t_{arr})$ for the entry queue, plus $f(0,0)(360 - t_{arr})$ for vehicles still waiting at the end (`sim/sim_run.m:127`, `373`).

- `fuel_rt_mL`, `fuel_str_mL`: mean fuel per exited right-turner / straight vehicle [mL].
- `fuel_L100km` (line 72):
$$
\text{fuel}_{L/100km} = 100\,\frac{\sum_{\text{all}} F_i\ [\text{mL}]}{\sum_{\text{entered}} D_i\ [\text{m}]}
$$
  (mL/m = L/km).

### 8.3 Delay (`sim/metrics_compute.m:77-86`)

$$
t_{ff} = \sum_{j} \frac{\Delta s}{\min(V_d, v_{curve}(s_j))},\qquad \text{delay} = t_{exit} - t_{arr} - t_{ff}
$$
over the path samples (one sample more than the path length, +0.25 m). Includes the entry-queue wait. Both times are on the decision clock since 70d8d01. Averaged over exited vehicles. `entry_wait_s` = mean $t_{spawn} - t_{arr}$ of entered vehicles; `n_blocked` = late entries plus never entered.

### 8.4 PET, conflicts and collisions (`sim/metrics_compute.m:96-145`)

For every entered right-turner $u$ and every `oppZ` row, occupancy intervals from the log by linear interpolation (`occupancy`, lines 188-194; `cross`, lines 196-202):
$$
a_0 = \text{first } t: s_u(t) \ge z_{0k},\ a_1 = \text{first } t: s_u(t) - L \ge z_{1k};\qquad b_0, b_1 \text{ likewise for vehicle } j \text{ on } q
$$
($a_1$ = last log time if still inside at the end). For every entered $j$ on $q$:
$$
g = \begin{cases} 0 & b_0 < a_1 \wedge a_0 < b_1\\ b_0 - a_1 & b_0 \ge a_1\\ a_0 - b_1 & \text{otherwise}\end{cases}
$$
PET of the turn = $\min(\min_j g, 30)$, only if the turner entered a zone. `n_conflict` = turns with PET $< 1.5$ s (line 144); `n_collision` = turns with PET $= 0$ (zone overlap at 2.6 m center distance, not a body-contact test). Pairs with $g < 3$ s go to `pet_detail`.

### 8.5 Comfort and stops

- `jerk_rms` (line 89): $\sqrt{\sum_{i\in ent}\sum_k ((a_k - a_{k-1})/\Delta t)^2 / \sum_{i\in ent} n_{steps,i}}$, with $a_0 = 0$ at entry and realized accelerations (after the guards).
- `stops_per_veh` (line 90), `gap_rt_s`, `red_rt_s`, `red_str_s` (lines 73-75), `qp_solves`, `qp_fail_pct`, `guard` (lines 91-94).

### 8.6 Shaping metrics (`sim/metrics_compute.m:147-180`)

`n_pulled`, `pull_mean_s` (largest requested gain per vehicle); `n_eco`; `n_shaped`, `shape_mean_s`, `shape_max_s` over records with final $t_{gt} - nat \ge 0.3$ s; `shape_decel` = largest $-a$ of a shaped vehicle while $t_0 \le t \le t_{gt}$, $s < s_{stop} - 20$, with no vehicle of the same approach within 30 m ahead (lines 163-178).

### 8.7 CSV (`sim/metrics_write.m:5-31`)

One row per run, `%.4f` for every metric; an older header renames the file to `*_oldformat.csv`.

### Things to check (part 8)

- **Survivor bias**: fuel per trip, delay, stops and idle times are averaged over exited vehicles only. A configuration that leaves more vehicles inside or in the entry queue at 360 s looks better. Report `n_unfinished`, `n_waiting`, `n_blocked` next to delay, or add total time spent by all arrived vehicles up to the run end.
- The free-flow reference uses each vehicle's own $V_d$, capped at 13.89 m/s for CAVs; in C1 the same vehicle may have $V_d = 14.4$ m/s. Total travel time is a cleaner cross-configuration comparison.
- `n_collision` is zone overlap, and the yielding rule prevents many overlaps by braking the oncoming vehicle. Match the paper wording.
- PET covers right turns against the opposite straight and left movements only.
- `fuel_L100km` mixes fuel of vehicles still in the system with distance of entered vehicles only.
- Fixed in 70d8d01: the conflict criterion in `function/plot_sweeps.m:27-28` now says PET $< 1.5$ s, matching line 144.

---

## 9. Study runners and scripts

### 9.1 Scripts in `main/`

- `main_train.m` (`main/main_train.m:14-21`): `train_estimators(1:20, 4, 720)`.
- `main_one.m` (`main/main_one.m:19-35`): one configuration (default C4, seed 101), row with an empty study name.
- `main_animation.m` (`main/main_animation.m:18-41`): `watch_main` for a list of configurations, nothing saved.
- `main_figures.m` (`main/main_figures.m:10-20`): `make_figures` and `plot_sweeps`.
- `main_all.m` (`main/main_all.m:18-65`): optional fresh start, training on seeds 1-20, main study `run_ablation(101:120, 720, true)`, figures, scenarios, sweeps on seeds 201-210, sweep figures.

### 9.2 Runners in `function/`

- `run_ablation.m` (`function/run_ablation.m:25-68`): seeds 101-120, 720 steps, the 12 configurations C1, V2V_MPC, C2, C3, C4, C4_NOUNC, C4_NOONLINE, C4_SPEED, C4_STRICT, C4_SEQ, C4_NOADV, C4_ONCE on the same traffic per seed; rows tagged `study=ablation`; optional `est_eval_run`; then `summarize_ablation`.
- `summarize_ablation.m`: last row per (study, point, seed, label) (`function/read_runs.m:13-17`), only `study=ablation` rows (lines 21-24). Per configuration and metric: $n$, mean, std, 95% CI half-width $t_{0.975, n-1}\,s/\sqrt n$ (lines 34-49). Paired comparisons B minus A on common seeds (lines 52-96): mean difference, its 95% CI, $100\,\bar d/\bar x_A$, Wilcoxon signed-rank $p$, paired $t$-test $p$; 18 pairs, 19 metrics.
- `sweep_definitions.m` (`function/sweep_definitions.m:24-64`), `const` schedule (demand 0.4 unless swept):

| study | values | configurations | reference |
|---|---|---|---|
| demand | 0.1 to 0.8 veh/s | C1, C2, C3, C4 (+ estimators) | |
| rtshare | `rt` 0.05 to 0.70 (axis x75 = % of arrivals) | C1, C2, C3, C4 | |
| latency | 0, 0.5, 1, 1.5, 2 s | C3, C4 | C1 |
| loss | 0 to 0.6 | C3, C4 | C1 |
| penetration | 0 to 1 (`sense=v2x`) | C2, C3, C4, C4_STRICT | C1 |
| dmax | 0, 0.5, 1, 2, 3 s | C4, C4_SEQ | C1 |
| driver | aggressive, nominal, cautious | C1, C3, C4, C4_NOONLINE (+ estimators) | |
| asym | 0.5, 0.65, 0.8 at demand 0.5 | C1, C3, C4 | |

- `run_sweeps.m` (`function/run_sweeps.m:28-108`): seeds 201-210, resumable, then `plot_sweeps`; the study list in the header now includes `dmax` (lines 9-10, fixed in 70d8d01). `plot_sweeps` writes mean and 95% CI per point and failure limits (`function/plot_sweeps.m:106-129`): completion $< 0.8\times$ base, gap wait $> 120$ s, blocked $\ge 1$, conflicts (PET $< 1.5$ s) $\ge 1$ per run.
- `run_scenarios.m` (`function/run_scenarios.m:16-52`): scenarios A and B, seed 1, 360 steps (180 s), C1, C2, C3, C4, logs saved.
- `make_figures.m` (`function/make_figures.m:19-66`): representative seed = C4 run whose `fuel_rt_mL` is closest to the median; reruns C1, V2V_MPC, C2, C3, C4 without logging, plus the estimator comparison for that seed.

Seeds: training 1-16, estimator test 17-20, main study 101-120, sweeps 201-210, scenarios 1 (fixed arrivals; the seed only sets the noise streams). The random-traffic seeds are disjoint.

### Things to check (part 9)

- 18 comparisons x 19 metrics without a multiple-comparison correction; state which comparisons are confirmatory.
- $n = 20$ seeds (10 per sweep point); for count metrics that are often 0 (`n_conflict`, `n_collision`) the tests are not informative.
- The representative seed is chosen on C4's fuel only.
- `main_one.m` writes to `output/ablation_runs.csv` with an empty study; `summarize_ablation` drops such rows only if `ablation` rows exist (line 22).
- "Last row wins" (`function/read_runs.m:16`): a partial rerun silently replaces rows. After the 70d8d01 clock change, delete old results and retrain before running the study.

---

## 10. Open questions for the reviewer

Ordered by how much they could change the conclusions. Each is still open at `e3d786a`.

1. **Time headway differs between the IDM-based baselines and the MPC configurations** (`sim/sim_run.m:272` vs `sim/ctrl_mpc.m:50`). IDM CAVs (C2, V2V_IDM) keep the drawn $T \sim U[1.2, 2.4]$ s (mean 1.8 s); every MPC CAV (C3, C4 and its variants, V2V_MPC) uses $T_c = 1.2$ s as a hard constraint. Discharge capacity therefore differs regardless of coordination, and the C2 vs C3 and C2 vs C4 differences mix the controller, the headway and the method. Either give both controllers the same headway or report V2V_IDM vs V2V_MPC (in `summarize_ablation`, but V2V_IDM is not in the default run list) to isolate it.
2. **Safe-side oncoming bounds make the geometric margins conservative, and that is a tunable choice** (`sim/rsu_plan.m:211-225`). Since 979b6c2 and e3d786a every oncoming vehicle enters the zone at full speed and leaves from its present speed, or from rest when it is slow now or predicted to stop first. $D^a$ is the largest the geometry allows and $D^b$ is large for slow vehicles, which leaves fewer feasible slots and fewer shaping and pulling opportunities in C4, on top of $PET = 1.5$ s and the $z\sigma$ term. The size of C4's gain over C4_STRICT, C2 and C3 therefore depends on this choice. Options to state or test: the bounds as now, a predicted stop-line speed (for example from $d/lo$), or a sensitivity run with both. Vehicles that stop only inside the box without a late stop-line prediction are still not covered; safety there rests on $u$ and the on-board check.
3. **Estimator trained on human traffic, used on CAV traffic** (`function/train_estimators.m:40`), and **which arrivals the online learner sees** (`sim/rsu_crossed.m:33-38`: vehicles delayed by 0.3 s or more, or pulled, are excluded; holds at the natural time and eco vehicles are kept since b3dfe15). Part of the C4 vs C3 gain may be online adaptation to the CAV regime rather than better modeling. Eco and held arrivals are themselves shaped by the MPC's arrival-time hold, so they are not fully natural either.
4. **Fixed 4 s baseline margin, eco messages and replanning in C2/C3** (`sim/rsu_plan.m:232`, `sim/sim_config.m:48`, `sim/rsu_plan.m:106-111`). Baseline plans can be geometrically infeasible and are rescued by the on-board check; C2/C3 also get eco times and replan when a new turner comes into range. Confirm each against [5] and the CISE paper, or state the deviations.
5. **Optimality wording** (`sim/rsu_plan.m:245-254`, `405-416`, `261-264`). The joint search is exhaustive over pre-filtered candidate slots, with all-or-nothing pulling, and falls back to the successive search above 5000 combinations. Say "exhaustive over candidate slots" unless the pruning is made exact.
6. **Latency model** (`sim/rsu_receive.m:7`, `sim/rsu_view.m:12`, `sim/sim_run.m:160`). Uplink latency is cancelled by constant-speed dead reckoning; only command delay is tested. Describe the latency sweep accordingly.
7. **Metrics with survivor bias and different free-flow references** (`sim/metrics_compute.m:79-86`). Delay and fuel per trip average over exited vehicles; the free-flow time uses each vehicle's own $V_d$, capped for CAVs. Add total time spent by all arrived vehicles as a robustness metric.
8. **Safety metric definitions** (`sim/metrics_compute.m:125-145`, `sim/sim_run.m:214-237`). Collisions are zone overlaps, and the yielding rule turns wrong turn decisions into oncoming delay. Count yield events and "could not stop" commits (`sim/sim_run.m:206`).
9. **`idm` estimator naming** (`sim/rsu_estimate.m:53-56` vs `function/train_estimators.m:93`). The holdout table and the per-run estimator table use different "idm" estimators.
10. **Human gap acceptance kinematics** (`sim/turn_gap_ok.m:55-56`, `90`). $t_{clear}$ assumes 1.5 m/s² for every turner; human turners drive with $a \in [1, 2]$ and noise. The C1 baseline's safety depends on this.
11. **Smaller items**: plans that send nothing are still counted in `rsu.plans` and `rsu.diag` (`sim/rsu_plan.m:283-293`); the "will stop first" test classes shaped vehicles and distant left-turners as late (`sim/rsu_plan.m:223`, safe side); non-physical guards and the unreported stop-guard count (`sim/sim_run.m:298-318`, `sim/metrics_compute.m:94`); lost messages that the plan relied on (`held` vehicles, part 6).

Fixed in 70d8d01: decision/log clock offset; left-turner crossing speed; pull threshold at the start of `evalSlots`; no-slot turners removed before messages; $s_{ctl} = 0.5$ s; MPC leader speed cap; 5000-combination fallback; comment fixes in `est_features`, `turn_gap_ok`, `gp_update`, `gp_fit`, `gp_calibrate`, `plot_sweeps`, `run_sweeps`, `geom_build`, `abl_settings` and `README.md`; latency sweep in whole steps.

Fixed in b3dfe15: kinematic oncoming crossing times (queued straight vehicles in $D^b$); pulled vehicles must gain at least 0.3 s (no pull without a message); no vehicle messages when the no-slot re-evaluation is infeasible; diagnostics counter computed before the re-evaluation and with the original candidate sets; `once` mode marks all in-range turners (no every-step replanning); online learning skips only delayed or pulled vehicles; schedule built on the decision clock ($t = (kk-1)\Delta t$, scenario index `round(t/dt)+1`) and the `sim_run.m` header comment.

Fixed in 979b6c2: oncoming entry at full speed and exit from rest below 5 m/s (no more optimistic $D^a$ for vehicles that are stopped now); no messages at all when the plan without no-slot turners is infeasible; pull message threshold $x \le lo - 0.29$, equal to the feasibility rule.

Fixed in e3d786a: oncoming exit time from rest when the vehicle is predicted to stop first; left-turners at arc speed no longer forced to start from rest; `once` mode retries after an inconsistent plan.
