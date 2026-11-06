# 06 — FEW-SHOT META-LEARNING FOR TERRAIN ADAPTATION

**Project:** Terrain-aware path planning for wheeled UGVs (MEG)  
**Research cutoff:** 06/11/2026 (Asia/Bangkok)  
**Status:** *Executable CPU PyTorch synthetic pilot*; **not** a real-dataset experiment, full published-method reproduction, nor a field-robot safety demonstration.  
**Linked sources:** `02_research_gap_and_novelty.md`, `03_problem_formulation.tex`, `04_dataset_and_benchmark.md`, Phase-05 `terrain_ugv_baseline.zip` (the Phase-05 planner is reused without modifying its A* algorithm).  
**Deliverables:** this document; `terrain_ugv_phase06/` package, tests, fixed config, CSVs, and learning/adaptation curves.

## Executive decision

**Choose at most two methods for the main pilot:** **(M1) First-Order MAML (FOMAML)** and **(M2) learned amortized terrain context**. Their input is an action-conditioned interaction history and their predicted outputs are a *synthetic, observed-response proxy* of longitudinal slip and yaw tracking shortfall. Compare against **no adaptation**, **ordinary full-network fine-tuning**, and **head-only self-supervised test-time update**. A simple fixed Phase-05 terrain-aware A* bridge serves only as an *offline route-risk audit*. **The proposed novelty of selective risk-calibrated adaptation has NOT been implemented/tested in this stage.**

**Measured results in synthetic test (K=5):** FOMAML slip MAE 0.1084, context 0.1277, ordinary fine-tune 0.1711, no adaptation 0.3164. But all five methods achieve the same 8/8 oracle-audited route-safety proxy on each K; there is **no observed planning advantage or uncertainty-calibration guarantee**. Pilot verdict: **GO** for replacing synthetic data with real, audited wheeled-UGV action–state trajectories and a proper physics closed-loop benchmark; **NO-GO** for claiming a Q1 scientific contribution or proceeding to a larger world-model/MoE stack based on these numbers alone.

## 1. Algorithm selection audit

| Approach | Inner/online adaptation | Advantages | Cost / limitations | Decision |
|---|---|---|---|---|
| **MAML** (Finn et al., ICML 2017) | Few gradient steps on support; differentiable outer loop through steps | Learns initialization explicitly optimized for quick adaptation | Higher-order derivatives, memory; strong hyperparameter sensitivity; no intrinsic terrain physics or safety | Theory comparator only |
| **FOMAML** | Same support update; ignores Hessian terms in meta gradient | Meta-trains **query-after-support** objective; implementation fits CPU; direct continuous regression | Approximate gradient; extra offline training; uncalibrated point estimates | **Selected M1** |
| **Reptile** (Nichol et al., 2018) | Repeat task SGD then move initialization toward adapted weights | Very simple first-order, efficient | Meta-objective implicit; task-internal batch structure matters; no dedicated uncertainty | Deferred alternative; not selected |
| **ANIL** (Raghu et al., ICLR 2020) | Inner loop only for prediction head | Reduces embedded update load, stresses feature reuse | Fixed features can fail badly under physics change | Not a separately meta-trained variant in this pilot; `tta_head` is **not ANIL** |
| **Prototypical Networks** | Class-prototype inference from labeled examples | Good for **discrete terrain classes** | Slip/dynamics are continuous action-conditioned regression; classification prototypes cannot be read as wheel–soil interaction parameters | Excluded as primary |
| **Latent/context adaptation** | Encode support transitions to a task vector; no target gradient | Very fast online update; amortized in-distribution inference | Requires diverse meta-training tasks; meta-test extrapolation may be poor | **Selected M2** |

**Definitions — do not mix labels:**

- **Transfer learning**: source training followed by downstream learning; **not automatically meta-learning**.
- **Ordinary fine-tuning**: minimize observed support loss starting from source-pretrained weights, without source training optimized for post-update query performance.
- **Meta-learning**: source training explicitly uses many source **tasks** and either support→query optimization (FOMAML) or learning a support→task-context inference mechanism (M2).
- **Test-time adaptation (TTA)**: updates/adjustments on test-time observations. If an update truly minimizes a loss from new observed state transitions, it **learns**, possibly self-supervised; fixed uncertainty-threshold adjustment without learned parameters is **not few-shot learning**. In this pilot `tta_head` is head-only learning on support action–response pairs, not unlabeled image-only TTA.
- **FS-SUP vs FS-SELF**: this pilot assumes the robot knows the actions and can observe subsequent odometry/proprioceptive response, so no *manual slip annotations* are used. It remains **simulated**, and is classified as *self-supervised transition adaptation* rather than supervised terrain-class few-shot classification.

### Closest related research (not claimed as reproduced)

1. **METAVerse** — Seo et al., *IROS 2024*, DOI `10.1109/IROS58592.2024.10802444`, [paper](https://arxiv.org/abs/2307.13991). Dense continuous traversability costs from LiDAR + self-supervised interaction feedback; meta-training and online adaptation; MPC demonstration. This is **a stronger application-level comparator** than a generic image classifier. Official full experimental stack, terrain labels and exact training data were **not independently reproduced** here.
2. **MAGIC-VFM** — Lupu et al., *IEEE T-RO* 41:180–199 (**2025 issue**, online 2024), DOI `10.1109/TRO.2024.3475212`, [IEEE source](https://ieeexplore.ieee.org/document/10706036). Meta-trained residual interaction model, foundation visual features, last-layer online adaptation and composite adaptive controller; tracked and car-like tests. Different task/guarantees; **we have no equivalent controller guarantees**.
3. **Levy et al.** — *RSS 2025*, DOI `10.15607/RSS.2025.XXI.139`, [proceedings](https://www.roboticsproceedings.org/rss21/p139.html). Meta-learned basis/adaptation parameters + online Kalman filter for off-road vehicle dynamics; full-scale vehicle evidence. A **strong external baseline** to reproduce after importing synchronized action–state data.
4. **MAML** — [PMLR ICML 2017](https://proceedings.mlr.press/v70/finn17a.html); **Reptile / FOMAML** — [Nichol, Achiam & Schulman 2018](https://openai.com/index/on-first-order-meta-learning-algorithms/); **ANIL** — [ICLR 2020](https://iclr.cc/virtual_2020/poster_rkgMkCEtPB.html).

## 2. Formal terrain task distribution

A domain/task is **not merely a terrain class label**. Define

\[
\mathcal{T}_i=(d_i,\nu_i,\mathcal{D}^{\mathrm{support}}_i,\mathcal{D}^{\mathrm{query}}_i),
\quad d_i=(\text{terrain family, contact regime, vehicle profile}),
\]

where \(\nu_i\) is a task-specific **hidden interaction parameter** (only simulator truth), and each transition uses the **same, explicitly identified wheeled platform**. Moving from a differential-drive to an Ackermann or skid-steer vehicle must define a new model and benchmark track; our pilot does **not** model those distinctions physically.

**Observations and targets** for interaction index \(t\):

\[
\begin{aligned}
x_t^{\mathrm{obs}} &= [\tilde c_t,\,s_t,\,r_t,\,v_t^{\mathrm{cmd}},\,|\omega_t^{\mathrm{cmd}}|],\\
y_t^{\mathrm{obs}} &= [\tilde\sigma_t,\,\widetilde{\Delta\omega}_t],\quad
\tilde\sigma_t\approx \operatorname{clip}\left(\frac{v_t^{\mathrm{cmd}}-v_{t+1}^{\mathrm{meas}}}{\max(|v_t^{\mathrm{cmd}}|,\epsilon)},0,1\right).
\end{aligned}
\]

Here appearance cue \(\tilde c\), normalized slope \(s\), roughness \(r\), and commanded motions are available to the model; the response target is reconstructed *after* the action. The expression above is the **proposed real-data interpretation**. The shipped synthetic generator directly simulates noisy response-target proxies; it is **not a wheel encoder or inertial sensor calibration model**. No true friction coefficient, task parameters, future query outcome or simulator oracle hazard is fed to the learner.

For the synthetic pilot, a task-specific 6-parameter sigmoid law generates slip:

\[
\sigma_t=\mathrm{sigmoid}\!\left(b_i+a_i s_t+b_i' r_t+c_i v_t+d_i|\omega_t|+e_i(1-\tilde c_t)+0.9v_t(1-\tilde c_t)\right),
\]

plus deterministic-seeded noisy observations. This equation is **the pilot's invented data generator, not a terramechanics equation validated on soil**. It deliberately omits sinkage, load transfer, braking, lateral slip, steer angle, powertrain limitations, deformable tire/contact dynamics and real navigation state.

**Frozen synthetic family split:**

- Meta-training: `hardpack`, `dry_grass`, `gravel`, `dirt`.
- Validation only: `dry_sand`.
- Unseen synthetic meta-test: `wet_mud`, `wet_clay`.
- For actual field data: **site / route-overlap cluster / date / run first**, then create disjoint support and query windows; embargo **60 seconds** and keep all repeated-route derivatives in the same outer split. This pilot has only **independently seeded synthetic support/query runs**, not actual independent physical sites.

## 3. Exact few-shot protocol / data budget

**Phase-04 preregistered unit retained:** **1 shot = 1 contiguous support window of 2.0 seconds sampled at 10 Hz = 20 transitions**. A shot is not 1 RGB image. The simulator generates 50 support windows (1,000 transitions) and a **separate 12-window / 240-transition query run per task**. Only the prefix of \(K\) support windows is visible to a method. The target task's query never enters online adaptation.

| K | Support transitions | Support time | Status |
|---:|---:|---:|---|
| 0 | 0 | 0 s | Zero-shot control |
| 1 | 20 | 2 s | Primary |
| 2 | 40 | 4 s | Primary (phase-04 consistency) |
| 5 | 100 | 10 s | Primary |
| 10 | 200 | 20 s | Primary |
| 20 | 400 | 40 s | Primary |
| 50 | 1,000 | 100 s | **Explicit extended-budget track**, not the phase-04 primary track |

In this **offline** pilot, each K consumes already-simulated transitions: **zero new real robot interactions**, zero human labels, and no field traversability certification. Real online evaluation must include physical cost/risk of gathering support (including failed probing) in its own ledger.

**Protocol:** separate source/validation/test families before constructing windows; use SHA256-derived seeds keyed by family, task, and `support_run`/`query_different_run`; reset model for each task and each K; reuse the **same** supports and queries for every method; evaluate every prescribed K (not the best K); do not inspect test outcomes to select training hyperparameters. In the supplied pilot, learning rates/steps were *fixed rather than tuned*, and comparison against published SOTA remains pending.

## 4. Mathematical derivation

Define a predictor \(f_\theta(x)\in[0,1]^2\) and regression loss

\[
\ell_D(\theta)=\frac{1}{|D|}\sum_{(x,y)\in D}\|f_\theta(x)-y\|_2^2.
\]

**FOMAML:** in source task \(\mathcal{T}_i\), perform \(m\) steps

\[
\theta'_i=U^m_{S_i}(\theta),\qquad
U_{S_i}(\theta)=\theta-\alpha\nabla_\theta \ell_{S_i}(\theta).
\]

Meta objective (source tasks only):

\[
\min_\theta\mathbb{E}_{\mathcal{T}_i\sim p_{\rm src}(\mathcal{T})}
\left[\ell_{Q_i}(U^m_{S_i}(\theta))\right].
\]

First-order approximation discards second-order Jacobian terms:

\[
\nabla_\theta \ell_{Q_i}(\theta_i')\approx \nabla_{\theta_i'}\ell_{Q_i}(\theta_i'),
\quad\theta\leftarrow\theta-\beta\frac1B\sum_i\nabla_{\theta'_i}\ell_{Q_i}(\theta'_i).
\]

At unseen test task \(\mathcal{T}^*\): \(\theta^*_K=U^m_{S_K^*}(\theta_{\rm meta})\); evaluate \(\ell_{Q^*}(\theta^*_K)\) **without gradient update using \(Q^*\)**.

**Amortized context:** a network learns a source-trained functional summary from action–response support:

\[
\hat c_{S}=g_\phi\Big(\mathop{\mathrm{mean}}_S x,\,\mathop{\mathrm{mean}}_S y,
\,\mathop{\mathrm{mean}}_S (x\,\tilde\sigma),\,\mathbf{1}[K>0]\Big),
\quad\hat y=h_\psi(x,\hat c_S).
\]

Outer objective:

\[
\min_{\phi,\psi}\mathbb E_{\mathcal T\sim p_{\rm src},K\sim p(K)}
[\ell_{Q_\mathcal T}(h_\psi(x,g_\phi(S_K)))].
\]

At meta-test the feature/context networks are **frozen**; the target support controls the context, not the model weights. \(K=0\) uses an all-zero context summary. Learning a support→context function across source tasks **is a form of amortized meta-learning** even though target-time context inference is gradient-free.

**Output to downstream planning:** predicted longitudinal slip ratio \(\hat\sigma_{t:t+H}\) or a spatial proxy \(\hat\sigma(c,a)\), possibly transformed to nonnegative costs, speed limits and an *estimated* forbidden mask. **No probabilistic uncertainty distribution is learned here.** This stage does not claim calibrated confidence intervals, risk probabilities, chance constraints, physical safety certificates or a multistep world model.

## 5. Training, testing and leak-safe pseudocode

```text
source_tasks, val_tasks, sealed_test_tasks = group_split_by_terrain_and_site()
# Source preprocessing only; no normalization statistics estimated from test data
for outer_step in range(meta_steps):
    for source_task in sample(source_tasks, B):
        S, Q = disjoint_support_query(source_task)
        adapted = copy(meta_initialization)
        for step in range(inner_steps):
            adapted = SGD(adapted, loss_on(S))
        outer_grad += grad(loss_on(Q), wrt=adapted)  # FOMAML first order
    meta_initialization = optimizer_step(outer_grad)

# For each test task and K in [0,1,2,5,10,20,(50 extended)]:
S_K = first_K_complete_windows(target_support_run)
learner = fresh_copy(meta_initialization)
learner = update_from(S_K)                # never from test Q
prediction = learner(target_query_features) # NO target Q labels used in adaptation
log_query_error(prediction, target_query_labels)
log_offline_Astar_route_audit(optional_fixed_planner)
```

Context-based meta training replaces inner updates by learned \(g_\phi(S)\) and query-prediction gradients with respect to \(\phi,\psi\) on **source tasks only**. Each meta-test K starts from the same frozen source-trained checkpoint.

## 6. Executable PyTorch pipeline and Phase-05 integration

```text
terrain_ugv_phase06/
├── README.md
├── requirements.txt
├── configs/protocol.json
├── src/phase06/core.py             # split, synthetic tasks, FOMAML, context, FT, TTA
├── src/phase06/experiment.py       # fixed-K evaluation, Phase-05 A* bridge, exports
├── vendor_phase05/terrain_baseline # unchanged stage-05 A* planner/model subset
├── tests/test_protocol.py          # 12 unit/protocol/determinism tests
├── analyze_results.py             # seed-family cluster paired bootstrap
└── results/pilot_v02/
    ├── episode_metrics.csv
    ├── adaptation_curves.csv
    ├── negative_transfer_paired.csv
    ├── training_curves.csv
    ├── run_metadata.csv
    ├── validation_calibration_transfer.csv
    ├── paired_bootstrap.csv
    ├── manifest.json
    ├── learning_curves.png
    ├── adaptation_curves.png
    └── route_proxy_curves.png
```

The experiment code uses **CPU PyTorch**, `numpy`, `pandas`, `matplotlib`, and carries an exact copy of Phase-05 A* and two associated data types. It does not install ROS2 or run a deformable-soil simulator. Data loader is a **task generator**; real TartanDrive `.bag` parsing has **not** been built. The Phase-05 terrain-aware A* is held constant across all methods and receives only observed proxy features and method prediction; **ground-truth terrain interaction parameters are only accessed by the scorer after the path is selected**.

Reproduce this run on Fedora/Linux:

```bash
cd terrain_ugv_phase06
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
PYTHONPATH=src:vendor_phase05 pytest -q tests
PYTHONPATH=src:vendor_phase05 python -m phase06.experiment \
  --out results/pilot_v02 \
  --seeds 41,42 --source-steps 300 --meta-steps 100 \
  --context-steps 450 --n-train 10 --n-val 4 --n-test 5 --extended-k50
python analyze_results.py
```

**Test status:** 12/12 tests pass. **Seeds:** 41,42; source tasks = 4 families × 10 tasks × 2 seeds = 80 independently generated tasks across the two training repetitions; validation = 4 × 2 = 8 tasks; test = 2 families × 5 tasks × 2 seeds = 20 test tasks. Each task is evaluated at 7 K values and 5 methods. **Total 980 evaluation rows** (280 validation + 700 test), including **280 separate offline A* route audits** on the first two synthetic test tasks per family/seed × 7 K × 5 methods.

**Training budgets are NOT compute/data matched:** source pooled training = 300 optimizer steps per seed; FOMAML = 100 outer steps × 3 source tasks × 3 inner updates (plus query evaluation); context meta-training = 450 outer steps per seed. Even if all methods use source **family membership** only, the number of training examples, optimization passes and total compute differ. Thus this pilot **does not isolate meta-learning algorithmic superiority** against a properly tuned, compute-matched dense/fine-tuning baseline.

## 7. Measured synthetic results: adaptation curves

**MAE (lower is better) on unseen synthetic `wet_mud`, `wet_clay`**. These are observed slip-*proxy* errors, not actual measured wheel slip errors on TartanDrive or a robot.

| Method | K=0 | K=1 | K=2 | K=5 | K=10 | K=20 | K=50* |
|---|---:|---:|---:|---:|---:|---:|---:|
| Never adapt | 0.3164 | 0.3164 | 0.3164 | 0.3164 | 0.3164 | 0.3164 | 0.3164 |
| Ordinary fine-tune | 0.3164 | 0.1793 | 0.1712 | 0.1711 | 0.1677 | 0.1654 | 0.1673 |
| TTA head-only | 0.3164 | 0.2518 | 0.2496 | 0.2504 | 0.2490 | 0.2480 | 0.2491 |
| **FOMAML** | 0.2755 | **0.1218** | **0.1105** | **0.1084** | **0.1036** | **0.1005** | **0.1022** |
| **Amortized context** | **0.2351** | 0.1344 | 0.1277 | 0.1277 | 0.1245 | 0.1223 | 0.1233 |

\* K=50 is an extended-budget sensitivity test outside primary phase-04 K set.

For K=5, **FOMAML - ordinary fine-tune = −0.0627 MAE**, roughly **36.6% lower** relative to fine-tune. A simple seed×terrain-family cluster bootstrap (4 groups only) estimates 95% percentile interval **[−0.0661, −0.0593]** for the synthetic task average difference. **This CI is descriptive only:** 4 seed–family clusters are far too few to establish real generalization or statistical independence between nearby terrain samples. Source training and parameter tuning are not compute matched, so this is **not strong evidence of a publishable method advantage**.

**Online adaptation plus query inference CPU latency, K=5 (median per task, current container):** no-adapt ~0.26 ms, full FT ~4.85 ms, head-only ~4.10 ms, FOMAML ~4.78 ms, context ~0.54 ms. They exclude sensor processing, vehicle control, safety checking, and device-to-device transfers; **do not quote as onboard ROS2 control frequency**. Context is fastest at this stage because it does not run SGD at target time.

The source/meta-training curves are **not directly comparable losses**: base and context source query MSE differ in task sampling; FOMAML records query loss *after* support inner updates. The plot documents optimization behavior, not a fair same-compute learning-rate comparison.

## 8. Prediction harm, uncertainty and planning outcome

### 8.1 Negative transfer and catastrophic forgetting

`negative_transfer_paired.csv` compares every adapted method's test task slip MAE with **never-adapt on the same task and K**. Observed synthetic *prediction harm rate* was **0/20 tasks** at every tested K (all adapted methods). **This is not evidence that negative transfer is solved:** the synthetic family distribution and observation noise are too simple to reliably trigger harmful updates. Need instrument-support corruption, label delay, partial contact shifts, distribution reversals, and hard-to-identify friction regimes; the present pilot must not be used to justify a selective-update gate.

Because meta-test models are always **reset between tasks** and this experiment does not sequentially adapt one persistent model across several unrelated terrain environments, **catastrophic forgetting was not tested**. No rollback or adaptation acceptance gate has been implemented. These are high-priority stage-07 scientific obligations.

### 8.2 Uncertainty calibration

The `Predictor` and `ContextPredictor` output **point values**, not aleatoric/epistemic variances or event probabilities. Therefore **ECE, Brier score for calibrated event probabilities, NLL and guaranteed chance-constraint coverage are N/A**, not zeros. `validation_calibration_transfer.csv` stores a **validation-only 90th-quantile threshold of the *task-mean absolute error*** and checks if a test task's mean error is below it. At K=5, FOMAML passes this crude test for **11/20 tasks (55%)**, context **14/20 (70%)**. This diagnostic is **not** per-transition predictive interval coverage or a valid conformal prediction guarantee. Poor transfer highlights terrain-shift calibration risks, but does not alone quantify physical accident probabilities.

### 8.3 Planner coupling: strongest negative result

The **same Phase-05 8-connected terrain-aware A*** is used for all methods. It consumes a risk map derived from model-predicted slip (and a fixed, common hard-ground source prior when a visible cue indicates hard ground). A held-out simulator formula determines true terrain-specific slip **after** path selection. This is an **offline path risk audit**, not execution, not action-conditioned MPC, and not dynamic constraint validation.

**Result:** At every K in the pilot, **all five methods selected an oracle-audited safe path in all 8 audited synthetic tasks** (8/8 per method/K); all methods had 0 unsafe planned paths. **No gain in path safety, planning success, energy or rollout quality was demonstrated**. This indicates the current route benchmark is **too easy/saturated** to discriminate. Report actual robot/planner success, collision, stuck, slope, tracking, travel time and unsafe-event rates as **N/A** — do **not** substitute this 100% oracle path proxy for actual closed-loop mission success.

**Planning-specific failure criteria for next iteration:** compare at equal mission completion, report path length/travel time and abstention, include physically ambiguous areas where choosing a shortcut trades risk against distance, use Gazebo/Chrono action-conditioned soil simulations, and ensure baseline risk thresholds are tuned only on validation. Any method must improve **actual** closed-loop outcomes at matched compute before it supports a planning novelty claim.

## 9. Strongest comparator and reproduction ledger

| Target / method | Published claim | What this project actually performed | Reproduction status |
|---|---|---|---|
| METAVerse (IROS 2024) | Meta-learned traversability estimation, interaction adaptation, MPC | **No** original cost-map network, LiDAR dataset, or MPC; compare as closest prior art only | **Not reproduced** |
| MAGIC-VFM (T-RO 2025) | Meta-learned residual dynamics + online controller; physical tests | No original VFM/control-law setup | **Not reproduced** |
| Levy et al. (RSS 2025) | Meta-learned basis + online Kalman dynamics adaptation on off-road car | No original Kalman adaptation or vehicle logs | **Not reproduced** |
| Phase-05 fixed terrain-aware A* | Synthetic geometry + risk weighted global planning | **Reused** exact 8-connected A* implementation and RiskMap interface | **Code-level reused, not experimental reproduction** |
| Strongest **implemented** supervised baseline | Source-pretrain + full support fine-tune | K=5 test MAE 0.1711; FOMAML 0.1084; context 0.1277 | **Measured synthetic-only**, **unmatched source compute** |

This project **cannot** claim to beat METAVerse, MAGIC-VFM, or Levy because their code/hardware/datasets/metrics are not matched, and it cannot assert a quantitative off-road improvement over published methods.

## 10. Go/no-go criteria for an SCI(E) Q1 research contribution

**Engineering GO (pilot):** a runnable and tested meta-training / meta-test scaffold exists; support–query separation, K budgets, CSV logs and learning curves are in place; FOMAML/context improve a synthetic continuous interaction prediction proxy against preliminary baselines.

**Scientific NO-GO (today):** no real TartanDrive 1.0/2.0 ingestion or site-disjoint held-out evaluation; no slip GT verification; source compute not matched; no independent simulation of wheel–soil deformation or closed-loop planning; prediction-only improvement; no calibrated uncertainty distribution; no catastrophic forgetting or genuine negative-transfer case; no METAVerse/Levy replication.

**Mandatory next-phase gates (do not quietly edit benchmark v0.1 after model tuning):**

1. **Data authenticity:** ingest at least one verified TartanDrive set with action, odometry/IMU, wheel-speed topics; publish timestamp alignment/QC and group/route-separated splits. Never call a semantic pixel label a physical slip label.
2. **Fair baseline:** equalize pretraining data, source updates, architecture capacity, support transitions, adaptation steps and CPU/GPU budget; include source pretrain, tuned FT/head-only, a Kalman residual dynamics baseline and a METAVerse-like traversability adaptation if actually reproducible.
3. **OOD evidence:** at least three independent target terrain/site families, 5 training seeds (minimum pilot, not power justification) and route-level cluster CI; add sensor corruption, partial support bias and contact transition shifts to reveal negative transfer.
4. **Safety link:** use the frozen `SHIFT-PHYS-CL`/`SHIFT-VIS-CL` stage-04 benchmark with action-conditioned vehicle simulation; report success, failure/constraint events, stuck, slip, time, distance and latency with identical planner/controller across methods.
5. **Uncertainty:** produce an explicit probabilistic output and validate calibration on held-out data; if coverage assumptions fail on shifted terrain, report empirical coverage without guarantee.
6. **Novelty:** only then test a selective update/retain/rollback mechanism against tuned no-adapt, always-adapt, simple threshold and risk-aware calibration. Prove that update decisions improve **planner outcomes**, not only predictor MAE.

**Decision for next prompt:** continue with **real-data plumbing + controlled dynamics and safety validation**, not an immediate JEPA/MoE architecture commitment. Retain FOMAML and amortized context as *candidate prediction baselines*, not yet an accepted SCI(E) Q1 contribution.

## 11. Reproducibility and audit notes

- Inputs and task split are generated **locally**; no network data were used in the measured experiment. See `configs/protocol.json` and `results/pilot_v02/manifest.json` for SHA256 deterministic task IDs, source/test family segregation, machine/PyTorch/Numpy versions, training steps and model inventory.
- `pytest -q tests`: **12 passed**. Two independent training seeds 41, 42. Timing is wall-clock current CPU and is not deterministic across machines; accuracy/proxy outputs are seeded.
- The stochastic synthetic task design does **not** recreate real spatial/temporal correlation between road weather, site, tire load and terrain classes. Hard-coded 20-step support windows mimic benchmark accounting but cannot substitute real resampled sensor sequences.
- This document's observed results apply solely to `results/pilot_v02`; no external paper's experimental numbers are copied as new results. Do not use the **100% safe-path proxy** in abstracts or submission claims.
