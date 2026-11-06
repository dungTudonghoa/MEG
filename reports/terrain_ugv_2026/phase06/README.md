# Phase 06 — Few-shot meta-learning for terrain adaptation (research pilot)

**Important:** runnable PyTorch **synthetic interaction experiment**; not TartanDrive training, METAVerse/MAGIC-VFM/Levy reproduction, verified wheel-soil dynamics, ROS2, or real-world UGV safety benchmark. The planning bridge reuses the fixed Phase-05 A* and produces **offline oracle-audited route-risk proxies**, not closed-loop navigation success.

## Quick start (Fedora/Linux; Python 3.10+)

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
# The required Phase-05 A* subset is bundled in vendor_phase05:
export PYTHONPATH="$PWD/src:$PWD/vendor_phase05"
pytest -q tests
python -m phase06.experiment --out results --extended-k50
```

For a minimal smoke run (no Phase-05 bridge):

```bash
PYTHONPATH=src python -m phase06.experiment --out results_smoke --seeds 41 --source-steps 12 --meta-steps 4 --context-steps 15 --n-train 2 --n-val 1 --n-test 1 --no-planning-bridge
```

Measured CSVs: `episode_metrics.csv`, `adaptation_curves.csv`, `negative_transfer_paired.csv`, `training_curves.csv`, `run_metadata.csv`, and `manifest.json`. Plots: `learning_curves.png`, `adaptation_curves.png`, `route_proxy_curves.png`. See [`06_few_shot_meta_learning.md`](06_few_shot_meta_learning.md) for derivations, fairness qualifications, evidence and go/no-go decision.

## Safety & governance

- K=1 is **one 2 s window with 20 transitions**. K=50 is an **extended-budget** track, not in the phase-04 primary K set. Not 50 images.
- Synthetic source families = hardpack, dry grass, gravel, dirt; validation = dry sand; sealed test families = wet mud/wet clay. **This split is only for synthetic unit/protocol testing.** No actual site/route clusters exist.
- All methods see the same support windows/queries and same **source-only** training task families. Source, FOMAML and context training compute is **not matched**; source-pretrained vs meta must not be interpreted as a controlled data/compute ablation.
- `fine_tune` uses all-weight SGD on support transitions; `tta_head` uses head-only SGD on **identical** self-supervised measured transition targets, a useful parameter-efficiency test (not vision-only TTA). `context` uses a learned support-conditioned vector trained across source tasks, no target gradient updates.
- The synthetic slips are observed-response proxies, not wheel-slip ground truth or a probabilistically calibrated slip model.
- Route oracle is evaluation-only. Safety threshold is a synthetic policy constant and confers **no guarantees**.

## Bundled Phase-05 adapter

`vendor_phase05/terrain_baseline/{__init__,model,planner}.py` are copied unchanged from the **previous user's Phase-05 baseline ZIP**, and are included here to avoid a missing optional dependency. No ROS/Nav2 or OMPL APIs are used. Do not mistake the `safe_path_oracle_proxy` column for a simulator rollout.

## Run exactly the included pilot config

```bash
PYTHONPATH=src:vendor_phase05 python -m phase06.experiment \
  --out results/pilot_v02 --seeds 41,42 --source-steps 300 \
  --meta-steps 100 --context-steps 450 --n-train 10 \
  --n-val 4 --n-test 5 --extended-k50
python analyze_results.py
```

The CSV and PNG files shipped in `results/pilot_v02` come from this command on CPU. `validation_calibration_transfer.csv` is **only a task-level absolute-error threshold transfer diagnostic**: it is NOT probability calibration, ECE, or conformal coverage for individual slip predictions.
