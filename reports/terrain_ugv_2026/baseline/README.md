# Terrain-Aware Planning Baseline — Phase 05

**Status:** executable, deterministic synthetic 2.5D Python baseline. **Not:** Feng et al. (2025) reproduction, ROS 2 / Nav2 plugin, OMPL implementation, wheel-soil physical simulator, ML model or certified safety system.

## Run (Fedora/Linux, Python 3.10+)

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -e . pytest
pytest -q
python -m terrain_baseline.cli --config configs/benchmark.yaml --output results
```

Outputs: `metrics_raw.csv`, `metrics_summary.csv`, `MEASURED_RESULTS.md`, `run_manifest.json`, each scenario PNG, and initial-seed actual/estimated trajectories and twist-command CSV. Run from the project root. `--no-plots` disables visualization.

## What it actually does

- Provides typed `TerrainGrid` for 2.5D synthetic elevation, traction proxy, obstacle mask and observed mask.
- `ElevationSensorLoader` reads/writes `.npz` grid fields, not raw RGB images or ROS bags.
- `assess()` derives slope, roughness proxy, and **synthetic slip-risk proxy** from *observed* terrain only; no learned traversability.
- Compares geometry-only 8-connected A* with terrain-weighted 8-connected A* using identical start/goal and synthetic worlds.
- Uses deterministic differential-drive rotate-to-heading/drive waypoint controller with twist/acceleration limits.
- Uses **simulator-only oracle safety checker** before each action; both planners share the checker. `unsafe_plan` is an evaluation of a proposed path, not proof the robot drove it.
- Synthetic terrain traction changes speed; no sinkage, shear strength, soil pressure, load transfer, side slip, suspension, realistic contact friction, or actual vehicle odometry drift.
- Does not train or deploy meta-learning, JEPA, MoE.

## Key caveat

**A* global paths are not Ackermann-feasible and not inherently vehicle-dynamics feasible.** Differential drive is selected because it permits in-place rotation in this simplified setting. Do not transfer these code paths to an Ackermann or skid-steer 4WD robot without a new nonholonomic model and validation. For a real skid-steer platform, the interaction-dependent slip model must be identified/validated.

## Benchmark policy

`configs/benchmark.yaml` is the baseline frozen config (v0.1). The five synthetic scenes and fixed seeds form **software regression tests only**. These are not independent real terrain domains, not few-shot episodes, and cannot support claims of off-road generalization. Follow `04_dataset_and_benchmark.md` to implement real dataset manifest and benchmark before training any AI model.

See `05_baseline_reproduction.md` for algorithm audit, literature comparisons and reproduction ledger.
