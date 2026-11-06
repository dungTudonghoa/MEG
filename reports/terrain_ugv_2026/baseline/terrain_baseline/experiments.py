from __future__ import annotations
import csv
import hashlib
import json
import platform
import sys
from pathlib import Path
import math
import numpy as np
from .model import Vehicle, TerrainGrid
from .sensors import GridObservation, ElevationSensorLoader
from .scenarios import START,GOAL,SCENARIOS,make_scene
from .terrain import assess
from .planner import astar
from .safety import SafetyChecker
from .local import follow_path


def execute(scenario: str, seed: int, mode: str, robot: Vehicle) -> tuple[dict,dict]:
    true_grid=make_scene(scenario, seed)
    estimated_grid=GridObservation.from_truth(true_grid)
    # NOTE true grid cannot enter the planner except for public obstacle channel
    # via observation. All risk maps for planning are computed from the observed grid.
    est_risk=assess(estimated_grid,robot,unknown_forbidden=True)
    # The simulator truth contains all elevation/material properties, including
    # cells withheld from the planning observation; only evaluation uses them.
    full_truth=TerrainGrid(true_grid.elevation,true_grid.traction,true_grid.obstacle,
                           np.ones_like(true_grid.observed,dtype=bool),true_grid.resolution)
    true_risk=assess(full_truth,robot,unknown_forbidden=False)
    p,info=astar(estimated_grid,est_risk,START,GOAL,mode=mode)
    rollout=follow_path(true_grid,SafetyChecker(true_grid,true_risk,robot),p,robot)
    pxy=[true_grid.xy(q) for q in p]
    plan_len=sum(math.dist(a,b) for a,b in zip(pxy,pxy[1:]))
    # Check planned path in simulator truth. This is evaluation-only.
    simulator_checker=SafetyChecker(true_grid,true_risk,robot)
    plan_safe=all(simulator_checker.check_segment(a,b)[0] for a,b in zip(pxy,pxy[1:]))
    metric={'scenario':scenario,'seed':seed,'planner':mode,'success':int(rollout['success']),
            'stop_reason':rollout['stop_reason'],'unsafe_plan':int(bool(p) and not plan_safe),
            'safety_stops':rollout['safety_stops'],'executed_constraint_violations':0,
            'planned_length_m':round(plan_len,4),'executed_length_m':round(rollout['path_length_actual_m'],4),
            'travel_time_s':round(rollout['travel_time_s'],4),
            'tracking_rmse_m':round(rollout['tracking_rmse_m'],5) if np.isfinite(rollout['tracking_rmse_m']) else None,
            'expanded':info['expanded'],'plan_latency_ms':round(info['plan_latency_ms'],4),
            'path_found':int(bool(p))}
    return metric, {'truth':true_grid,'estimated':estimated_grid,'risk_est':est_risk,'risk_true':true_risk,
                    'path':p,'trajectory':rollout['trajectory'],'estimated_trajectory':rollout.get('estimated_trajectory',[]),'controls':rollout.get('control_commands',[])}


def write_csv(path: Path, rows:list[dict]):
    with path.open('w',newline='',encoding='utf-8') as f:
        out=csv.DictWriter(f,fieldnames=list(rows[0]));out.writeheader();out.writerows(rows)


def aggregate(rows):
    out=[]
    for scene in SCENARIOS:
        for planner in ('geometry','terrain'):
            r=[q for q in rows if q['scenario']==scene and q['planner']==planner]
            if not r: continue
            out.append(dict(scenario=scene,planner=planner,n=len(r),success_rate=float(np.mean([q['success'] for q in r])),
                            unsafe_plan_rate=float(np.mean([q['unsafe_plan'] for q in r])),
                            safety_stop_rate=float(np.mean([q['safety_stops'] for q in r])),
                            median_planning_ms=float(np.median([q['plan_latency_ms'] for q in r])),
                            mean_planned_length_m=float(np.mean([q['planned_length_m'] for q in r]))))
    return out


def run(config:dict,output:Path,make_plots=True):
    from .visual import plot_case
    output.mkdir(exist_ok=True,parents=True)
    robot=Vehicle(**config.get('vehicle',{}))
    scenes=config.get('scenarios',list(SCENARIOS))
    seeds=config.get('seeds',[42,43,44])
    plans=config.get('planners',['geometry','terrain'])
    rows=[]
    for scene in scenes:
        if scene not in SCENARIOS: raise ValueError(f'Invalid scenario {scene}')
        first_cases={}
        for seed in seeds:
            for plan in plans:
                row,art=execute(scene,seed,plan,robot)
                rows.append(row)
                if seed==seeds[0]:first_cases[plan]=art
                if seed==seeds[0]:
                    tpath=output/f'{scene}_{plan}_trajectory.csv'
                    with tpath.open('w',newline='') as f:
                        w=csv.writer(f);w.writerow(['x_m','y_m','heading_rad','t_s']);w.writerows(art['trajectory'])
                    with (output/f'{scene}_{plan}_estimated_trajectory.csv').open('w',newline='') as f:
                        w=csv.writer(f);w.writerow(['x_m','y_m','heading_rad','t_s']);w.writerows(art['estimated_trajectory'])
                    with (output/f'{scene}_{plan}_controls.csv').open('w',newline='') as f:
                        w=csv.writer(f);w.writerow(['v_cmd_mps','w_cmd_rps']);w.writerows(art['controls'])
        if make_plots and first_cases:
            plot_case(scene,first_cases,output/f'{scene}.png')
    if not rows:raise ValueError('No experiments')
    write_csv(output/'metrics_raw.csv',rows)
    summary=aggregate(rows)
    if summary:write_csv(output/'metrics_summary.csv',summary)
    md=[]
    md.append('# Measured synthetic 2.5D benchmark (NOT original-paper replication)\n')
    md.append('`success` means reaching goal with the shared oracle supervisor enabled. `unsafe_plan` means at least one segment of the proposed global path violates simulator ground-truth limits; it does **not** mean the robot executed an unsafe action. Timing measured on the current container, not embedded UGV hardware.\n')
    md.append('| Terrain | Planner | n | Success | Unsafe proposal | Safety-stop | Median global planning (ms) | Mean planned length (m) |')
    md.append('|---|---|---:|---:|---:|---:|---:|---:|')
    for r in summary:
        md.append(f"|{r['scenario']}|{r['planner']}|{r['n']}|{r['success_rate']:.2f}|{r['unsafe_plan_rate']:.2f}|{r['safety_stop_rate']:.2f}|{r['median_planning_ms']:.2f}|{r['mean_planned_length_m']:.2f}|")
    (output/'MEASURED_RESULTS.md').write_text('\n'.join(md)+'\n',encoding='utf-8')
    meta={'python':sys.version,'platform':platform.platform(),'numpy':np.__version__,
          'config':config,'parameters':vars(robot)}
    (output/'run_manifest.json').write_text(json.dumps(meta,indent=2,ensure_ascii=False),encoding='utf-8')
    return rows,summary
