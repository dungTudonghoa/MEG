"""Executable few-shot pilot, writes long-form metrics, learning/adaptation curves.

Evaluation uses only held-out test task query for *scoring*. Query is never
passed to inner updates or support summary and no test results are used to
choose training hyperparameters. This benchmark is synthetic, not real UGV.
"""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
import platform
import sys
from pathlib import Path
import numpy as np
import torch
from .core import (
    SPLITS, K_PRIMARY, K_EXTENDED, SHOT_TRANSITIONS, SHOT_DURATION_S,
    make_tasks, source_train, fomaml_train, context_train, predict, stable_seed,
    true_response_np
)

METHODS=('never','fine_tune','tta_head','fomaml','context')


def dump_csv(filename:Path,rows:list[dict])->None:
    if not rows:return
    with filename.open('w',newline='',encoding='utf-8') as fh:
        w=csv.DictWriter(fh,fieldnames=list(dict.fromkeys(k for row in rows for k in row)))
        w.writeheader();w.writerows(rows)


def route_eval(method:str,base,meta,ctx,task,k,seed):
    """Evaluate the SAME Phase-05 8-connected A* planner under learned risk.

    NO physics sim, NO ROS2 / real robot, NO true params in model/planner.
    'Path safe' checks withheld synthetic world truth and is a route-risk proxy.
    """
    from terrain_baseline.model import TerrainGrid, RiskMap
    from terrain_baseline.planner import astar
    from phase06.core import inner_update
    H,W=19,29
    rows,cols=np.mgrid[0:H,0:W]
    rng=np.random.default_rng(stable_seed('map',seed,task.task_id,k))
    zone=(rows>=5)&(rows<=13)&(cols>=9)&(cols<=19)
    # Full grid observed; terrain-specific *appearance* signal is not true friction.
    cue=np.where(zone, .2, .92).astype(float)
    cue=np.clip(cue+rng.normal(0,.02,(H,W)),0,1)
    slope=np.where(zone,.55,.07)
    rough=np.where(zone,.48,.11)
    x=np.column_stack((cue.ravel(),slope.ravel(),rough.ravel(),np.full(H*W,.67),np.full(H*W,.18))).astype('float32')
    sx,sy=task.support(k)
    if method=='never':
        model=base
    elif method=='fine_tune':
        model=inner_update(base,sx,sy,5,.35)
    elif method=='tta_head':
        model=inner_update(base,sx,sy,5,.35,True)
    elif method=='fomaml':
        model=inner_update(meta,sx,sy,5,.35)
    elif method=='context':
        model=None
    else:raise ValueError(method)
    with torch.no_grad():
        out=(ctx(torch.from_numpy(x),sx,sy) if method=='context'
             else model(torch.from_numpy(x))).numpy()
    # Allowed model-derived estimate, without truth or hidden soil parameters.
    # Fixed hand-off rule: only visually suspected soft patch uses target-adapted
    # estimate; observed hard-ground cue uses the same source model for everyone.
    # Threshold .6 is applied to the *observed cue*, not hidden terrain identity.
    with torch.no_grad(): source_slip=base(torch.from_numpy(x))[:,0].numpy().reshape(H,W)
    pred_slip=np.where(cue < .6, out[:,0].reshape(H,W),source_slip)
    # Synthetic fixed safety threshold is a policy design constant, not a guarantee.
    forbid=pred_slip>.60
    obs=np.ones((H,W),bool)
    grid=TerrainGrid(np.zeros((H,W)),np.ones((H,W)),np.zeros((H,W),bool),obs,.4)
    risk=RiskMap(slope*.22,rough*.045,pred_slip,forbid,obs)
    path,info=astar(grid,risk,(9,2),(9,26),mode='terrain')
    # Truth used AFTER selecting route only. Noise-free synthetic simulator risk.
    from phase06.core import make_params
    hardpack_truth=make_params('hardpack',73,0)
    true_soft=true_response_np(x.astype(float),task.params)[:,0].reshape(H,W)
    true_hard=true_response_np(x.astype(float),hardpack_truth)[:,0].reshape(H,W)
    actual=np.where(zone,true_soft,true_hard)
    if path:
        pos=np.array(path)
        route_slips=actual[pos[:,0],pos[:,1]]
        unsafe=int(bool(np.any(route_slips>.60)))
        length=sum(float(np.hypot(i[0]-j[0],i[1]-j[1])*.4) for i,j in zip(path,path[1:]))
        mean_risk=float(np.mean(route_slips))
    else:unsafe=0;length=float('nan');mean_risk=float('nan')
    return {'path_found':int(bool(path)),'unsafe_path_oracle_proxy':unsafe,
            'safe_path_oracle_proxy':int(bool(path) and not unsafe),
            'path_length_m_proxy':length,'mean_path_slip_truth_proxy':mean_risk,
            'expanded_nodes':info['expanded'],'planning_ms':info['plan_latency_ms']}


def experiment(output:Path,seeds:list[int],steps_source:int,steps_meta:int,steps_context:int,
               n_train:int,n_val:int,n_test:int,k_levels:list[int],bridge:bool=True):
    output.mkdir(parents=True,exist_ok=True)
    rows=[];learning=[];runinfo=[];test_task_ids=[]
    for seed in seeds:
        source=make_tasks('train',seed,n_train)
        validation=make_tasks('val',seed,n_val)
        target=make_tasks('test',seed,n_test)
        assert not ({t.family for t in source}&{t.family for t in target})
        test_task_ids += [t.task_id+f'/seed{seed}' for t in target]
        base,base_curve,base_time=source_train(source,steps_source,seed)
        meta,meta_curve,meta_time=fomaml_train(source,steps_meta,seed+10)
        ctx,ctx_curve,ctx_time=context_train(source,steps_context,seed+20)
        runinfo.append({'seed':seed,'source_training_seconds':base_time,
                        'fomaml_training_seconds':meta_time,'context_training_seconds':ctx_time,
                        'source_tasks':len(source),'validation_tasks':len(validation),
                        'test_tasks':len(target)})
        for method,curve in [('source_training',base_curve),('fomaml_meta_training',meta_curve),('context_meta_training',ctx_curve)]:
            for row in curve:learning.append({'seed':seed,'method':method,**row})
        # validation computed but not used to tune parameters in this frozen run
        for split,tasks in [('val',validation),('test',target)]:
            for task in tasks:
                qx,qy=task.query()
                for k in k_levels:
                    for method in METHODS:
                        p,latency,updates=predict(method,base,meta,ctx,task,k)
                        slip_mae=float(np.mean(np.abs(p[:,0]-qy[:,0].numpy())))
                        slip_rmse=float(np.sqrt(np.mean((p[:,0]-qy[:,0].numpy())**2)))
                        yaw_mae=float(np.mean(np.abs(p[:,1]-qy[:,1].numpy())))
                        # We don't output event probabilities -> no ECE / calibration claim
                        row={'split':split,'seed':seed,'task':task.task_id,'family':task.family,
                             'method':method,'k_windows':k,'support_transitions':k*SHOT_TRANSITIONS,
                             'support_seconds':k*SHOT_DURATION_S,'labels':'self_supervised_past_state_actions',
                             'query_transitions':len(qy),'adapt_steps':updates,
                             'slip_mae':slip_mae,'slip_rmse':slip_rmse,'yaw_shortfall_mae':yaw_mae,
                             'adapt_and_infer_ms_cpu':latency}
                        if split=='test' and bridge and int(task.task_id.split('/')[-1]) < 2:
                            row.update(route_eval(method,base,meta,ctx,task,k,seed))
                        rows.append(row)
    dump_csv(output/'episode_metrics.csv',rows)
    dump_csv(output/'training_curves.csv',learning)
    dump_csv(output/'run_metadata.csv',runinfo)
    test=[r for r in rows if r['split']=='test']
    meanrows=[]
    for method in METHODS:
        for k in k_levels:
            sub=[r for r in test if r['method']==method and r['k_windows']==k]
            x={'method':method,'k_windows':k,'n_tasks':len(sub),
               'slip_mae_mean':float(np.mean([r['slip_mae'] for r in sub])),
               'slip_rmse_mean':float(np.mean([r['slip_rmse'] for r in sub])),
               'yaw_mae_mean':float(np.mean([r['yaw_shortfall_mae'] for r in sub])),
               'adapt_infer_ms_median':float(np.median([r['adapt_and_infer_ms_cpu'] for r in sub]))}
            if bridge:
                plan=[r for r in sub if 'path_found' in r]
                x.update({'planning_audit_n':len(plan),
                          'path_found_rate_proxy':float(np.mean([r['path_found'] for r in plan])) if plan else float('nan'),
                          'safe_path_rate_oracle_proxy':float(np.mean([r['safe_path_oracle_proxy'] for r in plan])) if plan else float('nan'),
                          'unsafe_path_rate_oracle_proxy':float(np.mean([r['unsafe_path_oracle_proxy'] for r in plan])) if plan else float('nan')})
            meanrows.append(x)
    dump_csv(output/'adaptation_curves.csv',meanrows)
    # paired harm: did adaptation raise target query error above never-adapt, same task/K?
    harms=[]
    for k in k_levels:
        paired={(r['seed'],r['task'],r['method']):r for r in test if r['k_windows']==k}
        for method in METHODS:
            if method=='never':continue
            for task in [r for r in test if r['k_windows']==k and r['method']==method]:
                base_row=paired[(task['seed'],task['task'],'never')]
                harms.append({'k_windows':k,'method':method,'seed':task['seed'],'task':task['task'],
                              'delta_slip_mae_vs_never':task['slip_mae']-base_row['slip_mae'],
                              'harmful_prediction_update':int(task['slip_mae']>base_row['slip_mae']+1e-8),
                              'note':'Prediction harm only; not physical safety harm'})
    dump_csv(output/'negative_transfer_paired.csv',harms)
    # Validation-only calibration: per-method/K/seed empirical |slip residual|
    # quantile, then report target coverage under family shift; never recalibrate
    # using target query. These are descriptive intervals, NOT shift guarantees.
    coverage_rows=[]
    for seed in seeds:
        for k in k_levels:
            for method in METHODS:
                vr=[r for r in rows if r['split']=='val' and r['seed']==seed
                    and r['k_windows']==k and r['method']==method]
                tr=[r for r in test if r['seed']==seed and r['k_windows']==k and r['method']==method]
                # Conservative proxy calibration: task-level mean absolute residual
                # is an observable scalar. This is NOT per-transition coverage.
                scores=np.sort([r['slip_mae'] for r in vr])
                if not len(scores):continue
                idx=min(len(scores)-1,max(0,int(np.ceil((len(scores)+1)*.9))-1))
                qhat=float(scores[idx])
                for r in tr:
                    coverage_rows.append({'seed':seed,'k_windows':k,'method':method,'task':r['task'],
                                          'calibration_set':'validation_dry_sand_only',
                                          'calibration_n_tasks':len(scores),
                                          'qhat_val_abs_task_mae':qhat,
                                          'test_task_mean_abs_error':r['slip_mae'],
                                          'task_mean_error_within_val_quantile':int(r['slip_mae']<=qhat),
                                          'interpretation':'task-level error threshold, not pointwise calibrated predictive interval'})
    dump_csv(output/'validation_calibration_transfer.csv',coverage_rows)
    manifest={'protocol':'phase04 v0.1 synthetic pilot (not frozen real-data benchmark)',
              'methods':METHODS,'seeds':seeds,'k_levels':k_levels,
              'shot_definition':f'{SHOT_TRANSITIONS} 10-Hz transitions = {SHOT_DURATION_S}s of one support window',
              'source_families':SPLITS['train'],'validation_families':SPLITS['val'],'test_families':SPLITS['test'],
              'planning_audit_tasks_per_family_per_seed':2 if bridge else 0,
              'n_source_tasks_per_family':n_train,'n_validation_tasks_per_family':n_val,'n_test_tasks_per_family':n_test,
              'source_train_steps':steps_source,'fomaml_meta_steps':steps_meta,
              'context_meta_steps':steps_context,'fomaml_tasks_per_outer_step':3,
              'source_data_access':'all source tasks only','meta_test_query_use':'evaluation only',
              'test_task_ids_sha256':hashlib.sha256('\n'.join(test_task_ids).encode()).hexdigest(),
              'torch':torch.__version__,'numpy':np.__version__,'python':sys.version,'platform':platform.platform(),
              'actual_dataset':'none - synthetic task generator',
              'closed_loop_physics':'none - fixed Phase05 A* with post-plan truth audit only',
              'random_seed_policy':'SHA256 deterministic per split/family/task/run'}
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False,default=str),encoding='utf-8')
    return rows,meanrows,runinfo,harms


def plot_results(output:Path)->None:
    import matplotlib.pyplot as plt
    import pandas as pd
    c=pd.read_csv(output/'adaptation_curves.csv')
    fig,ax=plt.subplots(figsize=(8,4.5))
    for name,part in c.groupby('method',sort=False):
        part=part.sort_values('k_windows')
        ax.plot(part.k_windows,part.slip_mae_mean,marker='o',label=name)
    ax.set(xlabel='Support windows K (1 window = 20 transitions)',ylabel='Held-out synthetic slip MAE',
           title='Test adaptation curves (unseen synthetic terrain families)')
    ax.legend(ncol=2,fontsize=8);fig.tight_layout();fig.savefig(output/'adaptation_curves.png',dpi=150);plt.close(fig)
    curves=pd.read_csv(output/'training_curves.csv')
    fig,ax=plt.subplots(figsize=(8,4.5))
    for method,part in curves.groupby('method'):
        m=part.groupby('step').loss.mean().reset_index()
        ax.plot(m.step,m.loss,label=method)
    ax.set(xlabel='Optimizer outer/source steps (not compute matched)',ylabel='Source-domain MSE loss',
           title='Pilot source/meta-training curves')
    ax.legend(fontsize=8);fig.tight_layout();fig.savefig(output/'learning_curves.png',dpi=150);plt.close(fig)
    if 'safe_path_rate_oracle_proxy' in c.columns:
        fig,ax=plt.subplots(figsize=(8,4.5))
        for name,part in c.groupby('method',sort=False):
            ax.plot(part.k_windows,part.safe_path_rate_oracle_proxy,marker='o',label=name)
        ax.set(xlabel='Support windows K',ylabel='Synthetic oracle-audited safe planned path rate',
               title='Offline A* route audit (NOT closed-loop robot success)',ylim=(0,1.03))
        ax.legend(ncol=2,fontsize=8);fig.tight_layout();fig.savefig(output/'route_proxy_curves.png',dpi=150);plt.close(fig)


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--out',default='results')
    p.add_argument('--seeds',default='41,42')
    p.add_argument('--source-steps',type=int,default=450)
    p.add_argument('--meta-steps',type=int,default=145)
    p.add_argument('--context-steps',type=int,default=650)
    p.add_argument('--n-train',type=int,default=16)
    p.add_argument('--n-val',type=int,default=4)
    p.add_argument('--n-test',type=int,default=6)
    p.add_argument('--extended-k50',action='store_true')
    p.add_argument('--no-planning-bridge',action='store_true')
    args=p.parse_args()
    ks=list(K_PRIMARY)+(list(K_EXTENDED) if args.extended_k50 else [])
    out=Path(args.out)
    rows,means,meta,harms=experiment(out,[int(v) for v in args.seeds.split(',')],args.source_steps,
                                     args.meta_steps,args.context_steps,args.n_train,args.n_val,args.n_test,ks,
                                     bridge=not args.no_planning_bridge)
    plot_results(out)
    for k in ks:
        print(f'K={k}',{m:round(next(r['slip_mae_mean'] for r in means if r['method']==m and r['k_windows']==k),4) for m in METHODS})
    print(f'Wrote results in {out} | episodes={len(rows)} | predictor_harm_rows={len(harms)}')

if __name__=='__main__':main()
