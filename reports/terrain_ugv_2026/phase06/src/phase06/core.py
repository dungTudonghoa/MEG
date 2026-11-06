"""Phase 06 research pilot: *synthetic* wheeled-UGV interaction prediction.

No ROS2 / terrain sensor dataset / real slip measurements. The task model is synthetic
and this module must not be used to claim field-robot safety or original paper reproduction.
"""
from __future__ import annotations
import copy
import hashlib
import json
import math
import time
from dataclasses import dataclass
from typing import Any
import numpy as np
import torch
from torch import nn

# Frozen before any model fit: all domains and their parameter ranges are source code.
FAMILIES = {
    "hardpack": ( -2.7, 1.0, 1.1, 0.9, 0.6, 1.2),
    "dry_grass": (-2.0, 1.4, 1.3, 0.8, 0.9, 1.4),
    "gravel":    (-1.65,1.2, 1.6, 1.3, 0.8, 1.3),
    "dirt":      (-1.85,1.5, 1.1, 1.1, 0.7, 1.2),
    "dry_sand":  (-0.85,1.9, 1.7, 1.5, 1.0, 1.7),
    "wet_mud":   (-0.25,2.0, 2.0, 1.8, 1.5, 2.1),
    "wet_clay":  (-0.5, 2.2, 1.8, 1.7, 1.6, 2.0),
}
SPLITS = {"train": ("hardpack", "dry_grass", "gravel", "dirt"),
          "val": ("dry_sand",), "test": ("wet_mud", "wet_clay")}
SHOT_TRANSITIONS = 20
SHOT_DURATION_S = 2.0
SAMPLE_HZ = 10
K_PRIMARY = (0, 1, 2, 5, 10, 20)
K_EXTENDED = (50,)
INPUT_DIM = 5
OUTPUT_DIM = 2 # actual slip ratio proxy, normalized non-negative yaw tracking shortfall


def stable_seed(*args: object) -> int:
    return int.from_bytes(hashlib.sha256("|".join(map(str,args)).encode()).digest()[:4], 'big')


def set_seed(seed: int) -> None:
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.set_num_threads(1)  # cpu reproducibility of small experiment
    torch.use_deterministic_algorithms(True)


@dataclass(frozen=True)
class Task:
    task_id: str
    family: str
    split: str
    params: np.ndarray
    support_x: np.ndarray  # (50, 20, 5), chronologically ordered windows
    support_y: np.ndarray  # (50, 20, 2), generated from past action->measured next state
    query_x: np.ndarray    # independent run; never use for adaptation
    query_y: np.ndarray
    support_run_id: str
    query_run_id: str

    def support(self, k: int) -> tuple[torch.Tensor,torch.Tensor]:
        assert 0 <= k <= len(self.support_x)
        return (torch.from_numpy(self.support_x[:k].reshape(-1, INPUT_DIM).copy()),
                torch.from_numpy(self.support_y[:k].reshape(-1, OUTPUT_DIM).copy()))

    def query(self) -> tuple[torch.Tensor,torch.Tensor]:
        return torch.from_numpy(self.query_x.copy()), torch.from_numpy(self.query_y.copy())


def make_params(family: str, seed: int, task_index: int) -> np.ndarray:
    rng = np.random.default_rng(stable_seed('params',seed,family,task_index))
    p=np.array(FAMILIES[family],dtype=np.float64)
    p += rng.normal(0,[.18,.12,.12,.11,.1,.14])
    return p


def true_response_np(x: np.ndarray, params: np.ndarray) -> np.ndarray:
    """Synthetic process, used to generate observed next-state labels and score held-out routes.

    x = [noisy appearance cue, slope/0.4, roughness/0.2, v/1mps, |yawcmd|/1rps].
    Parameters are NOT visible to learner or its planner. This is NOT terramechanics.
    """
    cue,slope,rough,v,yaw=x.T
    b,a,b_s,c,d,e=params
    z = b + a*slope + b_s*rough + c*v + d*yaw + e*(1-cue) + 0.9*v*(1-cue)
    slip=1/(1+np.exp(-np.clip(z,-15,15)))
    yawshort= np.clip(slip*(0.15+0.43*yaw),0,0.99)
    return np.stack([slip,yawshort],axis=-1)


def make_windows(family: str, params: np.ndarray, seed: int, task_index: int,
                 run_tag: str, num_windows: int) -> tuple[np.ndarray,np.ndarray]:
    """Window-level correlated synthetic signals, different RNG per independent run.

    y simulates observed response after x_t,u_t; no manual slip labels. Sampling
    and noisy cues are exogenous, not derived from held-out query for normalization.
    """
    rng = np.random.default_rng(stable_seed('windows',seed,family,task_index,run_tag))
    xs=[];ys=[]
    for j in range(num_windows):
        window_base=rng.uniform(.15,.85,size=3)
        cue0,slope0,rough0=window_base
        x=np.column_stack([
            np.clip(cue0+rng.normal(0,.075,size=SHOT_TRANSITIONS),0,1),
            np.clip(slope0+rng.normal(0,.075,size=SHOT_TRANSITIONS),0,1),
            np.clip(rough0+rng.normal(0,.09,size=SHOT_TRANSITIONS),0,1),
            rng.uniform(.25,.9,size=SHOT_TRANSITIONS),
            rng.uniform(.02,.95,size=SHOT_TRANSITIONS),
        ]).astype(np.float32)
        slip_yaw=true_response_np(x.astype(float),params)
        # Next-state observation noise, not oracle labels at inference.
        noisy=np.clip(slip_yaw+rng.normal(0,[.025,.022],size=(SHOT_TRANSITIONS,2)),0,1)
        xs.append(x)
        ys.append(noisy.astype(np.float32))
    return np.stack(xs),np.stack(ys)


def make_tasks(split:str,seed:int,n_per_family:int) -> list[Task]:
    assert split in SPLITS
    tasks=[]
    for family in SPLITS[split]:
        for i in range(n_per_family):
            params=make_params(family,seed,i)
            sx,sy=make_windows(family,params,seed,i,'support_run',50)
            qx,qy=make_windows(family,params,seed,i,'query_different_run',12)
            tasks.append(Task(f'{split}/{family}/{i:03d}',family,split,params,sx,sy,qx.reshape(-1,5),qy.reshape(-1,2),
                         f'{split}/{family}/{i:03d}/S',f'{split}/{family}/{i:03d}/Q'))
    return tasks


class Predictor(nn.Module):
    """Dense network. Same base architecture for source, FOMAML, FT, TTA-head."""
    def __init__(self,hidden:int=24):
        super().__init__()
        self.encoder=nn.Sequential(nn.Linear(INPUT_DIM,hidden),nn.Tanh(),nn.Linear(hidden,hidden),nn.Tanh())
        self.head=nn.Linear(hidden,OUTPUT_DIM)
    def forward(self,x:torch.Tensor)->torch.Tensor:
        return torch.sigmoid(self.head(self.encoder(x)))


class ContextPredictor(nn.Module):
    """Amortized source-trained context meta-learner; only support signals form c(S).

    No gradient/weight update at test time; meta-TRAINED across tasks, so adaptation
    uses learned inference rather than a hand-adjusted threshold.
    """
    def __init__(self,hidden:int=24):
        super().__init__()
        # context stats: mean_x 5, mean_y 2, mean_xySlip 5, presence 1
        self.ctx=nn.Sequential(nn.Linear(13,hidden),nn.Tanh(),nn.Linear(hidden,hidden),nn.Tanh())
        self.pred=nn.Sequential(nn.Linear(INPUT_DIM+hidden,hidden),nn.Tanh(),nn.Linear(hidden,OUTPUT_DIM))
    @staticmethod
    def stats(sx:torch.Tensor,sy:torch.Tensor)->torch.Tensor:
        if sx.shape[0]==0:
            return sx.new_zeros(13)
        return torch.cat([sx.mean(0),sy.mean(0), (sx*sy[:,0:1]).mean(0), sx.new_ones(1)])
    def forward(self,x:torch.Tensor,sx:torch.Tensor,sy:torch.Tensor)->torch.Tensor:
        c=self.ctx(self.stats(sx,sy)).unsqueeze(0).expand(x.shape[0],-1)
        return torch.sigmoid(self.pred(torch.cat([x,c],dim=-1)))


def source_train(tasks:list[Task],steps:int,seed:int,lr:float=0.005)->tuple[Predictor,list[dict],float]:
    set_seed(seed)
    model=Predictor(); opt=torch.optim.Adam(model.parameters(),lr=lr)
    rng=np.random.default_rng(seed)
    logs=[]; t0=time.perf_counter()
    for i in range(steps):
        task=tasks[int(rng.integers(0,len(tasks)))]
        x,y=task.query()   # training SOURCE task query permissible; no test task data
        ix=rng.choice(len(x),size=80,replace=False)
        opt.zero_grad(); loss=((model(x[ix])-y[ix])**2).mean();loss.backward();opt.step()
        if i%max(1,steps//12)==0 or i==steps-1:logs.append({'step':i+1,'loss':loss.item()})
    return model,logs,time.perf_counter()-t0


def inner_update(model:Predictor,sx:torch.Tensor,sy:torch.Tensor,steps:int,lr:float,
                 head_only:bool=False)->Predictor:
    fast=copy.deepcopy(model)
    if len(sx)==0:return fast
    params=list(fast.head.parameters()) if head_only else list(fast.parameters())
    for _ in range(steps):
        loss=((fast(sx)-sy)**2).mean()
        grads=torch.autograd.grad(loss,params)
        with torch.no_grad():
            for p,g in zip(params,grads):p.add_(-lr*g)
    return fast


def fomaml_train(tasks:list[Task],steps:int,seed:int,lr_outer:float=0.005,
                  lr_inner:float=.35,inner_steps:int=3)->tuple[Predictor,list[dict],float]:
    """First-order MAML: no 2nd-order Jacobian. Source support->source query.
    Outer grad is query grad evaluated at adapted parameters (FO surrogate).
    """
    set_seed(seed)
    model=Predictor(); outer=torch.optim.Adam(model.parameters(),lr=lr_outer)
    rng=np.random.default_rng(seed+130)
    logs=[];t0=time.perf_counter()
    for i in range(steps):
        grads_sum=[torch.zeros_like(p) for p in model.parameters()]
        qerrs=[]
        for _ in range(3):
            task=tasks[int(rng.integers(0,len(tasks)))]
            sx,sy=task.support(5)
            fast=inner_update(model,sx,sy,inner_steps,lr_inner)
            qx,qy=task.query()
            idx=rng.choice(len(qx),size=50,replace=False)
            err=((fast(qx[idx])-qy[idx])**2).mean()
            qgrads=torch.autograd.grad(err,list(fast.parameters()))
            for acc,g in zip(grads_sum,qgrads):acc.add_(g.detach()/3)
            qerrs.append(err.item())
        outer.zero_grad()
        for p,g in zip(model.parameters(),grads_sum):p.grad=g
        torch.nn.utils.clip_grad_norm_(model.parameters(),max_norm=2.0)
        outer.step()
        if i%max(1,steps//12)==0 or i==steps-1: logs.append({'step':i+1,'loss':float(np.mean(qerrs))})
    return model,logs,time.perf_counter()-t0


def context_train(tasks:list[Task],steps:int,seed:int,lr:float=0.004)->tuple[ContextPredictor,list[dict],float]:
    set_seed(seed)
    model=ContextPredictor();opt=torch.optim.Adam(model.parameters(),lr=lr)
    rng=np.random.default_rng(seed+210)
    logs=[];t0=time.perf_counter()
    for i in range(steps):
        task=tasks[int(rng.integers(0,len(tasks)))]
        k=int(rng.choice([0,1,2,5,10,20]))
        sx,sy=task.support(k);qx,qy=task.query()
        idx=rng.choice(len(qx),size=80,replace=False)
        opt.zero_grad();loss=((model(qx[idx],sx,sy)-qy[idx])**2).mean();loss.backward()
        opt.step()
        if i%max(1,steps//12)==0 or i==steps-1:logs.append({'step':i+1,'loss':loss.item()})
    return model,logs,time.perf_counter()-t0


def predict(method:str,base:Predictor,meta:Predictor,ctx:ContextPredictor,task:Task,k:int,
            adapt_steps:int=5,adapt_lr:float=.35)->tuple[np.ndarray,float,int]:
    sx,sy=task.support(k);qx,_=task.query()
    t0=time.perf_counter()
    if method=='never':
        fn=lambda x:base(x);updates=0
    elif method=='fine_tune':
        model=inner_update(base,sx,sy,adapt_steps,adapt_lr);fn=lambda x:model(x); updates=adapt_steps if k else 0
    elif method=='tta_head':
        model=inner_update(base,sx,sy,adapt_steps,adapt_lr,head_only=True);fn=lambda x:model(x);updates=adapt_steps if k else 0
    elif method=='fomaml':
        model=inner_update(meta,sx,sy,adapt_steps,adapt_lr);fn=lambda x:model(x);updates=adapt_steps if k else 0
    elif method=='context':
        fn=lambda x:ctx(x,sx,sy);updates=0
    else: raise ValueError(method)
    with torch.no_grad():arr=fn(qx).numpy()
    return arr,1000*(time.perf_counter()-t0),updates


def ece_binary_soft_risk(y:np.ndarray,p:np.ndarray,n_bins:int=8,threshold:float=.55)->float:
    """Calibration of P(event) is NOT identifiable from slip-point predictions.
    Here only a thresholded *risk proxy* Brier score is applicable; ECE omitted.
    """
    raise NotImplementedError('Use probabilistic event prediction with held-out calibration to calculate ECE.')


def route_score(model_pred:np.ndarray, true_slip:np.ndarray,threshold:float=.55)->dict:
    """Offline risk-decision proxy; NOT closed-loop vehicle trajectory evidence."""
    # risky shortcut (10 tiles) or safe but longer bypass (14 tiles). Predictions
    # depend on selected model. Unknown true slippery tiles evaluated only after decision.
    route='shortcut' if float(np.mean(model_pred))<=.40 else 'bypass'
    actual_risk=float(np.mean(true_slip)) if route=='shortcut' else .045
    return {'route':route,'unsafe_route_proxy':int(actual_risk>threshold),
            'travel_length_proxy':10.0 if route=='shortcut' else 14.0,
            'realized_slip_risk_proxy':actual_risk}
