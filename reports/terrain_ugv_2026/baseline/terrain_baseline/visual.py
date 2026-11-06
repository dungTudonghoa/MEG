from __future__ import annotations
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def plot_case(scene:str, cases:dict, path:Path):
    # 2-panel comparable global path + executed trajectory
    fig,axes=plt.subplots(1,2,figsize=(11,4.7),constrained_layout=True)
    for ax, (mode,art) in zip(axes,cases.items()):
        truth=art['truth']; res=truth.resolution
        image=art['risk_true'].slip_proxy + 1.8*art['risk_true'].slope_rad + 2*art['risk_true'].roughness_m
        ax.imshow(image,origin='lower',extent=(-res/2,(truth.shape[1]-.5)*res,-res/2,(truth.shape[0]-.5)*res),cmap='viridis')
        if scene=='unknown':
            ax.contour(np.arange(truth.shape[1])*res,np.arange(truth.shape[0])*res,
                       (~truth.observed).astype(float),levels=[.5],colors=['white'],linewidths=1.5)
        p=np.array([truth.xy(i) for i in art['path']])
        if len(p):ax.plot(p[:,0],p[:,1],label='global path',linewidth=1.8)
        q=np.array(art['trajectory'])
        if len(q):ax.plot(q[:,0],q[:,1],label='executed',linewidth=1.8,linestyle='--')
        ax.plot(*truth.xy((22,4)),marker='o',markersize=8,label='start')
        ax.plot(*truth.xy((22,42)),marker='*',markersize=10,label='goal')
        ax.set(title=f'{scene}: {mode}',xlabel='x [m]',ylabel='y [m]')
        ax.set_aspect('equal');ax.legend(loc='upper right',fontsize=7)
    fig.savefig(path,dpi=140)
    plt.close(fig)
