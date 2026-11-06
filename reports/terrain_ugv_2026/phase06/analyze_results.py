"""Paired cluster bootstrap on held-out synthetic tasks (seed and task IDs).
No frame-level IID confidence intervals. Latency is environment-specific.
"""
from pathlib import Path
import numpy as np
import pandas as pd
root=Path(__file__).parent/'results'/'pilot_v02'
rows=pd.read_csv(root/'episode_metrics.csv')
rows=rows[rows.split=='test']
rng=np.random.default_rng(20261106)
summary=[]
for k in sorted(rows.k_windows.unique()):
    a=rows[rows.k_windows==k].pivot(index=['seed','task'],columns='method',values='slip_mae')
    for method in ('fine_tune','tta_head','fomaml','context'):
        d=(a[method]-a['fine_tune']).rename('delta').reset_index()
        d['family']=d.task.str.split('/').str[1]
        groups=d.groupby(['seed','family']).delta.mean().values
        # Resample 4 seed-by-family groups; not frame-wise iid.
        bootstrap=np.mean(rng.choice(groups,size=(10000,len(groups)),replace=True),axis=1)
        lo,hi=np.quantile(bootstrap,[.025,.975])
        summary.append({'k_windows':k,'method':method,'reference':'fine_tune',
                        'n_paired_tasks':len(d),'n_seed_family_clusters':len(groups),
                        'mean_delta_slip_mae':float(np.mean(d.delta)),
                        'cluster_bootstrap_ci95_lo':float(lo),'cluster_bootstrap_ci95_hi':float(hi)})
pd.DataFrame(summary).to_csv(root/'paired_bootstrap.csv',index=False)
print(pd.DataFrame(summary).query('k_windows in [1,5,10,20,50] and method in ["fomaml","context"]').round(4).to_string(index=False))
