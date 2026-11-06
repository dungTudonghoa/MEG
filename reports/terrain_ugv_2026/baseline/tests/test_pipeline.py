import csv
import math
import numpy as np
from terrain_baseline.experiments import run
from terrain_baseline.estimation import OdometryEstimator


def test_full_pipeline_exports(tmp_path):
    runs, summary=run({'scenarios':['flat','hazard'],'seeds':[42],
        'planners':['geometry','terrain']},tmp_path,make_plots=False)
    assert len(runs)==4
    assert len(summary)==4
    for f in ('metrics_raw.csv','metrics_summary.csv','run_manifest.json','MEASURED_RESULTS.md'):
        assert (tmp_path/f).exists()
    assert sum(int(row['success']) for row in runs)==3
    with (tmp_path/'metrics_raw.csv').open() as f:
        assert len(list(csv.DictReader(f)))==4


def test_zero_motion_estimator_no_drift():
    e=OdometryEstimator(2,3,1.)
    for i in range(100):e.step(0.,0.,0.1)
    assert (e.x,e.y,e.yaw)==(2,3,1.)
