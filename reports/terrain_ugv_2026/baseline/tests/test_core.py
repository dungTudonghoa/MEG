import math
import numpy as np
import pytest
from terrain_baseline.model import TerrainGrid, Vehicle
from terrain_baseline.terrain import assess
from terrain_baseline.planner import astar
from terrain_baseline.sensors import ElevationSensorLoader,GridObservation
from terrain_baseline.scenarios import make_scene, START, GOAL, SCENARIOS
from terrain_baseline.experiments import execute


def test_grid_validation():
    with pytest.raises(ValueError):
        TerrainGrid(np.ones((3,3)),np.ones((3,2)),np.zeros((3,3)),np.ones((3,3)))


def test_npz_roundtrip(tmp_path):
    g=make_scene('flat',42)
    f=tmp_path/'grid.npz'
    ElevationSensorLoader.save(f,g)
    q=ElevationSensorLoader.load(f)
    assert np.array_equal(q.elevation,g.elevation)
    assert q.resolution==g.resolution


def test_observation_masks_private_data():
    g=make_scene('unknown',42)
    obs=GridObservation.from_truth(g)
    assert np.all(obs.traction[~g.observed]==1)
    assert np.all(obs.elevation[~g.observed]==0)
    assert np.all(obs.observed==g.observed)


def test_terrain_avoids_unknown_and_true_hazard():
    robot=Vehicle()
    g=make_scene('unknown',42)
    r=assess(GridObservation.from_truth(g),robot)
    p,_=astar(GridObservation.from_truth(g),r,START,GOAL,'terrain')
    assert p
    assert all(g.observed[q] for q in p)
    assert all(not r.forbidden[q] for q in p)


def test_path_corner_cutting_forbidden():
    obs=np.ones((3,3),dtype=bool)
    ob=np.zeros((3,3),dtype=bool)
    ob[0,1]=True; ob[1,0]=True
    g=TerrainGrid(np.zeros((3,3)),np.ones((3,3)),ob,obs,1.0)
    r=assess(g,Vehicle(radius_m=0))
    path,_=astar(g,r,(0,0),(2,2),'geometry')
    assert path==[]


@pytest.mark.parametrize('scene',SCENARIOS)
def test_repeatable_path_and_outcome(scene):
    robot=Vehicle()
    a,art1=execute(scene,42,'terrain',robot)
    b,art2=execute(scene,42,'terrain',robot)
    assert art1['path']==art2['path']
    assert a['success']==b['success']
    assert a['stop_reason']==b['stop_reason']
    assert np.allclose(art1['trajectory'],art2['trajectory'])
    assert a['executed_constraint_violations']==0


def test_terrain_plans_are_not_unsafe():
    for scene in SCENARIOS:
        row,_=execute(scene,42,'terrain',Vehicle())
        assert row['unsafe_plan']==0


def test_shared_supervisor_prevents_unsafe_motion():
    # A geometry-only planner may propose a hazardous path, but supervisor stops.
    row,_=execute('hazard',42,'geometry',Vehicle())
    assert row['unsafe_plan']==1
    assert row['executed_constraint_violations']==0
    assert row['safety_stops']==1


def test_unknown_fallback_never_oracle_exposure():
    g=make_scene('unknown',42)
    est=GridObservation.from_truth(g)
    est.traction[~g.observed]=0.77 # alters only unobserved placeholders
    # terrain A* blocks unknown cells regardless of placeholders
    p,_=astar(est,assess(est,Vehicle()),START,GOAL,'terrain')
    assert p and all(g.observed[q] for q in p)


def test_odometry_estimator_known_motion():
    from terrain_baseline.estimation import OdometryEstimator
    est=OdometryEstimator(0.,0.,0.)
    est.step(1.,0.,1.)
    assert est.x==pytest.approx(1.) and est.y==pytest.approx(0.)
    est.step(0.,math.pi/2,1.)
    assert est.yaw==pytest.approx(math.pi/2)
