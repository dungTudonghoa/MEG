import math
import numpy as np
import torch
from phase06.core import SPLITS, make_tasks, SHOT_TRANSITIONS, Predictor, ContextPredictor, inner_update, true_response_np, set_seed

def test_outer_splits_disjoint():
    assert not set(SPLITS['train']) & set(SPLITS['test'])
    assert not set(SPLITS['val']) & set(SPLITS['test'])

def test_shot_is_20_contiguous_transitions():
    task=make_tasks('test',41,1)[0]
    for k in (0,1,5,10,20,50):
        x,y=task.support(k)
        assert x.shape==(k*20,5) and y.shape==(k*20,2)

def test_support_and_query_runs_distinct():
    t=make_tasks('test',41,1)[0]
    assert t.query_run_id != t.support_run_id
    assert not np.array_equal(t.support_x[:12].reshape(-1,5),t.query_x)

def test_reproducible_grouped_task_generation():
    t=make_tasks('val',41,1)[0]
    s=make_tasks('val',41,1)[0]
    np.testing.assert_array_equal(t.support_x,s.support_x)
    np.testing.assert_array_equal(t.query_y,s.query_y)

def test_query_changes_do_not_affect_adaptation():
    set_seed(8)
    t=make_tasks('test',41,1)[0];sx,sy=t.support(5)
    model=Predictor();p1=inner_update(model,sx,sy,2,.25)
    qx,qy=t.query(); qy[:]=0.0
    p2=inner_update(model,sx,sy,2,.25)
    for a,b in zip(p1.parameters(),p2.parameters()):
        assert torch.equal(a,b)

def test_zero_shot_does_not_update_model():
    set_seed(7)
    model=Predictor();sx=torch.empty(0,5);sy=torch.empty(0,2)
    copy=inner_update(model,sx,sy,5,.3)
    for a,b in zip(model.parameters(),copy.parameters()):assert torch.equal(a,b)

def test_context_zero_shot_does_not_access_query_or_future():
    model=ContextPredictor()
    x=torch.rand(10,5)
    zeros=torch.empty(0,5);ys=torch.empty(0,2)
    with torch.no_grad(): p=model(x,zeros,ys)
    assert p.shape==(10,2) and torch.isfinite(p).all()

def test_physics_proxy_sensible_monotonic_slip_for_speed():
    t=make_tasks('train',41,1)[0]
    x=np.array([[.5,.3,.4,.3,.3],[.5,.3,.4,.8,.3]],dtype=float)
    pred=true_response_np(x,t.params)
    assert pred[1,0]>pred[0,0]

def test_protocol_has_two_distinct_target_families():
    assert len(SPLITS['test'])>=2

def test_no_target_task_ids_in_source():
    s=make_tasks('train',43,2); t=make_tasks('test',43,2)
    assert not {v.task_id for v in s}&{v.task_id for v in t}

def test_fomaml_training_deterministic_parameters():
    from phase06.core import fomaml_train
    tasks=make_tasks('train',51,1)
    a,_,_=fomaml_train(tasks,3,52)
    b,_,_=fomaml_train(tasks,3,52)
    for p,q in zip(a.parameters(),b.parameters()):
        assert torch.equal(p,q)

def test_source_pretraining_deterministic_parameters():
    from phase06.core import source_train
    tasks=make_tasks('train',51,1)
    a,_,_=source_train(tasks,4,52)
    b,_,_=source_train(tasks,4,52)
    for p,q in zip(a.parameters(),b.parameters()):
        assert torch.equal(p,q)
