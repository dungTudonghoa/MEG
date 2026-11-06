from __future__ import annotations
import math
import numpy as np
from .model import TerrainGrid, Vehicle, wrap
from .safety import SafetyChecker
from .estimation import OdometryEstimator


class DeterministicDiffDrive:
    """Synthetic ideal differential-drive body twist with traction-dependent speed loss.

    Ignores sinkage, tire force, lateral slip, suspension and high-speed instability.
    This is a test harness, not Gazebo, Chrono, or physical ground truth.
    """
    def __init__(self, grid: TerrainGrid, robot: Vehicle):
        self.grid=grid; self.robot=robot

    def step(self, state, v_cmd:float, w_cmd:float):
        x,y,theta,v_old=state
        dt=self.robot.dt_s
        v_cmd=float(np.clip(v_cmd,0,self.robot.v_max_mps))
        v=float(np.clip(v_cmd, max(0.,v_old-self.robot.accel_max_mps2*dt),v_old+self.robot.accel_max_mps2*dt))
        w=float(np.clip(w_cmd,-self.robot.w_max_rps,self.robot.w_max_rps))
        ij=self.grid.ij((x,y))
        traction=self.grid.traction[ij] if self.grid.inside(ij) else 0.
        slowdown=0.025+0.28*(1-traction)
        actual_v=v*(1-slowdown)
        th_new=wrap(theta+w*dt)
        th_mid=theta+0.5*w*dt
        return (x+actual_v*math.cos(th_mid)*dt,y+actual_v*math.sin(th_mid)*dt,th_new,v)


def _distance_to_segments(p, poly):
    if len(poly)==1:return math.dist(p,poly[0])
    P=np.array(p); xx=np.array(poly)
    a=xx[:-1]; b=xx[1:]; v=b-a
    u=np.sum((P-a)*v,axis=1)/np.maximum(np.sum(v*v,axis=1),1e-12)
    q=a+v*np.clip(u,0,1)[:,None]
    return float(np.min(np.linalg.norm(q-P,axis=1)))


def follow_path(grid: TerrainGrid, true_checker: SafetyChecker, path: list[tuple[int,int]], robot: Vehicle,
                *, max_steps=4000) -> dict:
    if not path:
        return {'success':False,'stop_reason':'no_path','trajectory':[],'steps':0,'safety_stops':0,
                'path_length_actual_m':0.,'travel_time_s':0.,'tracking_rmse_m':float('nan')}
    xy=[grid.xy(q) for q in path]
    start=xy[0]
    state=(*start,0.0,0.0)
    estimator=OdometryEstimator(*start)
    track=[(state[0],state[1],state[2],0.)]
    estimated_track=[(start[0],start[1],0.,0.)]
    dists=[]
    controls=[]
    simulator=DeterministicDiffDrive(grid,robot)
    j=1; total=0.0; reason='timeout'; stops=0
    for k in range(max_steps):
        if j>=len(xy):
            if math.dist((state[0],state[1]),xy[-1]) <= robot.goal_tolerance_m:
                reason='goal_reached'
            else: reason='waypoints_exhausted'
            break
        # Controller observes dead-reckoned odometry, not simulator x/y truth.
        x,y,th=estimator.x,estimator.y,estimator.yaw
        true_x,true_y,_,vold=state
        tx,ty=xy[j]
        remaining=math.hypot(tx-x,ty-y)
        if remaining<=robot.goal_tolerance_m:
            j+=1;continue
        desired=math.atan2(ty-y,tx-x)
        err=wrap(desired-th)
        if abs(err)>0.09:
            v_cmd=0.0
            w_cmd=np.clip(3.0*err,-robot.w_max_rps,robot.w_max_rps)
        else:
            # rotate-then-forward, ideal diff-drive, bounded speed/acceleration
            v_cmd=min(robot.v_max_mps,remaining/max(robot.dt_s,1e-6))
            w_cmd=np.clip(2.0*err,-robot.w_max_rps,robot.w_max_rps)
        next_state=simulator.step(state,v_cmd,w_cmd)
        good, why=true_checker.check_segment((true_x,true_y),(next_state[0],next_state[1]))
        if not good:
            reason='supervisor_stop_'+why; stops=1; break
        actual_step=math.hypot(next_state[0]-true_x,next_state[1]-true_y)
        total+=actual_step
        w_meas=wrap(next_state[2]-state[2])/robot.dt_s
        estimator.step(actual_step/robot.dt_s,w_meas,robot.dt_s)
        state=next_state
        track.append((state[0],state[1],state[2],(k+1)*robot.dt_s))
        estimated_track.append((estimator.x,estimator.y,estimator.yaw,(k+1)*robot.dt_s))
        controls.append((v_cmd,w_cmd))
        dists.append(_distance_to_segments((state[0],state[1]),xy))
    else:
        reason='timeout'
    success= reason=='goal_reached'
    return {'success':success,'stop_reason':reason,'trajectory':track,'steps':len(track)-1,
            'safety_stops':stops,'path_length_actual_m':total,
            'travel_time_s':(len(track)-1)*robot.dt_s,
            'tracking_rmse_m':float(np.sqrt(np.mean(np.square(dists)))) if dists else float('nan'),
            'control_commands':controls,'estimated_trajectory':estimated_track}
