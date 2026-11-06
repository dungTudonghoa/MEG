from __future__ import annotations
import math
from dataclasses import dataclass
from .model import wrap


@dataclass
class OdometryEstimator:
    """Differential-drive proprioceptive dead reckoning from measured v and yaw rate.

    No position/orientation correction (GPS, VIO, EKF) is claimed. Without noise,
    it tracks the synthetic simulator exactly; with drift it will diverge.
    """
    x: float
    y: float
    yaw: float = 0.0

    def step(self, v_meas_mps: float, w_meas_rps: float, dt_s: float):
        if dt_s<=0:raise ValueError('dt must be positive')
        mid=self.yaw+0.5*w_meas_rps*dt_s
        self.x+=v_meas_mps*math.cos(mid)*dt_s
        self.y+=v_meas_mps*math.sin(mid)*dt_s
        self.yaw=wrap(self.yaw+w_meas_rps*dt_s)
        return self.x,self.y,self.yaw
