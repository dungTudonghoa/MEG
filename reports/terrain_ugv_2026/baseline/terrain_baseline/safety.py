from __future__ import annotations
import numpy as np
from .model import TerrainGrid, RiskMap, Vehicle


class SafetyChecker:
    """Ground-truth checker is a simulator-only oracle, never given to the planner."""
    def __init__(self, grid: TerrainGrid, risk: RiskMap, robot: Vehicle):
        self.grid=grid
        self.risk=risk
        self.robot=robot

    def check_pose(self, x: float, y: float) -> tuple[bool,str]:
        n=8
        for a in np.linspace(0,2*np.pi,n,endpoint=False):
            for rad in (0,self.robot.radius_m):
                xx=x+rad*np.cos(a); yy=y+rad*np.sin(a)
                ij=self.grid.ij((xx,yy))
                if not self.grid.inside(ij):return False,'outside'
                if self.grid.obstacle[ij]:return False,'collision'
                if self.risk.slope_rad[ij]>np.deg2rad(self.robot.slope_max_deg):return False,'slope'
                if self.risk.roughness_m[ij]>self.robot.roughness_max_m:return False,'roughness'
                if self.risk.slip_proxy[ij]>self.robot.slip_proxy_max:return False,'slip_proxy'
        return True,'ok'

    def check_segment(self, a: tuple[float,float], b: tuple[float,float]) -> tuple[bool,str]:
        d=np.hypot(b[0]-a[0],b[1]-a[1]); n=max(2,int(np.ceil(d/(self.grid.resolution/5))))
        for u in np.linspace(0,1,n):
            ok,reason=self.check_pose(a[0]+u*(b[0]-a[0]),a[1]+u*(b[1]-a[1]))
            if not ok:return False,reason
        return True,'ok'
