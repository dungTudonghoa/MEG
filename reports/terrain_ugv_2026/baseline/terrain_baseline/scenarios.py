from __future__ import annotations
import numpy as np
from scipy.ndimage import gaussian_filter
from .model import TerrainGrid

SCENARIOS = ('flat','slope','rough','hazard','unknown')
START = (22, 4)
GOAL = (22, 42)


def make_scene(name: str, seed: int = 42) -> TerrainGrid:
    if name not in SCENARIOS: raise ValueError(f'Unknown scenario {name}')
    rng = np.random.default_rng(seed)
    h, w = 45, 47
    rows, cols = np.mgrid[0:h,0:w]
    z = np.zeros((h,w), dtype=float)
    mu = np.full((h,w), 0.93, dtype=float)
    obstacle = np.zeros((h,w),dtype=bool)
    obs = np.ones((h,w),dtype=bool)
    # Low ambient terrain roughness; sampling controlled by seed, not robot actions.
    z += 0.004 * gaussian_filter(rng.standard_normal((h,w)), 1.2)
    if name == 'slope':
        # tall ridge across direct route, safe northern / southern bypass
        z += 2.05 * np.exp(-0.5*((cols-24)/4.0)**2 - 0.5*((rows-22)/5.3)**2)
    if name == 'rough':
        s = gaussian_filter(rng.standard_normal((h,w)), 0.65)
        patch = (np.abs(cols-24)<8) & (np.abs(rows-22)<7)
        z += patch * 0.48 * s
    if name in ('hazard', 'unknown'):
        danger = ((cols>=17)&(cols<=29)&(rows>=16)&(rows<=28))
        mu[danger] = 0.12
    if name == 'unknown':
        obs[(cols>=14)&(cols<=31)&(rows>=14)&(rows<=30)] = False
    return TerrainGrid(z,mu,obstacle,obs,resolution=0.38)
