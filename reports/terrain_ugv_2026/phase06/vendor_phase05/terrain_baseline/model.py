from __future__ import annotations
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class Vehicle:
    # Ideal differential-drive twist interface. v,w are limited body twists.
    radius_m: float = 0.18
    v_max_mps: float = 0.75
    w_max_rps: float = 1.15
    accel_max_mps2: float = 0.65
    slope_max_deg: float = 17.0
    roughness_max_m: float = 0.100
    slip_proxy_max: float = 0.56
    dt_s: float = 0.16
    goal_tolerance_m: float = 0.14


@dataclass
class TerrainGrid:
    elevation: np.ndarray
    traction: np.ndarray
    obstacle: np.ndarray
    observed: np.ndarray
    resolution: float = 0.38
    # grid indexed [row, col]; poses in x=col*resolution, y=row*resolution

    def __post_init__(self):
        self.elevation = np.asarray(self.elevation, dtype=float)
        self.traction = np.asarray(self.traction, dtype=float)
        self.obstacle = np.asarray(self.obstacle, dtype=bool)
        self.observed = np.asarray(self.observed, dtype=bool)
        dims = {tuple(x.shape) for x in (self.elevation, self.traction, self.obstacle, self.observed)}
        if len(dims) != 1 or self.elevation.ndim != 2:
            raise ValueError('All terrain channels must share a 2D shape')
        if self.resolution <= 0 or not np.isfinite(self.elevation).all():
            raise ValueError('Invalid elevation or resolution')
        if np.any(self.traction < 0) or np.any(self.traction > 1):
            raise ValueError('traction must be in [0,1]')

    @property
    def shape(self):
        return self.elevation.shape

    def xy(self, ij: tuple[int, int]):
        r, c = ij
        return float(c * self.resolution), float(r * self.resolution)

    def ij(self, xy: tuple[float, float]):
        x, y = xy
        return (int(round(y / self.resolution)), int(round(x / self.resolution)))

    def inside(self, ij: tuple[int, int]):
        return 0 <= ij[0] < self.shape[0] and 0 <= ij[1] < self.shape[1]


@dataclass
class RiskMap:
    slope_rad: np.ndarray
    roughness_m: np.ndarray
    slip_proxy: np.ndarray
    forbidden: np.ndarray
    observed: np.ndarray


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi
