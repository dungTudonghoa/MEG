from __future__ import annotations
import numpy as np
from scipy.ndimage import distance_transform_edt, uniform_filter, binary_dilation
from .model import TerrainGrid, Vehicle, RiskMap


def assess(grid: TerrainGrid, robot: Vehicle, unknown_forbidden: bool = True) -> RiskMap:
    obs = grid.observed
    if np.any(obs):
        nearest = distance_transform_edt(~obs, return_distances=False, return_indices=True)
        z = grid.elevation[tuple(nearest)]
    else:
        z = np.zeros_like(grid.elevation)
    gy, gx = np.gradient(z, grid.resolution)
    slope = np.arctan(np.hypot(gx, gy))
    # 3x3 detrended elevation variation (planar slope excluded approximately).
    # For truly nonplanar patches the proxy responds to high-frequency height changes.
    residual = z - uniform_filter(z, size=3, mode='nearest')
    rough = np.sqrt(uniform_filter(residual * residual, size=3, mode='nearest'))
    # A deliberately synthetic risk heuristic, NOT measured wheel slip or friction law.
    slip = np.clip(0.04 + 0.63 * (1 - grid.traction) + 0.12 * np.minimum(rough / 0.10, 1), 0, 1)
    hard = (slope > np.deg2rad(robot.slope_max_deg)) | (rough > robot.roughness_max_m) | (slip > robot.slip_proxy_max)
    hard |= grid.obstacle
    if unknown_forbidden:
        hard |= ~obs
    # Inflate obstacles and dangerous cells by a discrete circular footprint;
    # simulator separately checks metric footprint at continuous poses.
    # Conservative grid clearance: footprint + half cell diagonal + ~0.13 m
    # tracking allowance. This is not a certified tracking-error bound.
    radius = robot.radius_m + 0.5 * grid.resolution * np.sqrt(2) + 0.13
    k = int(np.ceil(radius / grid.resolution))
    if k:
        jj, ii = np.mgrid[-k:k+1, -k:k+1]
        footprint = (ii * grid.resolution) ** 2 + (jj * grid.resolution) ** 2 <= (radius + 1e-9) ** 2
        hard = binary_dilation(hard, structure=footprint)
    return RiskMap(slope, rough, slip, hard, obs.copy())
