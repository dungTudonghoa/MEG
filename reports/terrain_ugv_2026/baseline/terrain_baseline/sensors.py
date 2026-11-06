from __future__ import annotations
from pathlib import Path
import numpy as np
from .model import TerrainGrid


class ElevationSensorLoader:
    """NPZ field loader. Supports synthetic grid observations, NOT raw ROS bag decoding."""
    @staticmethod
    def load(path: str | Path) -> TerrainGrid:
        with np.load(path, allow_pickle=False) as d:
            required = {'elevation', 'traction', 'obstacle', 'observed', 'resolution'}
            if not required <= set(d.files):
                raise ValueError(f'Missing NPZ fields: {sorted(required - set(d.files))}')
            return TerrainGrid(d['elevation'], d['traction'], d['obstacle'], d['observed'], float(d['resolution']))

    @staticmethod
    def save(path: str | Path, grid: TerrainGrid):
        np.savez_compressed(path, elevation=grid.elevation, traction=grid.traction,
                            obstacle=grid.obstacle, observed=grid.observed, resolution=grid.resolution)


class GridObservation:
    """Only observed cells are supplied to the estimator (no hidden ground truth)."""
    @staticmethod
    def from_truth(grid: TerrainGrid) -> TerrainGrid:
        # Invalid observations are zero-filled but remain masked as *unobserved*.
        obs = grid.observed
        return TerrainGrid(np.where(obs, grid.elevation, 0.0),
                           np.where(obs, grid.traction, 1.0),
                           grid.obstacle & obs, obs.copy(), grid.resolution)
