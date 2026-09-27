"""Small temporal voxel map used for short-horizon obstacle reconstruction."""

from dataclasses import dataclass

import numpy as np


@dataclass
class _Voxel:
    hits: int
    last_seen: float


class TemporalVoxelMap:
    """Fuse point-cloud frames into confirmed voxels with bounded persistence."""

    def __init__(
        self,
        voxel_size: float = 0.03,
        minimum_hits: int = 2,
        decay_seconds: float = 2.0,
        max_voxels: int = 50000,
    ):
        if voxel_size <= 0.0:
            raise ValueError("voxel_size must be positive")
        if minimum_hits < 1:
            raise ValueError("minimum_hits must be at least one")
        self.voxel_size = float(voxel_size)
        self.minimum_hits = int(minimum_hits)
        self.decay_seconds = max(0.0, float(decay_seconds))
        self.max_voxels = max(1, int(max_voxels))
        self._voxels = {}
        self._last_stamp = None

    def clear(self):
        self._voxels.clear()
        self._last_stamp = None

    def update(self, points, stamp: float) -> np.ndarray:
        """Insert one frame and return confirmed voxel centres in map frame."""
        stamp = float(stamp)
        if self._last_stamp is not None and stamp < self._last_stamp:
            # Gazebo reset / rosbag loop: old timestamps must not keep occupancy alive.
            self._voxels.clear()
        self._last_stamp = stamp
        self._prune(stamp)

        cloud = np.asarray(points, dtype=np.float64).reshape((-1, 3))
        cloud = cloud[np.isfinite(cloud).all(axis=1)]
        if len(cloud):
            keys = np.floor(cloud / self.voxel_size).astype(np.int64)
            for key_array in np.unique(keys, axis=0):
                key = tuple(int(value) for value in key_array)
                voxel = self._voxels.get(key)
                if voxel is None:
                    self._voxels[key] = _Voxel(1, stamp)
                else:
                    voxel.hits = min(voxel.hits + 1, self.minimum_hits + 8)
                    voxel.last_seen = stamp

        if len(self._voxels) > self.max_voxels:
            oldest = sorted(self._voxels, key=lambda key: self._voxels[key].last_seen)
            for key in oldest[: len(self._voxels) - self.max_voxels]:
                del self._voxels[key]

        occupied = [key for key, voxel in self._voxels.items()
                    if voxel.hits >= self.minimum_hits]
        if not occupied:
            return np.empty((0, 3), dtype=np.float64)
        return (np.asarray(occupied, dtype=np.float64) + 0.5) * self.voxel_size

    def _prune(self, stamp: float):
        expired = [key for key, voxel in self._voxels.items()
                   if stamp - voxel.last_seen > self.decay_seconds]
        for key in expired:
            del self._voxels[key]
