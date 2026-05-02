import math

import numpy as np

def to_yaw(zz, ww) -> float:
    return math.atan2(2 * zz * ww, 1 - 2 * zz * zz)

def to_homo(x, y, yaw) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([
        [c, -s, 0, x],
        [s,  c, 0, y],
        [0,  0, 1, 0],
        [0,  0, 0, 1],
    ])

def to_nanosec(sec: int, nanosec: int) -> int:
    return nanosec + sec * 1_000_000_000
