"""
Authors:
    Rodrigo Perez-Dattari <r.j.perezdattari@tudelft.nl>
"""

from dataclasses import dataclass
import numpy as np


@dataclass
class Params:
    stiffness_pos: np.ndarray = np.array([600.0, 600.0, 500.0, 450.0, 180.0, 100.0, 50.0])
    damping_pos: np.ndarray = np.array([3.0, 3.0, 1.5, 1.5, 0.8, 0.8, 0.8]) * np.sqrt(stiffness_pos)
    stiffness_vel: np.ndarray = np.array([600.0, 600.0, 500.0, 450.0, 180.0, 100.0, 50.0])
    damping_vel: np.ndarray = damping_pos * np.array([1.0, 1.0, 1.0, 1.0, 0.8, 0.8, 0.8])
    alpha: np.ndarray = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
    max_q_delta: np.ndarray = np.array([np.pi/64, np.pi/64, np.pi/64, np.pi/64, np.pi/64, np.pi/64, np.pi/64])
