"""
Authors:
    Rodrigo Perez-Dattari <r.j.perezdattari@tudelft.nl>
"""

from dataclasses import dataclass
import numpy as np


@dataclass
class Params:
    stiffness_pos: np.ndarray = np.array([700.0, 700.0, 650.0, 550.0, 300.0, 200.0, 60.0])
    damping_pos: np.ndarray = np.array([3.0, 3.0, 1.5, 1.5, 0.8, 0.8, 0.8]) * np.sqrt(stiffness_pos)
    stiffness_vel: np.ndarray = np.array([700.0, 700.0, 650.0, 550.0, 250.0, 120.0, 60.0]) * 1.0
    damping_vel: np.ndarray = damping_pos * np.array([1.0, 1.0, 1.0, 1.0, 0.8, 0.8, 0.8]) * 0.7
    alpha: np.ndarray = np.array([0.2, 0.2, 0.2, 0.3, 0.8, 0.9, 0.9])
    max_q_delta: np.ndarray = np.array([np.pi/64, np.pi/64, np.pi/64, np.pi/64, np.pi/64, np.pi/64, np.pi/64])