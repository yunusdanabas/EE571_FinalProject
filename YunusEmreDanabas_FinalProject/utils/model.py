# Model construction and discretization utilities - built incrementally by part agents

import numpy as np
from scipy.signal import cont2discrete


def build_continuous_model():
    """Build continuous-time system matrices from prep_final.m."""
    A = np.array([
        [0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0],
        [0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0],
        [0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0],
        [0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0],
        [0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0],
        [0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1],
        [-2,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0],
        [1, -2,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0],
        [0,  1, -2,  1,  0,  0,  0,  0,  0,  0,  0,  0],
        [0,  0,  1, -2,  1,  0,  0,  0,  0,  0,  0,  0],
        [0,  0,  0,  1, -2,  1,  0,  0,  0,  0,  0,  0],
        [0,  0,  0,  0,  1, -1,  0,  0,  0,  0,  0,  0]
    ])
    
    B = np.array([
        [0,  0,  0],
        [0,  0,  0],
        [0,  0,  0],
        [0,  0,  0],
        [0,  0,  0],
        [0,  0,  0],
        [1,  0,  0],
        [-1,  0, -1],
        [0,  0,  0],
        [0,  0,  0],
        [0,  1,  0],
        [0, -1,  0]
    ])
    
    C = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
    
    return A, B, C


def discretize_zoh(A, B, C, Ts):
    """Discretize using zero-order hold to match MATLAB's c2d()."""
    p = C.shape[0]
    m = B.shape[1]
    D = np.zeros((p, m))
    Ad, Bd, Cd, Dd, _ = cont2discrete((A, B, C, D), dt=Ts, method='zoh')
    return Ad, Bd, Cd, Dd
