"""
Model construction and discretization utilities.

This module provides functions to build the continuous-time model from
matlab/prep_final.m and discretize it using zero-order hold (ZOH) to match
MATLAB's c2d() function.
"""

import numpy as np
from scipy.signal import cont2discrete


def build_continuous_model():
    """
    Build the continuous-time system matrices from prep_final.m.
    
    Matrices are hardcoded (not parsed from MATLAB text) to ensure
    exact numerical match and avoid parsing brittleness.
    
    Returns:
        tuple: (A, B, C) where
            - A: (12, 12) continuous-time state matrix
            - B: (12, 3) continuous-time input matrix
            - C: (1, 12) continuous-time output matrix
    """
    # Continuous-time system matrices hardcoded from prep_final.m (lines 4-32)
    # These are copied exactly once from the MATLAB file, not parsed
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
    
    # Output matrix: measures displacement of mass 1 (x1) only
    C = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
    
    return A, B, C


def discretize_zoh(A, B, C, Ts):
    """
    Discretize continuous-time system using zero-order hold (ZOH).
    
    This function matches MATLAB's c2d(ss(A, B, C, 0), Ts) convention.
    
    Args:
        A: (n, n) continuous-time state matrix
        B: (n, m) continuous-time input matrix
        C: (p, n) continuous-time output matrix
        Ts: Sampling time in seconds
    
    Returns:
        tuple: (Ad, Bd, Cd) where
            - Ad: (n, n) discrete-time state matrix
            - Bd: (n, m) discrete-time input matrix
            - Cd: (p, n) discrete-time output matrix (same as C)
    """
    # scipy.signal.cont2discrete expects D matrix as well
    # MATLAB convention: D = zeros(size(C, 1), size(B, 2))
    p = C.shape[0]
    m = B.shape[1]
    D = np.zeros((p, m))
    
    # Discretize using zero-order hold
    # cont2discrete returns (Ad, Bd, Cd, Dd, dt)
    # This matches MATLAB's c2d(ss(A, B, C, D), Ts) with D=zeros(size(C,1), size(B,2))
    Ad, Bd, Cd, Dd, _ = cont2discrete((A, B, C, D), dt=Ts, method='zoh')
    
    # Note: Cd from cont2discrete should be the same as C
    # but we use the returned value for consistency
    # Dd should be all zeros (since D is zero), but we return it for validation
    return Ad, Bd, Cd, Dd


def load_reference_matrices(mat_file_path):
    """
    Load reference discrete-time matrices from MATLAB .mat file.
    
    This is optional and used for detailed comparison if needed.
    
    Args:
        mat_file_path: Path to .mat file containing 'Ad', 'Bd', 'Cd', 'Dd'
    
    Returns:
        tuple: (Ad, Bd, Cd, Dd) discrete-time matrices from MATLAB
    """
    try:
        from scipy.io import loadmat
        data = loadmat(mat_file_path)
        Ad = data['Ad']
        Bd = data['Bd']
        Cd = data['Cd']
        # Dd should exist (all zeros), include it for shape validation
        if 'Dd' in data:
            Dd = data['Dd']
        else:
            # Fallback: assume Dd is zeros if not exported
            p = Cd.shape[0]
            m = Bd.shape[1]
            Dd = np.zeros((p, m))
        return Ad, Bd, Cd, Dd
    except ImportError:
        raise ImportError("scipy.io is required to load .mat files")
    except KeyError as e:
        raise KeyError(f"Missing key in .mat file: {e}")

