"""
Metrics and validation utilities.

This module provides functions for dimension checking, cost computation,
input metrics, and estimation error metrics.
"""

import numpy as np


def check_dimensions(Ad, Bd, Cd, expected_n=12, expected_m=3, expected_p=1):
    """
    Check dimensions of discrete-time system matrices.
    
    Args:
        Ad: (n, n) discrete-time state matrix
        Bd: (n, m) discrete-time input matrix
        Cd: (p, n) discrete-time output matrix
        expected_n: Expected number of states
        expected_m: Expected number of inputs
        expected_p: Expected number of outputs
    
    Returns:
        dict: Dictionary with dimension check results
            - 'n': Actual number of states
            - 'm': Actual number of inputs
            - 'p': Actual number of outputs
            - 'Ad_shape': Shape of Ad matrix
            - 'Bd_shape': Shape of Bd matrix
            - 'Cd_shape': Shape of Cd matrix
            - 'Ad_valid': Whether Ad is square and matches expected_n
            - 'Bd_valid': Whether Bd dimensions match expectations
            - 'Cd_valid': Whether Cd dimensions match expectations
            - 'all_valid': Whether all dimensions are correct
    """
    n_actual = Ad.shape[0]
    m_actual = Bd.shape[1]
    p_actual = Cd.shape[0]
    
    Ad_valid = (Ad.shape == (expected_n, expected_n))
    Bd_valid = (Bd.shape == (expected_n, expected_m))
    Cd_valid = (Cd.shape == (expected_p, expected_n))
    
    all_valid = Ad_valid and Bd_valid and Cd_valid
    
    return {
        'n': n_actual,
        'm': m_actual,
        'p': p_actual,
        'Ad_shape': Ad.shape,
        'Bd_shape': Bd.shape,
        'Cd_shape': Cd.shape,
        'Ad_valid': Ad_valid,
        'Bd_valid': Bd_valid,
        'Cd_valid': Cd_valid,
        'all_valid': all_valid
    }


def compute_cost(u, y, Q=None, R=None):
    """
    Compute finite-horizon LQR cost.
    
    Cost function: J = Σ(uᵀRu + yᵀQy) over time steps
    
    This is a placeholder for Part 3+ implementation.
    
    Args:
        u: (m, N) input sequence
        y: (p, N) output sequence
        Q: (p, p) output weight matrix. If None, uses identity.
        R: (m, m) input weight matrix. If None, uses identity.
    
    Returns:
        float: Total cost J
    """
    # Placeholder implementation - will be fully implemented in Part 3
    raise NotImplementedError("Cost computation will be implemented in Part 3")


def max_input_magnitude(u):
    """
    Compute maximum input magnitude for each channel.
    
    This is a placeholder for Part 3+ implementation.
    
    Args:
        u: (m, N) input sequence
    
    Returns:
        dict: Maximum magnitude per channel and overall
            - 'per_channel': (m,) array of max(|u_i|) for each channel
            - 'overall': Maximum magnitude across all channels and time
    """
    # Placeholder implementation - will be fully implemented in Part 3
    raise NotImplementedError("Input magnitude metrics will be implemented in Part 3")


def rms_error(x_true, x_est):
    """
    Compute RMS (root mean square) estimation error.
    
    This is a placeholder for Part 5+ implementation.
    
    Args:
        x_true: (n, N) true state trajectory
        x_est: (n, N) estimated state trajectory
    
    Returns:
        dict: RMS error metrics
            - 'per_state': (n,) array of RMS error per state
            - 'overall': Overall RMS error across all states
    """
    # Placeholder implementation - will be fully implemented in Part 5
    raise NotImplementedError("RMS error computation will be implemented in Part 5")

