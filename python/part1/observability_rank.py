"""
Observability matrix construction and rank analysis.

This module provides functions to construct the observability matrix for a
discrete-time system and compute its rank using SVD with a documented tolerance policy.
"""

import numpy as np


def construct_observability_matrix(Ad, Cd):
    """
    Construct the observability matrix for a discrete-time system.
    
    For system (Ad, Cd), the observability matrix is:
        O = [Cd; Cd*Ad; Cd*Ad^2; ...; Cd*Ad^(n-1)]
    
    Args:
        Ad: (n, n) discrete-time state matrix
        Cd: (p, n) discrete-time output matrix
    
    Returns:
        O: (n*p, n) observability matrix
    """
    n = Ad.shape[0]
    p = Cd.shape[0]
    
    # Preallocate observability matrix
    # For p outputs, we have n*p rows and n columns
    O = np.zeros((n * p, n))
    
    # Build O row by row
    # First block: Cd
    O[0:p, :] = Cd
    
    # Subsequent blocks: Cd * Ad^k for k = 1, 2, ..., n-1
    Ad_power = np.eye(n)  # Start with identity
    for i in range(1, n):
        Ad_power = Ad_power @ Ad
        row_start = i * p
        row_end = row_start + p
        O[row_start:row_end, :] = Cd @ Ad_power
    
    return O


def compute_observability_rank(O, tol=None):
    """
    Compute the rank of the observability matrix using SVD.
    
    Uses SVD to compute numerical rank with a tolerance policy:
    - If tol is None, uses: tol = max(1e-10, machine_epsilon * max(sigma))
    - Otherwise uses the provided tolerance
    
    Args:
        O: (m, n) observability matrix
        tol: Optional tolerance for rank computation. If None, uses automatic policy.
    
    Returns:
        tuple: (rank, tol_used) where
            - rank: Numerical rank of O
            - tol_used: Tolerance actually used for rank computation
    """
    # Compute SVD
    U, sigma, Vh = np.linalg.svd(O, full_matrices=False)
    
    # Determine tolerance
    if tol is None:
        # Automatic tolerance: max(1e-10, machine_epsilon * max(singular_value))
        machine_eps = np.finfo(O.dtype).eps
        max_sigma = np.max(sigma)
        tol_used = max(1e-10, machine_eps * max_sigma)
    else:
        tol_used = tol
    
    # Count singular values above threshold
    # Rank = number of singular values > tol_used * max(sigma)
    max_sigma = np.max(sigma)
    if max_sigma > 0:
        threshold = tol_used * max_sigma
        rank = np.sum(sigma > threshold)
    else:
        # All singular values are zero (or very close)
        rank = 0
    
    return rank, tol_used


def analyze_observability(Ad, Cd, tol=None):
    """
    Complete observability analysis for a discrete-time system.
    
    Constructs the observability matrix, computes its rank, and returns
    comprehensive results including dimensions.
    
    Args:
        Ad: (n, n) discrete-time state matrix
        Cd: (p, n) discrete-time output matrix
        tol: Optional tolerance for rank computation. If None, uses automatic policy.
    
    Returns:
        dict: Dictionary containing:
            - 'O': Observability matrix (n*p, n)
            - 'rank': Rank of observability matrix
            - 'tol_used': Tolerance used for rank computation
            - 'n': System dimension (number of states)
            - 'p': Number of outputs
            - 'dim_observable': Dimension of observable subspace (equals rank)
            - 'dim_unobservable': Dimension of unobservable subspace (n - rank)
            - 'is_observable': Boolean, True if system is observable (rank == n)
    """
    # Construct observability matrix
    O = construct_observability_matrix(Ad, Cd)
    
    # Compute rank
    rank, tol_used = compute_observability_rank(O, tol=tol)
    
    # Get dimensions
    n = Ad.shape[0]
    p = Cd.shape[0]
    
    # Compute subspace dimensions
    dim_observable = rank
    dim_unobservable = n - rank
    
    # Check if system is observable
    is_observable = (rank == n)
    
    return {
        'O': O,
        'rank': rank,
        'tol_used': tol_used,
        'n': n,
        'p': p,
        'dim_observable': dim_observable,
        'dim_unobservable': dim_unobservable,
        'is_observable': is_observable
    }

