"""
Discrete-time simulation utilities.

This module provides functions for simulating discrete-time linear systems.
"""

import numpy as np


def simulate_discrete(Ad, Bd, Cd, x0, u, N, Ts=None):
    """
    Simulate a discrete-time linear system.
    
    System equations:
        x[k+1] = Ad @ x[k] + Bd @ u[k]
        y[k] = Cd @ x[k]
    
    Args:
        Ad: (n, n) discrete-time state matrix
        Bd: (n, m) discrete-time input matrix
        Cd: (p, n) discrete-time output matrix
        x0: (n,) initial state vector
        u: (m, N) input sequence (columns are time steps)
        N: Number of simulation steps
        Ts: Sampling time (optional, for time vector generation)
    
    Returns:
        tuple: (x, y, t) where
            - x: (n, N) state trajectory
            - y: (p, N) output trajectory
            - t: (N,) time vector (if Ts provided, else 0:N-1)
    """
    n = Ad.shape[0]
    p = Cd.shape[0]
    
    # Preallocate arrays
    x = np.zeros((n, N))
    y = np.zeros((p, N))
    
    # Initialize state
    x[:, 0] = x0
    
    # Compute outputs at initial time
    y[:, 0] = Cd @ x[:, 0]
    
    # Simulate forward
    for k in range(N - 1):
        # Update state: x[k+1] = Ad @ x[k] + Bd @ u[k]
        x[:, k + 1] = Ad @ x[:, k] + Bd @ u[:, k]
        
        # Compute output: y[k] = Cd @ x[k]
        y[:, k] = Cd @ x[:, k]
    
    # Final output at time N-1
    y[:, N - 1] = Cd @ x[:, N - 1]
    
    # Generate time vector
    if Ts is not None:
        t = np.arange(N) * Ts
    else:
        t = np.arange(N)
    
    return x, y, t

