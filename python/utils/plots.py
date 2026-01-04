"""
Plotting utilities for system outputs and states.

This module provides dimension-aware plotting functions that ensure legends
match the actual number of outputs defined by the C matrix.
"""

import numpy as np
import matplotlib.pyplot as plt


def plot_outputs(t, y, labels=None, title='Outputs', xlabel='Time (s)', ylabel=None, grid=True):
    """
    Plot system outputs with dimension-aware legend.
    
    CRITICAL PLOT POLICY: This function plots ONLY y = Cx (actual outputs).
    It ensures the number of legend entries exactly matches the number of
    outputs (size(C, 1)). This fixes the inconsistency in prep_final.m where
    the legend shows 6 outputs but C defines 1 output.
    
    For visualization of internal states (e.g., all 6 displacements), use
    plot_states() or plot_displacements() instead - these are NOT outputs.
    
    Args:
        t: (N,) time vector
        y: (p, N) output matrix where p is the number of outputs
        labels: list of p strings for legend entries. If None, auto-generates ['y1', 'y2', ..., 'yp']
        title: Plot title
        xlabel: X-axis label
        ylabel: Y-axis label. If None, uses 'Output' or 'Outputs' based on p
        grid: Whether to show grid
    
    Returns:
        fig, ax: matplotlib figure and axes objects
    """
    y = np.atleast_2d(y)
    p, N = y.shape
    
    # Auto-generate labels if not provided
    if labels is None:
        if p == 1:
            labels = ['y1']
        else:
            labels = [f'y{i+1}' for i in range(p)]
    else:
        if len(labels) != p:
            raise ValueError(f"Number of labels ({len(labels)}) must match number of outputs ({p})")
    
    # Default ylabel
    if ylabel is None:
        ylabel = 'Output' if p == 1 else 'Outputs'
    
    # Create plot
    fig, ax = plt.subplots(figsize=(10, 6))
    
    # Plot each output
    for i in range(p):
        ax.plot(t, y[i, :], label=labels[i])
    
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend()
    if grid:
        ax.grid(True)
    
    return fig, ax


def plot_states(t, x, state_indices=None, labels=None, title='States', 
                xlabel='Time (s)', ylabel=None, grid=True):
    """
    Plot state trajectories (for visualization purposes).
    
    This function is for plotting state variables directly, regardless of
    what the C matrix selects. For example, it can plot all 6 displacements
    (x1 through x6) even if C only measures one output.
    
    Args:
        t: (N,) time vector
        x: (n, N) state matrix
        state_indices: list of state indices to plot (0-indexed). If None, plots all states
        labels: list of strings for legend entries. If None, auto-generates based on indices
        title: Plot title
        xlabel: X-axis label
        ylabel: Y-axis label. If None, uses 'State' or 'States'
        grid: Whether to show grid
    
    Returns:
        fig, ax: matplotlib figure and axes objects
    """
    x = np.atleast_2d(x)
    n, N = x.shape
    
    # Determine which states to plot
    if state_indices is None:
        state_indices = list(range(n))
    
    num_plots = len(state_indices)
    
    # Auto-generate labels if not provided
    if labels is None:
        labels = []
        for idx in state_indices:
            if idx < 6:
                labels.append(f'x{idx+1}')  # Positions: x1, x2, ..., x6
            else:
                labels.append(f'x{idx-5}dot')  # Velocities: x1dot, x2dot, ..., x6dot
    else:
        if len(labels) != num_plots:
            raise ValueError(f"Number of labels ({len(labels)}) must match number of states to plot ({num_plots})")
    
    # Default ylabel
    if ylabel is None:
        ylabel = 'State' if num_plots == 1 else 'States'
    
    # Create plot
    fig, ax = plt.subplots(figsize=(10, 6))
    
    # Plot each selected state
    for i, idx in enumerate(state_indices):
        if idx < 0 or idx >= n:
            raise ValueError(f"State index {idx} out of range [0, {n-1}]")
        ax.plot(t, x[idx, :], label=labels[i])
    
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend()
    if grid:
        ax.grid(True)
    
    return fig, ax


def plot_displacements(t, x, labels=None, title='Displacements', grid=True):
    """
    Convenience function to plot all 6 position states (displacements).
    
    IMPORTANT: This plots INTERNAL STATE VISUALIZATION, not outputs.
    This is useful for visualizing all mass displacements regardless of
    what the C matrix measures. Do NOT confuse this with system outputs.
    
    Args:
        t: (N,) time vector
        x: (n, N) state matrix (must have n >= 6)
        labels: list of 6 strings for legend entries. If None, uses ['x1', 'x2', ..., 'x6']
        title: Plot title
        grid: Whether to show grid
    
    Returns:
        fig, ax: matplotlib figure and axes objects
    """
    if labels is None:
        labels = ['x1', 'x2', 'x3', 'x4', 'x5', 'x6']
    
    return plot_states(t, x, state_indices=list(range(6)), labels=labels,
                      title=title, ylabel='Displacement', grid=grid)

