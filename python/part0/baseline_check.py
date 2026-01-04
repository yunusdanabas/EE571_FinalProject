"""
Baseline verification script for Part 0.

This script:
1. Loads continuous-time model from prep_final.m
2. Discretizes using ZOH at Ts=0.01
3. Simulates open-loop with zero input
4. Validates dimensions and performs sanity checks
5. Generates plots consistent with actual output dimensions
"""

import numpy as np
import sys
import os

# Add parent directory to path to import utils
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from utils.build_model import build_continuous_model, discretize_zoh
from utils.sim import simulate_discrete
from utils.metrics import check_dimensions
from utils.plots import plot_outputs, plot_displacements
import matplotlib.pyplot as plt


def main():
    """Main baseline verification routine."""
    print("=" * 60)
    print("Part 0: Baseline Verification")
    print("=" * 60)
    
    # Parameters from prep_final.m
    Ts = 0.01  # Sampling time
    N = 1000   # Number of simulation steps
    
    # Initial condition from prep_final.m
    x0 = np.array([0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0])
    
    print("\n1. Building continuous-time model...")
    A, B, C = build_continuous_model()
    print(f"   A shape: {A.shape}")
    print(f"   B shape: {B.shape}")
    print(f"   C shape: {C.shape}")
    
    print("\n2. Discretizing using ZOH...")
    Ad, Bd, Cd, Dd = discretize_zoh(A, B, C, Ts)
    print(f"   Ad shape: {Ad.shape}")
    print(f"   Bd shape: {Bd.shape}")
    print(f"   Cd shape: {Cd.shape}")
    print(f"   Dd shape: {Dd.shape} (should be all zeros)")
    
    print("\n3. Validating dimensions...")
    dim_check = check_dimensions(Ad, Bd, Cd, expected_n=12, expected_m=3, expected_p=1)
    
    print(f"   Number of states (n): {dim_check['n']}")
    print(f"   Number of inputs (m): {dim_check['m']}")
    print(f"   Number of outputs (p): {dim_check['p']}")
    print(f"   Ad valid: {dim_check['Ad_valid']}")
    print(f"   Bd valid: {dim_check['Bd_valid']}")
    print(f"   Cd valid: {dim_check['Cd_valid']}")
    print(f"   All dimensions valid: {dim_check['all_valid']}")
    
    if not dim_check['all_valid']:
        print("\n   ERROR: Dimension validation failed!")
        return
    
    print("\n4. Setting up simulation...")
    u = np.zeros((3, N))  # Zero input for open-loop simulation
    print(f"   Initial condition: x0 = {x0}")
    print(f"   Input shape: {u.shape} (zero input)")
    print(f"   Number of steps: {N}")
    print(f"   Sampling time: {Ts} s")
    
    print("\n5. Running simulation...")
    x, y, t = simulate_discrete(Ad, Bd, Cd, x0, u, N, Ts=Ts)
    print(f"   State trajectory shape: {x.shape}")
    print(f"   Output trajectory shape: {y.shape}")
    print(f"   Time vector shape: {t.shape}")
    
    print("\n6. Performing sanity checks...")
    # Check for NaN and Inf
    has_nan = np.any(np.isnan(x)) or np.any(np.isnan(y))
    has_inf = np.any(np.isinf(x)) or np.any(np.isinf(y))
    
    print(f"   Contains NaN: {has_nan}")
    print(f"   Contains Inf: {has_inf}")
    
    if has_nan or has_inf:
        print("\n   WARNING: Simulation contains NaN or Inf values!")
    else:
        print("   States and outputs are bounded (no NaN/Inf)")
    
    # Check output magnitude
    y_max = np.max(np.abs(y))
    print(f"   Maximum output magnitude: {y_max:.6f}")
    
    # Check state magnitude
    x_max = np.max(np.abs(x))
    print(f"   Maximum state magnitude: {x_max:.6f}")
    
    if x_max > 1000 or y_max > 1000:
        print("\n   WARNING: Very large state/output values detected!")
    else:
        print("   State and output magnitudes are reasonable")
    
    print("\n7. Generating plots...")
    
    # Plot outputs (dimension-aware: should show only 1 trace since C is 1×12)
    fig1, ax1 = plot_outputs(t, y, labels=['y1'], 
                             title='System Output (displacement of mass 1)')
    plt.savefig('python/part0/output_plot.png', dpi=150, bbox_inches='tight')
    print("   Saved: python/part0/output_plot.png (shows 1 output matching C matrix)")
    
    # Plot all 6 displacements for visualization (using states, not outputs)
    fig2, ax2 = plot_displacements(t, x, title='All Mass Displacements (for visualization)')
    plt.savefig('python/part0/displacements_plot.png', dpi=150, bbox_inches='tight')
    print("   Saved: python/part0/displacements_plot.png (shows all 6 displacements)")
    
    # Show plots (optional - comment out if running in non-interactive mode)
    # plt.show()
    
    print("\n" + "=" * 60)
    print("Baseline verification complete!")
    print("=" * 60)
    print("\nNote: The output plot shows only 1 trace (matching C matrix),")
    print("      while the displacements plot shows all 6 positions for visualization.")
    print("      This addresses the legend mismatch issue in prep_final.m")


if __name__ == '__main__':
    main()

