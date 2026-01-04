"""
Part 2: Coupled plant-observer simulation.

This script runs a nominal (no-noise) simulation comparing true states x
vs estimated states xhat using the Part 2 initial conditions.
"""

import numpy as np
import matplotlib.pyplot as plt
import os
import sys

# Add paths for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'utils'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from build_model import build_continuous_model, discretize_zoh
from plots import plot_outputs, plot_states
from observer_design import get_part2_C_matrix, design_observer


def get_part2_initial_conditions():
    """
    Get Part 2 initial conditions.
    
    Source: Verified from final exam Question 2 (docs/sources/final_exam.pdf)
    The exam explicitly provides these initial conditions:
        - Actual state x0: [0; 0; 0; 1; 1; 1; 0; 0; 0; 0; 0; 0]
        - Observer initial state xhat0: [0; 0; 0; 0; 0; 1; 0; 0; 0; 0; 0; 0]
    
    Returns:
        tuple: (x0, xhat0) where both are (12,) arrays
    """
    # Actual initial state: x0 = [0; 0; 0; 1; 1; 1; 0; 0; 0; 0; 0; 0]
    x0 = np.array([0., 0., 0., 1., 1., 1., 0., 0., 0., 0., 0., 0.])
    
    # Observer initial state: xhat0 = [0; 0; 0; 0; 0; 1; 0; 0; 0; 0; 0; 0]
    xhat0 = np.array([0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0.])
    
    return x0, xhat0


def simulate_plant_observer(Ad, Bd, Cd_new, L, x0, xhat0, u, N, Ts):
    """
    Simulate coupled plant-observer system.
    
    Plant:  x[k+1] = Ad @ x[k] + Bd @ u[k]
            y[k] = Cd_new @ x[k]
    
    Observer: xhat[k+1] = Ad @ xhat[k] + Bd @ u[k] + L @ (y[k] - yhat[k])
              yhat[k] = Cd_new @ xhat[k]
    
    Args:
        Ad: (n, n) discrete-time state matrix
        Bd: (n, m) discrete-time input matrix
        Cd_new: (p, n) output matrix
        L: (n, p) observer gain
        x0: (n,) initial true state
        xhat0: (n,) initial observer state
        u: (m, N) input sequence
        N: Number of time steps
        Ts: Sampling time (for time vector)
    
    Returns:
        dict: Trajectories and errors
            - 'x': (n, N) true state trajectory
            - 'xhat': (n, N) estimated state trajectory
            - 'y': (p, N) true output trajectory
            - 'yhat': (p, N) estimated output trajectory
            - 'e': (n, N) estimation error (x - xhat)
            - 't': (N,) time vector
    """
    n = Ad.shape[0]
    p = Cd_new.shape[0]
    
    # Preallocate arrays
    x = np.zeros((n, N))
    xhat = np.zeros((n, N))
    y = np.zeros((p, N))
    yhat = np.zeros((p, N))
    e = np.zeros((n, N))
    
    # Initialize
    x[:, 0] = x0
    xhat[:, 0] = xhat0
    y[:, 0] = Cd_new @ x[:, 0]
    yhat[:, 0] = Cd_new @ xhat[:, 0]
    e[:, 0] = x[:, 0] - xhat[:, 0]
    
    # Simulate forward
    for k in range(N - 1):
        # Plant output
        y[:, k] = Cd_new @ x[:, k]
        
        # Observer output
        yhat[:, k] = Cd_new @ xhat[:, k]
        
        # Observer correction term
        y_error = y[:, k] - yhat[:, k]
        
        # Plant update
        x[:, k + 1] = Ad @ x[:, k] + Bd @ u[:, k]
        
        # Observer update
        xhat[:, k + 1] = Ad @ xhat[:, k] + Bd @ u[:, k] + L @ y_error
        
        # Estimation error
        e[:, k + 1] = x[:, k + 1] - xhat[:, k + 1]
    
    # Final outputs
    y[:, N - 1] = Cd_new @ x[:, N - 1]
    yhat[:, N - 1] = Cd_new @ xhat[:, N - 1]
    
    # Time vector
    t = np.arange(N) * Ts
    
    return {
        'x': x,
        'xhat': xhat,
        'y': y,
        'yhat': yhat,
        'e': e,
        't': t
    }


def compute_rms_errors(e, state_indices=None, steady_state_window=0.2):
    """
    Compute RMS estimation errors (full window and steady-state window).
    
    Args:
        e: (n, N) estimation error matrix
        state_indices: Optional list of state indices to compute RMS for.
            If None, computes for all states.
        steady_state_window: Fraction of samples to use for steady-state RMS (default 0.2 = last 20%)
    
    Returns:
        dict: RMS error metrics
            - 'per_state': (len(state_indices),) array of RMS errors (full window)
            - 'per_state_ss': (len(state_indices),) array of RMS errors (steady-state window)
            - 'overall': Overall RMS error (full window)
            - 'overall_ss': Overall RMS error (steady-state window)
            - 'state_indices': Which states were used
            - 'ss_start_index': Starting index for steady-state window
    """
    if state_indices is None:
        state_indices = list(range(e.shape[0]))
    
    n_states = len(state_indices)
    N = e.shape[1]
    
    # Steady-state window: last steady_state_window fraction of samples
    ss_start_index = int(N * (1 - steady_state_window))
    
    # Compute RMS per state (full window): sqrt(mean(e_i^2))
    rms_per_state = np.zeros(n_states)
    for i, idx in enumerate(state_indices):
        rms_per_state[i] = np.sqrt(np.mean(e[idx, :]**2))
    
    # Compute RMS per state (steady-state window)
    rms_per_state_ss = np.zeros(n_states)
    for i, idx in enumerate(state_indices):
        rms_per_state_ss[i] = np.sqrt(np.mean(e[idx, ss_start_index:]**2))
    
    # Compute overall RMS (full window): sqrt(mean(sum(e_i^2)))
    selected_errors = e[state_indices, :]
    overall_rms = np.sqrt(np.mean(np.sum(selected_errors**2, axis=0)))
    
    # Compute overall RMS (steady-state window)
    selected_errors_ss = e[state_indices, ss_start_index:]
    overall_rms_ss = np.sqrt(np.mean(np.sum(selected_errors_ss**2, axis=0)))
    
    return {
        'per_state': rms_per_state,
        'per_state_ss': rms_per_state_ss,
        'overall': overall_rms,
        'overall_ss': overall_rms_ss,
        'state_indices': state_indices,
        'ss_start_index': ss_start_index,
        'ss_fraction': steady_state_window
    }


if __name__ == '__main__':
    """
    Main runner: Load model, design observer, run simulation, generate plots and metrics.
    """
    print("="*60)
    print("Part 2: Observer Simulation")
    print("="*60)
    
    # Load model
    print("\n1. Loading model from Part 0 utilities...")
    A, B, C = build_continuous_model()
    Ad, Bd, Cd, Dd = discretize_zoh(A, B, C, Ts=0.01)
    Ts = 0.01
    
    # Get Part 2 C matrix
    print("2. Using Part 2 sensor matrix (measuring x1 and x6)...")
    Cd_new = get_part2_C_matrix()
    print(f"   Cd_new shape: {Cd_new.shape}")
    print(f"   Cd_new = \n{Cd_new}")
    
    # Get initial conditions
    print("3. Loading Part 2 initial conditions...")
    x0, xhat0 = get_part2_initial_conditions()
    print(f"   x0 = {x0}")
    print(f"   xhat0 = {xhat0}")
    
    # Design observer
    print("\n4. Designing observer...")
    try:
        # Use improved pole placement with automatic fallback to dual LQR
        # Design policy: 12 distinct real poles evenly spaced in [0.4, 0.8]
        # If pole placement fails, automatically falls back to dual LQR (more robust)
        L, design_info = design_observer(
            Ad, Cd_new, 
            method='pole_placement',
            pole_range=(0.4, 0.8),
            fallback_to_lqr=True
        )
        print(f"   Observer gain L shape: {design_info['L_shape']}")
        print(f"   Design policy: {design_info.get('design_policy', 'N/A')}")
        print(f"   Spectral radius: {design_info['spectral_radius']:.6f}")
        print(f"   Observer stable: {design_info['is_stable']}")
        
        # Critical validation: spectral radius must be < 1.0
        if design_info['spectral_radius'] >= 1.0:
            print(f"   ERROR: Observer validation FAILED - spectral radius >= 1.0")
            print(f"   Observer is UNSTABLE and will not converge.")
            sys.exit(1)
        
        if design_info['spectral_radius'] >= 0.99:
            print(f"   WARNING: Spectral radius is close to 1.0 - observer may have slow convergence.")
    except RuntimeError as e:
        print(f"   ERROR: Observer design failed: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"   ERROR: Unexpected error in observer design: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    
    # Simulation parameters
    print("\n5. Setting up simulation...")
    N = 1000  # Default: 1000 steps (10 seconds at Ts=0.01)
    # Input signal: zero input (default if not specified in exam)
    u = np.zeros((Bd.shape[1], N))
    print(f"   Simulation horizon: N = {N} steps ({N * Ts:.1f} seconds)")
    print(f"   Input signal: u[k] = 0 (zero input, open-loop)")
    
    # Run simulation
    print("\n6. Running coupled plant-observer simulation...")
    results = simulate_plant_observer(Ad, Bd, Cd_new, L, x0, xhat0, u, N, Ts)
    
    print(f"   Simulation complete.")
    print(f"   Final estimation error (displacements): {results['e'][:6, -1]}")
    
    # Compute RMS errors
    print("\n7. Computing RMS estimation errors...")
    # RMS for displacement states (x1..x6)
    rms_displacements = compute_rms_errors(results['e'], state_indices=list(range(6)), steady_state_window=0.2)
    print(f"   RMS errors for displacements (x1..x6) - Full window:")
    for i, idx in enumerate(rms_displacements['state_indices']):
        print(f"     x{idx+1}: {rms_displacements['per_state'][i]:.6e}")
    print(f"   Overall RMS (displacements, full): {rms_displacements['overall']:.6e}")
    
    print(f"\n   RMS errors for displacements (x1..x6) - Steady-state window (last {rms_displacements['ss_fraction']*100:.0f}%):")
    for i, idx in enumerate(rms_displacements['state_indices']):
        print(f"     x{idx+1}: {rms_displacements['per_state_ss'][i]:.6e}")
    print(f"   Overall RMS (displacements, steady-state): {rms_displacements['overall_ss']:.6e}")
    
    # RMS for all states
    rms_all = compute_rms_errors(results['e'], state_indices=None, steady_state_window=0.2)
    print(f"\n   Overall RMS (all states, full): {rms_all['overall']:.6e}")
    print(f"   Overall RMS (all states, steady-state): {rms_all['overall_ss']:.6e}")
    
    # Create output directory
    output_dir = os.path.join(os.path.dirname(__file__), 'outputs')
    os.makedirs(output_dir, exist_ok=True)
    
    # Generate plots
    print("\n8. Generating plots...")
    
    # Plot 1: Measured outputs and estimates
    fig1, ax1 = plt.subplots(2, 1, figsize=(10, 8))
    
    # y1 and yhat1
    ax1[0].plot(results['t'], results['y'][0, :], 'b-', label='y1 (true)', linewidth=2)
    ax1[0].plot(results['t'], results['yhat'][0, :], 'r--', label='yhat1 (estimated)', linewidth=2)
    ax1[0].set_xlabel('Time (s)')
    ax1[0].set_ylabel('Displacement')
    ax1[0].set_title('Output 1: Displacement of Mass 1 (x1)')
    ax1[0].legend()
    ax1[0].grid(True)
    
    # y6 and yhat6
    ax1[1].plot(results['t'], results['y'][1, :], 'b-', label='y6 (true)', linewidth=2)
    ax1[1].plot(results['t'], results['yhat'][1, :], 'r--', label='yhat6 (estimated)', linewidth=2)
    ax1[1].set_xlabel('Time (s)')
    ax1[1].set_ylabel('Displacement')
    ax1[1].set_title('Output 2: Displacement of Mass 6 (x6)')
    ax1[1].legend()
    ax1[1].grid(True)
    
    plt.tight_layout()
    outputs_file = os.path.join(output_dir, 'outputs_comparison.png')
    plt.savefig(outputs_file, dpi=150)
    print(f"   Saved: {outputs_file}")
    plt.close()
    
    # Plot 2: Estimation errors for displacements (x1..x6)
    fig2, ax2 = plt.subplots(figsize=(10, 6))
    labels = ['e1', 'e2', 'e3', 'e4', 'e5', 'e6']
    colors = plt.cm.tab10(np.linspace(0, 1, 6))
    
    for i in range(6):
        ax2.plot(results['t'], results['e'][i, :], label=labels[i], 
                color=colors[i], linewidth=1.5)
    
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Estimation Error')
    ax2.set_title('State Estimation Errors: Displacements (x1..x6)')
    ax2.legend()
    ax2.grid(True)
    
    errors_file = os.path.join(output_dir, 'estimation_errors.png')
    plt.savefig(errors_file, dpi=150)
    print(f"   Saved: {errors_file}")
    plt.close()
    
    # Plot 3: All state errors (optional, for reference)
    fig3, ax3 = plt.subplots(figsize=(12, 8))
    
    # Plot displacement errors (x1..x6)
    for i in range(6):
        ax3.plot(results['t'], results['e'][i, :], label=f'e{i+1} (x{i+1})', 
                linewidth=1.5, alpha=0.8)
    
    # Plot velocity errors (x1dot..x6dot)
    for i in range(6, 12):
        ax3.plot(results['t'], results['e'][i, :], '--', 
                label=f'e{i+1} (x{i-5}dot)', linewidth=1, alpha=0.6)
    
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Estimation Error')
    ax3.set_title('State Estimation Errors: All States')
    ax3.legend(ncol=2, fontsize=8)
    ax3.grid(True, alpha=0.3)
    
    all_errors_file = os.path.join(output_dir, 'all_state_errors.png')
    plt.savefig(all_errors_file, dpi=150)
    print(f"   Saved: {all_errors_file}")
    plt.close()
    
    # Save results to text file
    print("\n9. Saving results to file...")
    results_file = os.path.join(output_dir, 'results.txt')
    with open(results_file, 'w') as f:
        f.write("Part 2: Observer Simulation Results\n")
        f.write("="*60 + "\n\n")
        
        f.write("Measurement Matrix:\n")
        f.write(f"  Cd_new shape: {Cd_new.shape}\n")
        f.write(f"  Cd_new = \n{Cd_new}\n\n")
        
        f.write("Initial Conditions:\n")
        f.write(f"  x0 (actual) = {x0}\n")
        f.write(f"  xhat0 (observer) = {xhat0}\n\n")
        
        f.write("Simulation Parameters:\n")
        f.write(f"  N (simulation steps) = {N}\n")
        f.write(f"  Ts (sampling time) = {Ts} s\n")
        f.write(f"  Time span = {N * Ts:.1f} s\n")
        f.write(f"  Input signal: u[k] = 0 (zero input, open-loop)\n\n")
        
        f.write("Observer Design:\n")
        f.write(f"  Method: {design_info['method']}\n")
        if design_info['method'] == 'pole_placement_dual':
            f.write(f"  Placement method: {design_info.get('placement_method', 'N/A')}\n")
            f.write(f"  Design policy: {design_info.get('design_policy', 'N/A')}\n")
            f.write(f"  Balancing used: {design_info.get('balancing_used', False)}\n")
            if design_info.get('balancing_used', False):
                f.write(f"  Condition number (before): {design_info.get('cond_number_before', 'N/A'):.2e}\n")
                f.write(f"  Condition number (after): {design_info.get('cond_number_after', 'N/A'):.2e}\n")
            if 'desired_poles' in design_info:
                f.write(f"  Requested poles max magnitude: {design_info['max_desired_pole']:.6f}\n")
        elif design_info['method'] == 'dual_lqr':
            f.write(f"  Alpha used: {design_info.get('alpha_used', 'N/A')}\n")
            f.write(f"  Alpha sweep: {design_info.get('alpha_sweep', 'N/A')}\n")
        f.write(f"  Observer gain L shape: {design_info['L_shape']}\n")
        f.write(f"  Spectral radius (max(abs(eig(Ad - L@Cd_new)))): {design_info['spectral_radius']:.6f}\n")
        f.write(f"  Observer stable: {design_info['is_stable']}\n\n")
        
        f.write("RMS Estimation Errors:\n")
        f.write("  Note: Full-window RMS includes transient errors from initial condition mismatch.\n")
        f.write("        Steady-state RMS (last 20% window) reflects convergence performance.\n")
        f.write("        Both reported for Part 3 comparison completeness.\n\n")
        f.write("  Displacements (x1..x6) - Full window:\n")
        for i, idx in enumerate(rms_displacements['state_indices']):
            f.write(f"    x{idx+1}: {rms_displacements['per_state'][i]:.6e}\n")
        f.write(f"  Overall RMS (displacements, full): {rms_displacements['overall']:.6e}\n")
        f.write(f"\n  Displacements (x1..x6) - Steady-state window (last {rms_displacements['ss_fraction']*100:.0f}%, no-noise, float64):\n")
        for i, idx in enumerate(rms_displacements['state_indices']):
            f.write(f"    x{idx+1}: {rms_displacements['per_state_ss'][i]:.6e}\n")
        f.write(f"  Overall RMS (displacements, steady-state): {rms_displacements['overall_ss']:.6e}\n")
        f.write(f"\n  Overall RMS (all states, full): {rms_all['overall']:.6e}\n")
        f.write(f"  Overall RMS (all states, steady-state): {rms_all['overall_ss']:.6e}\n\n")
        
        f.write("Convergence Analysis:\n")
        initial_error_norm = np.linalg.norm(results['e'][:, 0])
        final_error_norm = np.linalg.norm(results['e'][:, -1])
        f.write(f"  Initial error norm: {initial_error_norm:.6e}\n")
        f.write(f"  Final error norm: {final_error_norm:.6e}\n")
        if final_error_norm < initial_error_norm:
            reduction = (1 - final_error_norm / initial_error_norm) * 100
            f.write(f"  Error reduction: {reduction:.2f}%\n")
        else:
            f.write(f"  WARNING: Error increased (observer may be unstable or converging slowly)\n")
        
        f.write("\nFinal Estimation Errors (displacements):\n")
        for i in range(6):
            f.write(f"  e{i+1} (x{i+1}): {results['e'][i, -1]:.6e}\n")
    
    print(f"   Saved: {results_file}")
    
    print("\n" + "="*60)
    print("Part 2 simulation complete!")
    print("="*60)
    print(f"\nOutput files saved to: {output_dir}/")
    print("  - outputs_comparison.png")
    print("  - estimation_errors.png")
    print("  - all_state_errors.png")
    print("  - results.txt")

