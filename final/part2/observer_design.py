import numpy as np
import matplotlib.pyplot as plt
import os
import sys
from scipy.signal import place_poles

# Add workspace root to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

from final.utils.model import build_continuous_model, discretize_zoh
from final.utils.simulation import simulate_observer


def get_part2_C():
    """Part 2 sensor matrix measuring x1 and x6."""
    return np.array([
        [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
    ])


def get_part2_initial_conditions():
    """Part 2 initial conditions: actual state and observer initial state."""
    x0 = np.array([0., 0., 0., 1., 1., 1., 0., 0., 0., 0., 0., 0.])
    xhat0 = np.array([0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0.])
    return x0, xhat0


def design_observer_gain(Ad, Cd, pole_range=(0.4, 0.8)):
    """Design observer gain L using pole placement via dual system."""
    n = Ad.shape[0]
    desired_poles = np.linspace(pole_range[0], pole_range[1], n)
    
    # Dual system: observer design is dual to controller design
    Ad_dual = Ad.T
    Cd_dual = Cd.T
    
    # Place poles for dual system
    result = place_poles(Ad_dual, Cd_dual, desired_poles, method='YT', rtol=1e-2)
    L_dual = result.gain_matrix
    
    # Transpose to get observer gain
    L = L_dual.T
    
    # Verify stability
    A_L = Ad - L @ Cd
    observer_poles = np.linalg.eigvals(A_L)
    spectral_radius = np.max(np.abs(observer_poles))
    
    return L, {'spectral_radius': spectral_radius, 'poles': observer_poles}


def main():
    """Design observer and simulate plant-observer system."""
    Ts = 0.01
    N = 1000
    
    # Build model
    A, B, C = build_continuous_model()
    Ad, Bd, Cd, Dd = discretize_zoh(A, B, C, Ts)
    
    # Part 2 C matrix and initial conditions
    Cd_new = get_part2_C()
    x0, xhat0 = get_part2_initial_conditions()
    
    # Design observer
    L, design_info = design_observer_gain(Ad, Cd_new)
    print(f"Observer gain L shape: {L.shape}")
    print(f"Spectral radius: {design_info['spectral_radius']:.6f}")
    
    # Simulate
    u = np.zeros((Bd.shape[1], N))
    results = simulate_observer(Ad, Bd, Cd_new, L, x0, xhat0, u, N, Ts)
    
    # Compute RMS errors for displacements
    e = results['e']
    rms_displacements = np.sqrt(np.mean(e[:6, :]**2, axis=1))
    rms_overall = np.sqrt(np.mean(np.sum(e[:6, :]**2, axis=0)))
    
    # Create outputs directory
    output_dir = os.path.join(os.path.dirname(__file__), 'outputs')
    os.makedirs(output_dir, exist_ok=True)
    
    # Plot outputs comparison
    fig1, ax1 = plt.subplots(2, 1, figsize=(10, 8))
    ax1[0].plot(results['t'], results['y'][0, :], 'b-', label='y1 (true)', linewidth=2)
    ax1[0].plot(results['t'], results['yhat'][0, :], 'r--', label='yhat1 (estimated)', linewidth=2)
    ax1[0].set_xlabel('Time (s)')
    ax1[0].set_ylabel('Displacement')
    ax1[0].set_title('Output 1: Displacement of Mass 1 (x1)')
    ax1[0].legend()
    ax1[0].grid(True)
    
    ax1[1].plot(results['t'], results['y'][1, :], 'b-', label='y6 (true)', linewidth=2)
    ax1[1].plot(results['t'], results['yhat'][1, :], 'r--', label='yhat6 (estimated)', linewidth=2)
    ax1[1].set_xlabel('Time (s)')
    ax1[1].set_ylabel('Displacement')
    ax1[1].set_title('Output 2: Displacement of Mass 6 (x6)')
    ax1[1].legend()
    ax1[1].grid(True)
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'outputs_comparison.png'), dpi=150)
    plt.close()
    
    # Plot estimation errors for displacements (full 10-second window)
    fig2, ax2 = plt.subplots(figsize=(10, 6))
    labels = [f'e{i+1}' for i in range(6)]
    colors = plt.cm.tab10(np.linspace(0, 1, 6))
    
    for i in range(6):
        ax2.plot(results['t'], e[i, :], label=labels[i], color=colors[i], linewidth=1.5)
    
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Estimation Error')
    ax2.set_title('State Estimation Errors: Displacements (x1..x6)')
    ax2.legend()
    ax2.grid(True)
    plt.savefig(os.path.join(output_dir, 'estimation_errors.png'), dpi=150)
    plt.close()
    
    # Plot estimation errors for displacements (zoomed 0.5-second window)
    idx_05sec = int(0.5 / Ts) + 1  # Include t=0 to t=0.5
    fig3, ax3 = plt.subplots(figsize=(10, 6))
    
    for i in range(6):
        ax3.plot(results['t'][:idx_05sec], e[i, :idx_05sec], label=labels[i], color=colors[i], linewidth=1.5)
    
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Estimation Error')
    ax3.set_title('State Estimation Errors: Displacements (x1..x6) - First 0.5 Second')
    ax3.legend()
    ax3.grid(True)
    plt.savefig(os.path.join(output_dir, 'estimation_errors_05sec.png'), dpi=150)
    plt.close()
    
    # Save results
    with open(os.path.join(output_dir, 'results.txt'), 'w') as f:
        f.write("Part 2: Observer Simulation Results\n")
        f.write("="*60 + "\n\n")
        f.write(f"Measurement Matrix (Cd_new):\n{Cd_new}\n\n")
        f.write(f"Initial Conditions:\n")
        f.write(f"  x0 = {x0}\n")
        f.write(f"  xhat0 = {xhat0}\n\n")
        f.write(f"Observer Design:\n")
        f.write(f"  Observer gain L shape: {L.shape}\n")
        f.write(f"  Spectral radius: {design_info['spectral_radius']:.6f}\n\n")
        f.write(f"RMS Estimation Errors (displacements x1-x6):\n")
        for i in range(6):
            f.write(f"  x{i+1}: {rms_displacements[i]:.6e}\n")
        f.write(f"  Overall RMS: {rms_overall:.6e}\n")
    
    print(f"Results saved to {output_dir}/")


if __name__ == '__main__':
    main()
