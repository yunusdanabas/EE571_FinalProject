"""Part 5: Kalman Filter Design"""

import numpy as np
import matplotlib.pyplot as plt
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

from final.utils.model import build_continuous_model, discretize_zoh
from final.utils.simulation import simulate_kalman_noisy
from final.part2.observer_design import get_part2_C, get_part2_initial_conditions
from scipy.linalg import solve_discrete_are


def design_kalman_filter(Ad, Cmeas, Qx, Rv):
    """Design steady-state Kalman filter using DARE."""
    P = solve_discrete_are(Ad.T, Cmeas.T, Qx, Rv)
    S = Cmeas @ P @ Cmeas.T + Rv
    Lk = P @ Cmeas.T @ np.linalg.inv(S)
    
    Aest = Ad - Lk @ Cmeas
    spectral_radius = np.max(np.abs(np.linalg.eigvals(Aest)))
    
    return Lk, P, spectral_radius


def compute_rms_metrics(x, xhat, y_true, y_meas, yhat, steady_state_window=0.2):
    """Compute RMS estimation error metrics (full window and steady-state)."""
    n, N_plus_1 = x.shape
    N = N_plus_1 - 1
    ss_start_index = int((N + 1) * (1 - steady_state_window))
    
    e = x - xhat
    error_norm = np.array([np.linalg.norm(e[:, k]) for k in range(N + 1)])
    
    rms_overall = np.sqrt(np.mean(error_norm**2))
    rms_overall_ss = np.sqrt(np.mean(error_norm[ss_start_index:]**2))
    rms_per_state = np.sqrt(np.mean(e**2, axis=1))
    rms_per_state_ss = np.sqrt(np.mean(e[:, ss_start_index:]**2, axis=1))
    
    output_error_true = y_true - yhat
    rms_output_y1 = np.sqrt(np.mean(output_error_true[0, :]**2))
    rms_output_y6 = np.sqrt(np.mean(output_error_true[1, :]**2))
    rms_output_y1_ss = np.sqrt(np.mean(output_error_true[0, ss_start_index:]**2))
    rms_output_y6_ss = np.sqrt(np.mean(output_error_true[1, ss_start_index:]**2))
    
    return {
        'rms_overall': rms_overall,
        'rms_overall_ss': rms_overall_ss,
        'rms_per_state': rms_per_state,
        'rms_per_state_ss': rms_per_state_ss,
        'rms_e_x1': rms_per_state[0],
        'rms_e_x6': rms_per_state[5],
        'rms_e_x1_ss': rms_per_state_ss[0],
        'rms_e_x6_ss': rms_per_state_ss[5],
        'rms_output_y1': rms_output_y1,
        'rms_output_y6': rms_output_y6,
        'rms_output_y1_ss': rms_output_y1_ss,
        'rms_output_y6_ss': rms_output_y6_ss,
        'error_norm': error_norm
    }


def main():
    Ts = 0.01
    N = 1000
    seed = 42
    
    # Build model
    A, B, C = build_continuous_model()
    Ad, Bd, Cd, Dd = discretize_zoh(A, B, C, Ts)
    
    # Part 2 dependencies
    Cmeas = get_part2_C()
    x0, xhat0 = get_part2_initial_conditions()
    
    # Noise covariances
    m = Bd.shape[1]
    p = Cmeas.shape[0]
    Qw = 0.05 * np.eye(m)
    Rv = 0.1 * np.eye(p)
    Qx = Bd @ Qw @ Bd.T
    
    # Design Kalman filter
    Lk, P, spectral_radius = design_kalman_filter(Ad, Cmeas, Qx, Rv)
    print(f"Kalman gain Lk shape: {Lk.shape}")
    print(f"Estimator spectral radius: {spectral_radius:.6f}")
    
    # Simulate
    results = simulate_kalman_noisy(Ad, Bd, Cmeas, Lk, x0, xhat0, N, Ts, Qw, Rv, seed=seed)
    
    # Compute metrics
    rms_metrics = compute_rms_metrics(
        results['x'], results['xhat'], results['y_true'], results['y_meas'], results['yhat']
    )
    print(f"Overall RMS estimation error: {rms_metrics['rms_overall']:.6e}")
    print(f"RMS error e_x1: {rms_metrics['rms_e_x1']:.6e}")
    print(f"RMS error e_x6: {rms_metrics['rms_e_x6']:.6e}")
    
    # Create outputs directory
    output_dir = os.path.join(os.path.dirname(__file__), 'outputs')
    os.makedirs(output_dir, exist_ok=True)
    
    # Plot outputs: True vs Estimated
    fig1, ax1 = plt.subplots(2, 1, figsize=(10, 8))
    ax1[0].plot(results['t'], results['y_true'][0, :], 'g-', label='y_true', linewidth=2, alpha=0.8)
    ax1[0].plot(results['t'], results['yhat'][0, :], 'r--', label='yhat', linewidth=2)
    ax1[0].plot(results['t'], results['y_meas'][0, :], 'b-', label='y_meas', linewidth=1, alpha=0.3)
    ax1[0].set_xlabel('Time (s)')
    ax1[0].set_ylabel('Displacement')
    ax1[0].set_title('Output 1: Displacement of Mass 1 (x1) - True vs Estimated')
    ax1[0].legend()
    ax1[0].grid(True)
    
    ax1[1].plot(results['t'], results['y_true'][1, :], 'g-', label='y_true', linewidth=2, alpha=0.8)
    ax1[1].plot(results['t'], results['yhat'][1, :], 'r--', label='yhat', linewidth=2)
    ax1[1].plot(results['t'], results['y_meas'][1, :], 'b-', label='y_meas', linewidth=1, alpha=0.3)
    ax1[1].set_xlabel('Time (s)')
    ax1[1].set_ylabel('Displacement')
    ax1[1].set_title('Output 2: Displacement of Mass 6 (x6) - True vs Estimated')
    ax1[1].legend()
    ax1[1].grid(True)
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'outputs_y_vs_yhat.png'), dpi=150)
    plt.close()
    
    # Plot estimation error norm
    fig2, ax2 = plt.subplots(figsize=(10, 6))
    ax2.plot(results['t'], rms_metrics['error_norm'], 'g-', label='||x - xhat||', linewidth=2)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Estimation Error Norm')
    ax2.set_title('Estimation Error Norm: ||x - xhat||')
    ax2.legend()
    ax2.grid(True)
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'estimation_error_norm.png'), dpi=150)
    plt.close()
    
    # Plot estimation errors for x1 and x6
    e = results['x'] - results['xhat']
    fig3, ax3 = plt.subplots(2, 1, figsize=(10, 8))
    ax3[0].plot(results['t'], e[0, :], 'b-', label='e_x1', linewidth=2)
    ax3[0].set_xlabel('Time (s)')
    ax3[0].set_ylabel('Estimation Error')
    ax3[0].set_title('Estimation Error: x1 (Displacement of Mass 1)')
    ax3[0].legend()
    ax3[0].grid(True)
    
    ax3[1].plot(results['t'], e[5, :], 'b-', label='e_x6', linewidth=2)
    ax3[1].set_xlabel('Time (s)')
    ax3[1].set_ylabel('Estimation Error')
    ax3[1].set_title('Estimation Error: x6 (Displacement of Mass 6)')
    ax3[1].legend()
    ax3[1].grid(True)
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'estimation_error_x1_x6.png'), dpi=150)
    plt.close()
    
    # Plot per-state RMS bar chart
    fig4, ax4 = plt.subplots(figsize=(10, 6))
    state_labels = [f'x{i+1}' for i in range(12)]
    colors = plt.cm.tab10(np.linspace(0, 1, 12))
    bars = ax4.bar(state_labels, rms_metrics['rms_per_state'], color=colors)
    ax4.set_xlabel('State')
    ax4.set_ylabel('RMS Estimation Error')
    ax4.set_title('Per-State RMS Estimation Error')
    ax4.set_yscale('log')
    ax4.grid(True, axis='y', alpha=0.3)
    plt.xticks(rotation=45)
    plt.tight_layout()
    
    plt.savefig(os.path.join(output_dir, 'per_state_rms_bar.png'), dpi=150)
    plt.close()
    
    # Save results
    with open(os.path.join(output_dir, 'results.txt'), 'w') as f:
        f.write("Part 5: Kalman Filter Design - Results\n")
        f.write("="*60 + "\n\n")
        f.write(f"Kalman Filter Design:\n")
        f.write(f"  Lk shape: {Lk.shape}\n")
        f.write(f"  Estimator spectral radius: {spectral_radius:.6f}\n\n")
        f.write(f"RMS Metrics:\n")
        f.write(f"  Overall RMS estimation error (full): {rms_metrics['rms_overall']:.6e}\n")
        f.write(f"  Overall RMS estimation error (steady-state): {rms_metrics['rms_overall_ss']:.6e}\n")
        f.write(f"  RMS error e_x1 (full): {rms_metrics['rms_e_x1']:.6e}\n")
        f.write(f"  RMS error e_x1 (steady-state): {rms_metrics['rms_e_x1_ss']:.6e}\n")
        f.write(f"  RMS error e_x6 (full): {rms_metrics['rms_e_x6']:.6e}\n")
        f.write(f"  RMS error e_x6 (steady-state): {rms_metrics['rms_e_x6_ss']:.6e}\n")
        f.write(f"  RMS output tracking y1 (full): {rms_metrics['rms_output_y1']:.6e}\n")
        f.write(f"  RMS output tracking y1 (steady-state): {rms_metrics['rms_output_y1_ss']:.6e}\n")
        f.write(f"  RMS output tracking y6 (full): {rms_metrics['rms_output_y6']:.6e}\n")
        f.write(f"  RMS output tracking y6 (steady-state): {rms_metrics['rms_output_y6_ss']:.6e}\n")
        f.write(f"\n  Per-state RMS (full window):\n")
        for i in range(12):
            f.write(f"    x{i+1}: {rms_metrics['rms_per_state'][i]:.6e}\n")
    
    # Save matrices and trajectories
    np.save(os.path.join(output_dir, 'Lk_matrix.npy'), Lk)
    np.savez(os.path.join(output_dir, 'traj.npz'),
             x=results['x'], xhat=results['xhat'],
             y_true=results['y_true'], y_meas=results['y_meas'], yhat=results['yhat'],
             innovations=results['innovations'], t=results['t'])
    
    print(f"\nResults saved to {output_dir}/")


if __name__ == '__main__':
    main()
