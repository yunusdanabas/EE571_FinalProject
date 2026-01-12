"""Part 7: Sensor Augmentation Analysis"""

import numpy as np
import matplotlib.pyplot as plt
import os
import sys
from scipy.linalg import solve_discrete_are

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

from final.utils.model import build_continuous_model, discretize_zoh
from final.utils.simulation import simulate_lqg_augmented
from final.part2.observer_design import get_part2_C, get_part2_initial_conditions


def get_C_case1():
    """Case 1: 4 sensors measuring x1, x2, x5, x6."""
    C = np.zeros((4, 12))
    C[0, 0] = 1  # x1
    C[1, 1] = 1  # x2
    C[2, 4] = 1  # x5
    C[3, 5] = 1  # x6
    return C


def get_C_case2():
    """Case 2: 6 sensors measuring all positions x1-x6."""
    C = np.zeros((6, 12))
    for i in range(6):
        C[i, i] = 1
    return C


def get_cost_output_selector():
    """Cost output selector: extracts x1 and x6 for cost computation."""
    Cy = np.zeros((2, 12))
    Cy[0, 0] = 1  # x1
    Cy[1, 5] = 1  # x6
    return Cy


def design_kalman_filter(Ad, Bd, Cmeas, Qw, Rv):
    """Design steady-state Kalman filter using DARE."""
    Qx = Bd @ Qw @ Bd.T
    P = solve_discrete_are(Ad.T, Cmeas.T, Qx, Rv)
    S = Cmeas @ P @ Cmeas.T + Rv
    Lk = P @ Cmeas.T @ np.linalg.inv(S)
    Aest = Ad - Lk @ Cmeas
    rho_est = np.max(np.abs(np.linalg.eigvals(Aest)))
    return Lk, rho_est


def compute_metrics(x, xhat, u, y_cost, N):
    """Compute cost and RMS metrics."""
    J = 0.0
    for k in range(N):
        J += u[:, k].T @ u[:, k] + y_cost[0, k]**2 + y_cost[1, k]**2
    
    e = x - xhat
    error_norm = np.array([np.linalg.norm(e[:, k]) for k in range(N + 1)])
    ss_start = int((N + 1) * 0.8)
    
    rms_error_overall = np.sqrt(np.mean(error_norm**2))
    rms_error_ss = np.sqrt(np.mean(error_norm[ss_start:]**2))
    rms_per_state_ss = np.sqrt(np.mean(e[:, ss_start:]**2, axis=1))
    max_u = np.max(np.abs(u))
    
    return {
        'J': J,
        'rms_error_overall': rms_error_overall,
        'rms_error_ss': rms_error_ss,
        'rms_per_state_ss': rms_per_state_ss,
        'error_norm': error_norm,
        'max_u': max_u
    }


def load_part6_baseline():
    """Load Part 6 baseline for comparison."""
    baseline = {}
    part6_dir = os.path.join(os.path.dirname(__file__), '../part6/outputs')
    traj_file = os.path.join(part6_dir, 'traj.npz')
    results_file = os.path.join(part6_dir, 'results.txt')
    
    if os.path.exists(traj_file):
        traj = np.load(traj_file)
        baseline['x'] = traj['x']
        baseline['xhat'] = traj['xhat']
        baseline['u'] = traj['u']
        baseline['t'] = traj['t']
        baseline['max_u'] = np.max(np.abs(traj['u']))
    
    if os.path.exists(results_file):
        import re
        with open(results_file, 'r') as f:
            content = f.read()
        
        match = re.search(r'Total cost J[^:]*:\s*([0-9.]+[eE][+-]?[0-9]+)', content)
        if match:
            baseline['J'] = float(match.group(1))
        
        match = re.search(r'RMS estimation error[^:]*:\s*([0-9.]+[eE][+-]?[0-9]+)', content)
        if match:
            baseline['rms_error'] = float(match.group(1))
    
    return baseline


def main():
    Ts = 0.01
    N = 1000
    seed = 42
    
    # Build model
    A, B, C = build_continuous_model()
    Ad, Bd, Cd, Dd = discretize_zoh(A, B, C, Ts)
    
    # Initial conditions
    x0, xhat0 = get_part2_initial_conditions()
    
    # Cost output selector
    Cy = get_cost_output_selector()
    
    # Load K from Part 3
    part3_dir = os.path.join(os.path.dirname(__file__), '../part3/outputs')
    K_path = os.path.join(part3_dir, 'K_matrix.npy')
    K = np.load(K_path)
    
    # Noise covariances
    m = Bd.shape[1]
    Qw = 0.05 * np.eye(m)
    
    # Load Part 6 baseline
    part6_baseline = load_part6_baseline()
    
    # Case 1: 4 sensors
    C_case1 = get_C_case1()
    p_case1 = C_case1.shape[0]
    Rv_case1 = 0.1 * np.eye(p_case1)
    Lk_case1, rho_case1 = design_kalman_filter(Ad, Bd, C_case1, Qw, Rv_case1)
    
    results_case1 = simulate_lqg_augmented(
        Ad, Bd, C_case1, Cy, K, Lk_case1, x0, xhat0, N, Ts, Qw, Rv_case1, seed
    )
    metrics_case1 = compute_metrics(
        results_case1['x'], results_case1['xhat'], results_case1['u'],
        results_case1['y_cost'], N
    )
    
    # Case 2: 6 sensors
    C_case2 = get_C_case2()
    p_case2 = C_case2.shape[0]
    Rv_case2 = 0.1 * np.eye(p_case2)
    Lk_case2, rho_case2 = design_kalman_filter(Ad, Bd, C_case2, Qw, Rv_case2)
    
    results_case2 = simulate_lqg_augmented(
        Ad, Bd, C_case2, Cy, K, Lk_case2, x0, xhat0, N, Ts, Qw, Rv_case2, seed
    )
    metrics_case2 = compute_metrics(
        results_case2['x'], results_case2['xhat'], results_case2['u'],
        results_case2['y_cost'], N
    )
    
    # Print comparison
    print(f"\nComparison:")
    print(f"  Part 6 (2 sensors): J={part6_baseline.get('J', 'N/A'):.4e}, "
          f"RMS={part6_baseline.get('rms_error', 'N/A'):.4e}")
    print(f"  Case 1 (4 sensors): J={metrics_case1['J']:.4e}, "
          f"RMS={metrics_case1['rms_error_ss']:.4e}, rho={rho_case1:.6f}")
    print(f"  Case 2 (6 sensors): J={metrics_case2['J']:.4e}, "
          f"RMS={metrics_case2['rms_error_ss']:.4e}, rho={rho_case2:.6f}")
    
    # Create output directory
    output_dir = os.path.join(os.path.dirname(__file__), 'outputs')
    os.makedirs(output_dir, exist_ok=True)
    
    # Plot estimation error comparison
    fig1, ax1 = plt.subplots(figsize=(10, 6))
    ax1.plot(results_case1['t'], metrics_case1['error_norm'], 'b-',
             label='Case 1 (4 sensors)', linewidth=2, alpha=0.8)
    ax1.plot(results_case2['t'], metrics_case2['error_norm'], 'g-',
             label='Case 2 (6 sensors)', linewidth=2, alpha=0.8)
    if 'x' in part6_baseline and 'xhat' in part6_baseline:
        e_part6 = part6_baseline['x'] - part6_baseline['xhat']
        error_norm_part6 = np.array([np.linalg.norm(e_part6[:, k]) for k in range(e_part6.shape[1])])
        ax1.plot(part6_baseline['t'], error_norm_part6, 'r--',
                 label='Part 6 (2 sensors)', linewidth=2, alpha=0.8)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Estimation Error Norm ||x - xhat||')
    ax1.set_title('Estimation Error Comparison')
    ax1.legend()
    ax1.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'estimation_error_comparison.png'), dpi=150)
    plt.close()
    
    # Plot outputs comparison
    fig2, ax2 = plt.subplots(2, 1, figsize=(10, 8))
    ax2[0].plot(results_case1['t'], results_case1['y_cost'][0, :], 'b-',
                label='Case 1', linewidth=2, alpha=0.8)
    ax2[0].plot(results_case2['t'], results_case2['y_cost'][0, :], 'g-',
                label='Case 2', linewidth=2, alpha=0.8)
    ax2[0].set_xlabel('Time (s)')
    ax2[0].set_ylabel('Displacement')
    ax2[0].set_title('Output 1 (x1) Comparison')
    ax2[0].legend()
    ax2[0].grid(True)
    
    ax2[1].plot(results_case1['t'], results_case1['y_cost'][1, :], 'b-',
                label='Case 1', linewidth=2, alpha=0.8)
    ax2[1].plot(results_case2['t'], results_case2['y_cost'][1, :], 'g-',
                label='Case 2', linewidth=2, alpha=0.8)
    ax2[1].set_xlabel('Time (s)')
    ax2[1].set_ylabel('Displacement')
    ax2[1].set_title('Output 2 (x6) Comparison')
    ax2[1].legend()
    ax2[1].grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'outputs_comparison.png'), dpi=150)
    plt.close()
    
    # Plot inputs comparison
    fig3, ax3 = plt.subplots(3, 1, figsize=(10, 8))
    t_inputs = results_case1['t'][:N]
    for i in range(3):
        ax3[i].plot(t_inputs, results_case1['u'][i, :], 'b-',
                    label='Case 1', linewidth=2, alpha=0.8)
        ax3[i].plot(t_inputs, results_case2['u'][i, :], 'g-',
                    label='Case 2', linewidth=2, alpha=0.8)
        ax3[i].set_xlabel('Time (s)')
        ax3[i].set_ylabel(f'Input u{i+1}')
        ax3[i].set_title(f'Input {i+1} Comparison')
        ax3[i].legend()
        ax3[i].grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'inputs_comparison.png'), dpi=150)
    plt.close()
    
    # Plot per-state RMS comparison
    fig4, ax4 = plt.subplots(figsize=(12, 6))
    states = ['x1', 'x2', 'x3', 'x4', 'x5', 'x6', 'v1', 'v2', 'v3', 'v4', 'v5', 'v6']
    x_pos = np.arange(len(states))
    width = 0.25
    ax4.bar(x_pos - width, metrics_case1['rms_per_state_ss'], width,
            label='Case 1 (4 sensors)', color='blue', alpha=0.7)
    ax4.bar(x_pos, metrics_case2['rms_per_state_ss'], width,
            label='Case 2 (6 sensors)', color='green', alpha=0.7)
    ax4.set_xlabel('State')
    ax4.set_ylabel('RMS Estimation Error (Steady-State)')
    ax4.set_title('Per-State RMS Estimation Error (Last 20%)')
    ax4.set_xticks(x_pos)
    ax4.set_xticklabels(states)
    ax4.legend()
    ax4.grid(True, axis='y')
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'per_state_rms_comparison.png'), dpi=150)
    plt.close()
    
    # Save results
    with open(os.path.join(output_dir, 'results.txt'), 'w') as f:
        f.write("Part 7: Sensor Augmentation Analysis - Results\n")
        f.write("="*60 + "\n\n")
        f.write("Case 1 (4 sensors: x1, x2, x5, x6):\n")
        f.write(f"  Lk shape: {Lk_case1.shape}\n")
        f.write(f"  Estimator spectral radius: {rho_case1:.6f}\n")
        f.write(f"  J: {metrics_case1['J']:.6e}\n")
        f.write(f"  RMS error (SS): {metrics_case1['rms_error_ss']:.6e}\n")
        f.write(f"  max|u|: {metrics_case1['max_u']:.6e}\n\n")
        f.write("Case 2 (6 sensors: x1-x6):\n")
        f.write(f"  Lk shape: {Lk_case2.shape}\n")
        f.write(f"  Estimator spectral radius: {rho_case2:.6f}\n")
        f.write(f"  J: {metrics_case2['J']:.6e}\n")
        f.write(f"  RMS error (SS): {metrics_case2['rms_error_ss']:.6e}\n")
        f.write(f"  max|u|: {metrics_case2['max_u']:.6e}\n\n")
        f.write("Comparison:\n")
        if 'J' in part6_baseline:
            f.write(f"  Part 6 J: {part6_baseline['J']:.6e}\n")
        f.write(f"  Case 1 J: {metrics_case1['J']:.6e}\n")
        f.write(f"  Case 2 J: {metrics_case2['J']:.6e}\n")
    
    # Save matrices and trajectories
    np.save(os.path.join(output_dir, 'Lk_case1_matrix.npy'), Lk_case1)
    np.save(os.path.join(output_dir, 'Lk_case2_matrix.npy'), Lk_case2)
    np.savez(os.path.join(output_dir, 'traj_case1.npz'),
             t=results_case1['t'], x=results_case1['x'], xhat=results_case1['xhat'],
             u=results_case1['u'], y_cost=results_case1['y_cost'],
             y_true=results_case1['y_true'], y_meas=results_case1['y_meas'],
             yhat=results_case1['yhat'])
    np.savez(os.path.join(output_dir, 'traj_case2.npz'),
             t=results_case2['t'], x=results_case2['x'], xhat=results_case2['xhat'],
             u=results_case2['u'], y_cost=results_case2['y_cost'],
             y_true=results_case2['y_true'], y_meas=results_case2['y_meas'],
             yhat=results_case2['yhat'])
    
    print(f"\nResults saved to {output_dir}/")


if __name__ == '__main__':
    main()