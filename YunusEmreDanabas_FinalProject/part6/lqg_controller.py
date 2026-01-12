"""Part 6: LQG Controller (LQR + Kalman Filter)"""

import numpy as np
import matplotlib.pyplot as plt
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

from final.utils.model import build_continuous_model, discretize_zoh
from final.utils.simulation import simulate_lqg
from final.part2.observer_design import get_part2_C, get_part2_initial_conditions


def compute_lqg_cost(u, y_true, N):
    """Compute LQG cost: J = sum(u[k]'u[k] + y1[k]^2 + y6[k]^2) using y_true."""
    J = 0.0
    for k in range(N):
        J += u[:, k].T @ u[:, k] + y_true[0, k]**2 + y_true[1, k]**2
    return J


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
    
    # Load K from Part 3
    part3_dir = os.path.join(os.path.dirname(__file__), '../part3/outputs')
    K_path = os.path.join(part3_dir, 'K_matrix.npy')
    K = np.load(K_path)
    print(f"Loaded K from Part 3: shape {K.shape}")
    
    # Load Lk from Part 5
    part5_dir = os.path.join(os.path.dirname(__file__), '../part5/outputs')
    Lk_path = os.path.join(part5_dir, 'Lk_matrix.npy')
    Lk = np.load(Lk_path)
    print(f"Loaded Lk from Part 5: shape {Lk.shape}")
    
    # Noise covariances (Part 5 frozen)
    m = Bd.shape[1]
    p = Cmeas.shape[0]
    Qw = 0.05 * np.eye(m)
    Rv = 0.1 * np.eye(p)
    
    # Simulate LQG
    results = simulate_lqg(Ad, Bd, Cmeas, K, Lk, x0, xhat0, N, Ts, Qw, Rv, seed=seed)
    
    # Compute cost
    J = compute_lqg_cost(results['u'], results['y_true'], N)
    max_u = np.max(np.abs(results['u']))
    
    # Estimation error metrics
    e = results['x'] - results['xhat']
    error_norm = np.array([np.linalg.norm(e[:, k]) for k in range(N + 1)])
    rms_error = np.sqrt(np.mean(error_norm**2))
    
    print(f"Total cost J: {J:.6e}")
    print(f"Max |u|: {max_u:.6e}")
    print(f"RMS estimation error: {rms_error:.6e}")
    
    # Create outputs directory
    output_dir = os.path.join(os.path.dirname(__file__), 'outputs')
    os.makedirs(output_dir, exist_ok=True)
    
    # Load Part 3 data for comparison if available
    part3_traj_path = os.path.join(part3_dir, 'traj.npz')
    part3_data = None
    if os.path.exists(part3_traj_path):
        part3_data = np.load(part3_traj_path)
    
    # Plot outputs comparison
    fig1, ax1 = plt.subplots(2, 1, figsize=(10, 8))
    if part3_data is not None:
        ax1[0].plot(part3_data['t'], part3_data['y'][0, :], 'b-', 
                    label='Part 3 (no noise)', linewidth=2, alpha=0.7)
    ax1[0].plot(results['t'], results['y_true'][0, :], 'g-', 
                label='Part 6 y_true', linewidth=2, alpha=0.7)
    ax1[0].plot(results['t'], results['yhat'][0, :], 'r--', 
                label='Part 6 yhat', linewidth=2)
    ax1[0].set_xlabel('Time (s)')
    ax1[0].set_ylabel('Displacement')
    ax1[0].set_title('Output 1: Displacement of Mass 1 (x1)')
    ax1[0].legend()
    ax1[0].grid(True)
    
    if part3_data is not None:
        ax1[1].plot(part3_data['t'], part3_data['y'][1, :], 'b-', 
                    label='Part 3 (no noise)', linewidth=2, alpha=0.7)
    ax1[1].plot(results['t'], results['y_true'][1, :], 'g-', 
                label='Part 6 y_true', linewidth=2, alpha=0.7)
    ax1[1].plot(results['t'], results['yhat'][1, :], 'r--', 
                label='Part 6 yhat', linewidth=2)
    ax1[1].set_xlabel('Time (s)')
    ax1[1].set_ylabel('Displacement')
    ax1[1].set_title('Output 2: Displacement of Mass 6 (x6)')
    ax1[1].legend()
    ax1[1].grid(True)
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'outputs_comparison.png'), dpi=150)
    plt.close()
    
    # Plot inputs
    t_inputs = results['t'][:N]
    fig2, ax2 = plt.subplots(3, 1, figsize=(10, 8))
    for i in range(3):
        ax2[i].plot(t_inputs, results['u'][i, :], 'r-', label=f'u{i+1}', linewidth=2)
        ax2[i].set_xlabel('Time (s)')
        ax2[i].set_ylabel('Input')
        ax2[i].set_title(f'Input {i+1}')
        ax2[i].legend()
        ax2[i].grid(True)
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'inputs_u1_u2_u3.png'), dpi=150)
    plt.close()
    
    # Plot estimation error norm
    fig3, ax3 = plt.subplots(figsize=(10, 6))
    ax3.plot(results['t'], error_norm, 'g-', label='||x - xhat||', linewidth=2)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Estimation Error Norm')
    ax3.set_title('Estimation Error Norm: ||x - xhat||')
    ax3.legend()
    ax3.grid(True)
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'estimation_error_norm.png'), dpi=150)
    plt.close()
    
    # Save results
    with open(os.path.join(output_dir, 'results.txt'), 'w') as f:
        f.write("Part 6: LQG Controller (LQR + Kalman Filter) - Results\n")
        f.write("="*60 + "\n\n")
        f.write(f"LQG Controller:\n")
        f.write(f"  K shape: {K.shape}\n")
        f.write(f"  Lk shape: {Lk.shape}\n\n")
        f.write(f"Cost Metrics:\n")
        f.write(f"  Total cost J: {J:.6e}\n")
        f.write(f"  Max |u|: {max_u:.6e}\n\n")
        f.write(f"Estimation Metrics:\n")
        f.write(f"  RMS estimation error: {rms_error:.6e}\n")
    
    # Save matrices and trajectories
    np.save(os.path.join(output_dir, 'K_matrix.npy'), K)
    np.save(os.path.join(output_dir, 'Lk_matrix.npy'), Lk)
    e_array = e
    np.savez(os.path.join(output_dir, 'traj.npz'),
             t=results['t'], x=results['x'], xhat=results['xhat'],
             u=results['u'], y=results['y_true'], e=e_array,
             y_true=results['y_true'], y_meas=results['y_meas'], yhat=results['yhat'])
    
    print(f"\nResults saved to {output_dir}/")


if __name__ == '__main__':
    main()
