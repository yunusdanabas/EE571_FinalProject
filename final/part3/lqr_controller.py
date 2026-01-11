"""Part 3: LQR Controller Design with Observer"""

import numpy as np
import matplotlib.pyplot as plt
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

from final.utils.model import build_continuous_model, discretize_zoh
from final.utils.simulation import simulate_lqr_with_observer
from final.part2.observer_design import get_part2_C, get_part2_initial_conditions, design_observer_gain
from scipy.linalg import solve_discrete_are


def design_lqr(Ad, Bd, Q, R):
    """Design discrete-time LQR controller using DARE."""
    P = solve_discrete_are(Ad, Bd, Q, R)
    R_BPB = R + Bd.T @ P @ Bd
    K = np.linalg.solve(R_BPB, Bd.T @ P @ Ad)
    
    Acl = Ad - Bd @ K
    spectral_radius = np.max(np.abs(np.linalg.eigvals(Acl)))
    
    return K, P, spectral_radius


def compute_cost(x, u, Cy, N):
    """Compute LQR cost: J = sum(u[k]'u[k] + y1[k]^2 + y6[k]^2)."""
    J = 0.0
    for k in range(N):
        y_cost = Cy @ x[:, k]
        J += u[:, k].T @ u[:, k] + y_cost[0]**2 + y_cost[1]**2
    return J


def main():
    Ts = 0.01
    N = 1000
    
    # Build model
    A, B, C = build_continuous_model()
    Ad, Bd, Cd, Dd = discretize_zoh(A, B, C, Ts)
    
    # Part 2 dependencies
    Cy = get_part2_C()
    x0, xhat0 = get_part2_initial_conditions()
    L, obs_info = design_observer_gain(Ad, Cy)
    
    # Cost matrices
    Q = Cy.T @ Cy
    R = np.eye(3)
    
    # Design LQR
    K, P, rho = design_lqr(Ad, Bd, Q, R)
    print(f"LQR gain K shape: {K.shape}")
    print(f"Closed-loop spectral radius: {rho:.6f}")
    
    # Simulate closed-loop
    results = simulate_lqr_with_observer(Ad, Bd, Cy, K, L, x0, xhat0, N, Ts)
    
    # Compute cost
    J = compute_cost(results['x'], results['u'], Cy, N)
    max_u = np.max(np.abs(results['u']))
    print(f"Total cost J: {J:.6e}")
    print(f"Max |u|: {max_u:.6e}")
    
    # Create outputs directory
    output_dir = os.path.join(os.path.dirname(__file__), 'outputs')
    os.makedirs(output_dir, exist_ok=True)
    
    # Plot outputs
    fig1, ax1 = plt.subplots(2, 1, figsize=(10, 8))
    ax1[0].plot(results['t'], results['y'][0, :], 'b-', label='y1 (x1)', linewidth=2)
    ax1[0].set_xlabel('Time (s)')
    ax1[0].set_ylabel('Displacement')
    ax1[0].set_title('Output 1: Displacement of Mass 1 (x1)')
    ax1[0].legend()
    ax1[0].grid(True)
    
    ax1[1].plot(results['t'], results['y'][1, :], 'b-', label='y6 (x6)', linewidth=2)
    ax1[1].set_xlabel('Time (s)')
    ax1[1].set_ylabel('Displacement')
    ax1[1].set_title('Output 2: Displacement of Mass 6 (x6)')
    ax1[1].legend()
    ax1[1].grid(True)
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'outputs_y1_y6.png'), dpi=150)
    plt.close()
    
    # Plot inputs (full window)
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
    
    # Plot inputs (0.5-second zoom)
    idx_05sec_inputs = int(0.5 / Ts)
    fig2_zoom, ax2_zoom = plt.subplots(3, 1, figsize=(10, 8))
    for i in range(3):
        ax2_zoom[i].plot(t_inputs[:idx_05sec_inputs], results['u'][i, :idx_05sec_inputs], 'r-', label=f'u{i+1}', linewidth=2)
        ax2_zoom[i].set_xlabel('Time (s)')
        ax2_zoom[i].set_ylabel('Input')
        ax2_zoom[i].set_title(f'Input {i+1} - First 0.5 Second')
        ax2_zoom[i].legend()
        ax2_zoom[i].grid(True)
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'inputs_u1_u2_u3_05sec.png'), dpi=150)
    plt.close()
    
    # Plot estimation error norm (full window)
    error_norm = np.array([np.linalg.norm(results['e'][:, k]) for k in range(N + 1)])
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
    
    # Plot estimation error norm (0.5-second zoom)
    idx_05sec = int(0.5 / Ts) + 1
    fig3_zoom, ax3_zoom = plt.subplots(figsize=(10, 6))
    ax3_zoom.plot(results['t'][:idx_05sec], error_norm[:idx_05sec], 'g-', label='||x - xhat||', linewidth=2)
    ax3_zoom.set_xlabel('Time (s)')
    ax3_zoom.set_ylabel('Estimation Error Norm')
    ax3_zoom.set_title('Estimation Error Norm: ||x - xhat|| - First 0.5 Second')
    ax3_zoom.legend()
    ax3_zoom.grid(True)
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'estimation_error_norm_05sec.png'), dpi=150)
    plt.close()
    
    # Save results
    with open(os.path.join(output_dir, 'results.txt'), 'w') as f:
        f.write("Part 3: LQR Controller Design with Observer - Results\n")
        f.write("="*60 + "\n\n")
        f.write(f"LQR Design:\n")
        f.write(f"  K shape: {K.shape}\n")
        f.write(f"  Closed-loop spectral radius: {rho:.6f}\n")
        f.write(f"Cost Metrics:\n")
        f.write(f"  Total cost J: {J:.6e}\n")
        f.write(f"  Max |u|: {max_u:.6e}\n")
        f.write(f"Observer:\n")
        f.write(f"  Spectral radius: {obs_info['spectral_radius']:.6f}\n")
    
    # Save matrices for later parts
    np.save(os.path.join(output_dir, 'K_matrix.npy'), K)
    np.save(os.path.join(output_dir, 'L_matrix.npy'), L)
    np.savez(os.path.join(output_dir, 'traj.npz'),
             t=results['t'], x=results['x'], xhat=results['xhat'],
             u=results['u'], y=results['y'], e=results['e'])
    
    print(f"\nResults saved to {output_dir}/")


if __name__ == '__main__':
    main()
