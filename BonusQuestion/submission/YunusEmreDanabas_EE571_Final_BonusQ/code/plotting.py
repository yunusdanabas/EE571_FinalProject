"""
plotting.py - Visualization and metrics functions for vehicle tracking results
"""

import numpy as np
import matplotlib.pyplot as plt


# =============================================================================
# Plotting Functions
# =============================================================================

def plot_trajectory_comparison(results_lqr, results_pp, scale, title_suffix=""):
    """Plot vehicle trajectory comparison for LQR and PP."""
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.plot(results_lqr['Xref'], results_lqr['Yref'], 'k-', linewidth=1.8, label='Reference', alpha=0.7)
    ax.plot(results_lqr['x'][:, 0], results_lqr['x'][:, 1], 'b-', linewidth=1.6, label='LQR')
    ax.plot(results_pp['x'][:, 0], results_pp['x'][:, 1], 'r--', linewidth=1.6, label='Pole Placement')
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_title(f'Trajectory Comparison (scale={scale}x){title_suffix}')
    ax.legend(loc='best')
    ax.axis('equal')
    ax.grid(True)
    return fig


def plot_errors_comparison(results_lqr, results_pp, scale, title_suffix=""):
    """Plot tracking errors comparison for LQR and PP."""
    fig, axes = plt.subplots(3, 1, figsize=(10, 8))
    
    axes[0].plot(results_lqr['t'], results_lqr['ey_log'], 'b-', linewidth=1.4, label='LQR')
    axes[0].plot(results_pp['t'], results_pp['ey_log'], 'r--', linewidth=1.4, label='Pole Placement')
    axes[0].set_ylabel('e_y [m]')
    axes[0].set_title('Cross-track error')
    axes[0].legend()
    axes[0].grid(True)
    
    axes[1].plot(results_lqr['t'], np.rad2deg(results_lqr['epsi_log']), 'b-', linewidth=1.4, label='LQR')
    axes[1].plot(results_pp['t'], np.rad2deg(results_pp['epsi_log']), 'r--', linewidth=1.4, label='Pole Placement')
    axes[1].set_ylabel('e_psi [deg]')
    axes[1].set_title('Heading error')
    axes[1].legend()
    axes[1].grid(True)
    
    axes[2].plot(results_lqr['t'], results_lqr['ev_log'], 'b-', linewidth=1.4, label='LQR')
    axes[2].plot(results_pp['t'], results_pp['ev_log'], 'r--', linewidth=1.4, label='Pole Placement')
    axes[2].set_xlabel('t [s]')
    axes[2].set_ylabel('e_v [m/s]')
    axes[2].set_title('Speed error')
    axes[2].legend()
    axes[2].grid(True)
    
    fig.suptitle(f'Tracking Errors Comparison (scale={scale}x){title_suffix}')
    plt.tight_layout()
    return fig


def plot_inputs_comparison(results_lqr, results_pp, scale, title_suffix=""):
    """Plot control inputs comparison for LQR and PP."""
    fig, axes = plt.subplots(2, 1, figsize=(10, 6))
    
    axes[0].plot(results_lqr['t'], np.rad2deg(results_lqr['u_log'][:, 0]), 'b-', linewidth=1.4, label='LQR')
    axes[0].plot(results_pp['t'], np.rad2deg(results_pp['u_log'][:, 0]), 'r--', linewidth=1.4, label='Pole Placement')
    axes[0].axhline(y=25, color='k', linestyle=':', alpha=0.5, label='Saturation')
    axes[0].axhline(y=-25, color='k', linestyle=':', alpha=0.5)
    axes[0].set_ylabel('delta [deg]')
    axes[0].set_title('Steering input')
    axes[0].legend()
    axes[0].grid(True)
    
    axes[1].plot(results_lqr['t'], results_lqr['u_log'][:, 1], 'b-', linewidth=1.4, label='LQR')
    axes[1].plot(results_pp['t'], results_pp['u_log'][:, 1], 'r--', linewidth=1.4, label='Pole Placement')
    axes[1].axhline(y=3, color='k', linestyle=':', alpha=0.5, label='Saturation')
    axes[1].axhline(y=-6, color='k', linestyle=':', alpha=0.5)
    axes[1].set_xlabel('t [s]')
    axes[1].set_ylabel('a_x [m/s^2]')
    axes[1].set_title('Longitudinal acceleration')
    axes[1].legend()
    axes[1].grid(True)
    
    fig.suptitle(f'Control Inputs Comparison (scale={scale}x){title_suffix}')
    plt.tight_layout()
    return fig


# =============================================================================
# Metrics Functions
# =============================================================================

def compute_metrics(results):
    """Compute numerical performance metrics."""
    ey = results['ey_log']
    epsi = results['epsi_log']
    ev = results['ev_log']
    u_log = results['u_log']
    
    # RMS errors
    rms_ey = np.sqrt(np.mean(ey**2))
    rms_epsi = np.sqrt(np.mean(epsi**2))
    rms_ev = np.sqrt(np.mean(ev**2))
    
    # Max absolute errors
    max_ey = np.max(np.abs(ey))
    max_epsi = np.max(np.abs(epsi))
    max_ev = np.max(np.abs(ev))
    
    # Final errors
    final_ey = ey[-1]
    final_epsi = epsi[-1]
    final_ev = ev[-1]
    
    # Control effort (RMS)
    rms_steering = np.sqrt(np.mean(u_log[:, 0]**2))
    rms_accel = np.sqrt(np.mean(u_log[:, 1]**2))
    
    # Saturation counts
    steering_sat_count = np.sum(np.abs(u_log[:, 0]) >= np.deg2rad(25))
    accel_sat_count = np.sum((u_log[:, 1] <= -6) | (u_log[:, 1] >= 3))
    
    return {
        'rms_ey': rms_ey,
        'rms_epsi': rms_epsi,
        'rms_ev': rms_ev,
        'max_ey': max_ey,
        'max_epsi': max_epsi,
        'max_ev': max_ev,
        'final_ey': final_ey,
        'final_epsi': final_epsi,
        'final_ev': final_ev,
        'rms_steering': rms_steering,
        'rms_accel': rms_accel,
        'steering_sat_count': steering_sat_count,
        'accel_sat_count': accel_sat_count,
    }


def create_metrics_summary(all_results):
    """Create metrics summary table from all results."""
    summary_lines = []
    summary_lines.append("# Performance Metrics Summary\n")
    summary_lines.append("Comparison of LQR and Pole Placement controllers across different initial error scales.\n\n")
    summary_lines.append("| Regulator | Scale | RMS e_y [m] | Max |e_y| [m] | RMS e_psi [deg] | Max |e_psi| [deg] | RMS e_v [m/s] | Max |e_v| [m/s] |\n")
    summary_lines.append("|-----------|-------|-------------|-----------------|------------------|-------------------|----------------|------------------|\n")
    
    for regulator in ['LQR', 'PP']:
        for scale in [1, 2, 3]:
            key = (regulator, scale)
            if key in all_results:
                results = all_results[key]
                metrics = compute_metrics(results)
                summary_lines.append(
                    f"| {regulator} | {scale}x | "
                    f"{metrics['rms_ey']:.6f} | {metrics['max_ey']:.6f} | "
                    f"{np.rad2deg(metrics['rms_epsi']):.6f} | {np.rad2deg(metrics['max_epsi']):.6f} | "
                    f"{metrics['rms_ev']:.6f} | {metrics['max_ev']:.6f} |\n"
                )
    
    summary_lines.append("\n## Additional Metrics\n\n")
    summary_lines.append("| Regulator | Scale | RMS Steering [deg] | RMS Accel [m/s²] | Steering Saturation | Accel Saturation |\n")
    summary_lines.append("|-----------|-------|---------------------|------------------|---------------------|------------------|\n")
    
    for regulator in ['LQR', 'PP']:
        for scale in [1, 2, 3]:
            key = (regulator, scale)
            if key in all_results:
                results = all_results[key]
                metrics = compute_metrics(results)
                N = len(results['t'])
                summary_lines.append(
                    f"| {regulator} | {scale}x | "
                    f"{np.rad2deg(metrics['rms_steering']):.6f} | {metrics['rms_accel']:.6f} | "
                    f"{metrics['steering_sat_count']}/{N} | {metrics['accel_sat_count']}/{N} |\n"
                )
    
    return ''.join(summary_lines)
