#!/usr/bin/env python3
"""
vehicle_tracking.py - Vehicle Trajectory Tracking with Discrete Controllers

Discrete-time model + DLQR/Pole-Placement controller for vehicle trajectory tracking
- Nonlinear plant: bicycle model
- Control design: linearized error model (continuous) -> ZOH discretization -> controller
- Control updated every Ts with zero-order hold

Plant state: [X, Y, psi, vx, vy, r]
Error state (control design): x_e = [vy, r, ey, epsi, ev]
Input: u = [steering_input, throttle]

Requires: numpy, scipy, matplotlib
"""

import numpy as np
from scipy.linalg import expm, solve_discrete_are
from scipy.signal import place_poles
import matplotlib.pyplot as plt

# =============================================================================
# Vehicle Parameters
# =============================================================================
params = {
    'm': 1500,      # mass [kg]
    'Iz': 2500,     # yaw inertia [kg m^2]
    'lf': 1.2,      # CG to front axle [m]
    'lr': 1.6,      # CG to rear axle [m]
    'Cf': 80000,    # front cornering stiffness [N/rad]
    'Cr': 80000,    # rear cornering stiffness [N/rad]
}

# =============================================================================
# Sampling / Simulation Parameters
# =============================================================================
Ts = 0.02       # controller sampling time [s] (50 Hz)
Tend = 25       # simulation duration [s]
Vx0 = 15        # nominal linearization speed [m/s]

# Time vector
t = np.arange(0, Tend + Ts, Ts)
N = len(t)

# =============================================================================
# Reference Trajectory
# =============================================================================
kappa_ref = 0.01 * np.sin(0.35 * t) + 0.005 * np.sin(0.10 * t)  # [1/m]
v_ref = Vx0 + 1.0 * np.sin(0.15 * t)                             # [m/s]
a_ref = np.append(np.diff(v_ref) / Ts, 0)                        # approx dv/dt

def integrate_reference(t, v_ref, kappa_ref):
    """Integrate reference pose from v and curvature."""
    Ts = t[1] - t[0]
    N = len(t)
    Xref = np.zeros(N)
    Yref = np.zeros(N)
    psiref = np.zeros(N)
    
    for k in range(N - 1):
        psidot = v_ref[k] * kappa_ref[k]
        psiref[k + 1] = psiref[k] + Ts * psidot
        Xref[k + 1] = Xref[k] + Ts * (v_ref[k] * np.cos(psiref[k]))
        Yref[k + 1] = Yref[k] + Ts * (v_ref[k] * np.sin(psiref[k]))
    
    return Xref, Yref, psiref

Xref, Yref, psiref = integrate_reference(t, v_ref, kappa_ref)

# =============================================================================
# Continuous-Time Linear Error Model
# =============================================================================
m = params['m']
Iz = params['Iz']
lf = params['lf']
lr = params['lr']
Cf = params['Cf']
Cr = params['Cr']

A11 = -(Cf + Cr) / (m * Vx0)
A12 = -(Vx0 + (lf * Cf - lr * Cr) / (m * Vx0))
A21 = -(lf * Cf - lr * Cr) / (Iz * Vx0)
A22 = -(lf**2 * Cf + lr**2 * Cr) / (Iz * Vx0)

Bvydelta = Cf / m
Brdelta = lf * Cf / Iz

# Continuous-time error model matrices
# State: x_e = [vy, r, ey, epsi, ev]
# Input: u_reg = [delta_reg, ax_reg]
Ac = np.zeros((5, 5))
Bc = np.zeros((5, 2))

# [vy, r] dynamics
Ac[0, 0] = A11
Ac[0, 1] = A12
Ac[1, 0] = A21
Ac[1, 1] = A22
Bc[0, 0] = Bvydelta
Bc[1, 0] = Brdelta

# ey_dot = vy + Vx0 * epsi
Ac[2, 0] = 1
Ac[2, 3] = Vx0

# epsi_dot = r (ref curvature handled by feedforward)
Ac[3, 1] = 1

# ev_dot = ax (ref accel handled by feedforward)
Bc[4, 1] = 1

# =============================================================================
# Discretization (ZOH)
# =============================================================================
def c2d_zoh_exact(Ac, Bc, Ts):
    """
    Exact ZOH discretization using augmented matrix exponential.
    exp([Ac Bc; 0 0] * Ts) = [Ad Bd; 0 I]
    """
    n = Ac.shape[0]
    m = Bc.shape[1]
    M = np.block([[Ac, Bc], [np.zeros((m, n)), np.zeros((m, m))]])
    Md = expm(M * Ts)
    Ad = Md[:n, :n]
    Bd = Md[:n, n:n+m]
    return Ad, Bd

Ad, Bd = c2d_zoh_exact(Ac, Bc, Ts)

# Verify dimensions
assert Ad.shape == (5, 5), "Ad must be 5x5"
assert Bd.shape == (5, 2), "Bd must be 5x2"

# Compute eigenvalues of Ad for reference
eig_Ad = np.linalg.eigvals(Ad)
print("Discretization complete:")
print(f"  Ad: {Ad.shape[0]}x{Ad.shape[1]}, Bd: {Bd.shape[0]}x{Bd.shape[1]}")
print(f"  Eigenvalues of Ad: {eig_Ad}")

# =============================================================================
# Controller Design
# =============================================================================
# LQR Controller Design (Part 3)
# Q matrix: Penalize error states [vy, r, ey, epsi, ev]
# Higher weights on tracking errors (ey, epsi, ev) than internal states (vy, r)
Q = np.diag([5.0, 5.0, 50.0, 50.0, 30.0])  # 5x5, positive semi-definite

# R matrix: Penalize control effort [delta_reg, ax_reg]
# Moderate weights to allow reasonable control authority
R = np.diag([2.0, 1.0])  # 2x2, positive definite

# Solve discrete algebraic Riccati equation
P = solve_discrete_are(Ad, Bd, Q, R)

# Compute LQR feedback gain
K_LQR = np.linalg.inv(R + Bd.T @ P @ Bd) @ (Bd.T @ P @ Ad)

# Verify K_LQR dimensions
assert K_LQR.shape == (2, 5), f"K_LQR must be 2x5, got {K_LQR.shape}"
assert not np.any(np.isnan(K_LQR)) and not np.any(np.isinf(K_LQR)), "K_LQR contains NaN or Inf values"

# Compute closed-loop eigenvalues for stability check
eig_cl = np.linalg.eigvals(Ad - Bd @ K_LQR)
print("\nLQR Controller Design:")
print(f"  Q matrix: diag({Q.diagonal()})")
print(f"  R matrix: diag({R.diagonal()})")
print(f"  K_LQR shape: {K_LQR.shape}")
print(f"  Closed-loop eigenvalues: {eig_cl}")
print(f"  All eigenvalues inside unit circle: {np.all(np.abs(eig_cl) < 1.0)}")

# Pole Placement Controller Design (Part 4)
# Choose 5 real poles inside unit circle (stable, discrete-time)
# Poles selected for good tracking performance: fast but stable
desired_poles = np.array([0.85, 0.80, 0.75, 0.70, 0.65])  # All real, inside unit circle

# Compute pole placement gain using scipy
result = place_poles(Ad, Bd, desired_poles)
K_PP = result.gain_matrix

# Verify K_PP dimensions
assert K_PP.shape == (2, 5), f"K_PP must be 2x5, got {K_PP.shape}"
assert not np.any(np.isnan(K_PP)) and not np.any(np.isinf(K_PP)), "K_PP contains NaN or Inf values"

# Compute closed-loop eigenvalues for verification
eig_cl_pp = np.linalg.eigvals(Ad - Bd @ K_PP)
print("\nPole Placement Controller Design:")
print(f"  Desired poles: {desired_poles}")
print(f"  K_PP shape: {K_PP.shape}")
print(f"  Closed-loop eigenvalues: {eig_cl_pp}")
print(f"  All eigenvalues real: {np.all(np.isreal(eig_cl_pp))}")
print(f"  All eigenvalues inside unit circle: {np.all(np.abs(eig_cl_pp) < 1.0)}")

# Verify all eigenvalues are real and inside unit circle
assert np.all(np.isreal(eig_cl_pp)), "All closed-loop eigenvalues must be real"
assert np.all(np.abs(eig_cl_pp) < 1.0), "All closed-loop eigenvalues must be inside unit circle"

# Select which controller to use
controller_type = 'LQR'  # Options: 'LQR', 'PP', 'feedforward_only'

# =============================================================================
# Helper Functions
# =============================================================================
def wrap_to_pi(angle):
    """Wrap angle to [-pi, pi]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi

def lateral_heading_error(X, Y, psi, Xr, Yr, psir):
    """Compute cross-track error and heading error."""
    ep = np.array([X - Xr, Y - Yr])
    n_hat = np.array([-np.sin(psir), np.cos(psir)])
    ey = n_hat @ ep
    epsi = wrap_to_pi(psi - psir)
    return ey, epsi

def bicycle_dynamics(x, u, params):
    """
    Nonlinear bicycle model with linear tire forces (small slip angles).
    State: x = [X, Y, psi, vx, vy, r]
    Input: u = [delta, ax]
    """
    X, Y, psi, vx, vy, r = x
    delta, ax = u
    
    m = params['m']
    Iz = params['Iz']
    lf = params['lf']
    lr = params['lr']
    Cf = params['Cf']
    Cr = params['Cr']
    
    vx_eff = max(vx, 0.5)
    
    alpha_f = delta - (vy + lf * r) / vx_eff
    alpha_r = -(vy - lr * r) / vx_eff
    
    Fyf = Cf * alpha_f
    Fyr = Cr * alpha_r
    
    Xdot = vx * np.cos(psi) - vy * np.sin(psi)
    Ydot = vx * np.sin(psi) + vy * np.cos(psi)
    psidot = r
    
    vxdot = ax + r * vy
    vydot = (Fyf + Fyr) / m - r * vx
    rdot = (lf * Fyf - lr * Fyr) / Iz
    
    return np.array([Xdot, Ydot, psidot, vxdot, vydot, rdot])

def rk4_step(f, x, u, params, h):
    """One Runge-Kutta 4 step."""
    k1 = f(x, u, params)
    k2 = f(x + 0.5 * h * k1, u, params)
    k3 = f(x + 0.5 * h * k2, u, params)
    k4 = f(x + h * k3, u, params)
    return x + (h / 6) * (k1 + 2 * k2 + 2 * k3 + k4)

# =============================================================================
# Simulation
# =============================================================================
def run_simulation(controller_type='feedforward_only', scale=1, K=None):
    """
    Run vehicle simulation with specified controller.
    
    Args:
        controller_type: 'LQR', 'PP', or 'feedforward_only'
        scale: Initial error scale (1, 2, or 3)
        K: Controller gain matrix (2x5)
    
    Returns:
        Dictionary with simulation results
    """
    dt_int = Ts / 10  # integration step for RK4
    n_int = int(round(Ts / dt_int))
    
    # State storage
    x = np.zeros((N, 6))  # [X, Y, psi, vx, vy, r]
    
    # Initial condition offset from reference (scaled)
    x[0, :] = [
        Xref[0] - 2.0 * scale,
        Yref[0] + 1.0 * scale,
        psiref[0] + np.deg2rad(8 * scale),
        Vx0 - 5 * scale,
        0.0,
        0.0
    ]
    
    # Logging arrays
    u_log = np.zeros((N, 2))
    ey_log = np.zeros(N)
    epsi_log = np.zeros(N)
    ev_log = np.zeros(N)
    
    for k in range(N - 1):
        # Current plant state
        X, Y, psi, vx, vy, r = x[k, :]
        
        # Reference at this sample
        Xr = Xref[k]
        Yr = Yref[k]
        psir = psiref[k]
        vr = v_ref[k]
        ar = a_ref[k]
        kap = kappa_ref[k]
        
        # Compute errors
        ey, epsi = lateral_heading_error(X, Y, psi, Xr, Yr, psir)
        ev = vx - vr
        
        # Feedforward inputs
        steering_feed_forward = (lf + lr) * kap
        throttle_feed_forward = ar
        
        # Build error state vector
        x_e = np.array([vy, r, ey, epsi, ev])
        
        # Compute regulation inputs
        if controller_type == 'feedforward_only' or K is None:
            steering_reg = 0.0
            ax_reg = 0.0
        else:
            u_reg = -K @ x_e
            steering_reg = u_reg[0]
            ax_reg = u_reg[1]
        
        # Combine feedforward and regulation
        steering_input = steering_feed_forward + steering_reg
        throttle = throttle_feed_forward + ax_reg
        
        # Apply saturations
        steering_input = np.clip(steering_input, np.deg2rad(-25), np.deg2rad(25))
        throttle = np.clip(throttle, -6, 3)
        
        # Log
        u_log[k, :] = [steering_input, throttle]
        ey_log[k] = ey
        epsi_log[k] = epsi
        ev_log[k] = ev
        
        # Propagate nonlinear plant over [t_k, t_{k+1}] with ZOH input
        xk = x[k, :].copy()
        u_input = np.array([steering_input, throttle])
        for j in range(n_int):
            xk = rk4_step(bicycle_dynamics, xk, u_input, params, dt_int)
            xk[3] = max(xk[3], 0.5)  # keep vx positive
        x[k + 1, :] = xk
    
    # Log last values
    u_log[-1, :] = u_log[-2, :]
    ey_log[-1] = ey_log[-2]
    epsi_log[-1] = epsi_log[-2]
    ev_log[-1] = ev_log[-2]
    
    return {
        't': t,
        'x': x,
        'u_log': u_log,
        'ey_log': ey_log,
        'epsi_log': epsi_log,
        'ev_log': ev_log,
        'Xref': Xref,
        'Yref': Yref,
        'psiref': psiref,
    }

# =============================================================================
# Plotting Functions
# =============================================================================
def plot_trajectory(results, title="Trajectory"):
    """Plot vehicle trajectory vs reference."""
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.plot(results['Xref'], results['Yref'], 'b-', linewidth=1.6, label='Reference')
    ax.plot(results['x'][:, 0], results['x'][:, 1], 'r--', linewidth=1.4, label='Vehicle')
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_title(title)
    ax.legend(loc='best')
    ax.axis('equal')
    ax.grid(True)
    return fig

def plot_errors(results, title="Tracking Errors"):
    """Plot tracking errors."""
    fig, axes = plt.subplots(3, 1, figsize=(10, 8))
    
    axes[0].plot(results['t'], results['ey_log'], linewidth=1.2)
    axes[0].set_ylabel('e_y [m]')
    axes[0].set_title('Cross-track error')
    axes[0].grid(True)
    
    axes[1].plot(results['t'], np.rad2deg(results['epsi_log']), linewidth=1.2)
    axes[1].set_ylabel('e_psi [deg]')
    axes[1].set_title('Heading error')
    axes[1].grid(True)
    
    axes[2].plot(results['t'], results['ev_log'], linewidth=1.2)
    axes[2].set_xlabel('t [s]')
    axes[2].set_ylabel('e_v [m/s]')
    axes[2].set_title('Speed error')
    axes[2].grid(True)
    
    fig.suptitle(title)
    plt.tight_layout()
    return fig

def plot_inputs(results, title="Control Inputs"):
    """Plot control inputs."""
    fig, axes = plt.subplots(2, 1, figsize=(10, 6))
    
    axes[0].plot(results['t'], np.rad2deg(results['u_log'][:, 0]), linewidth=1.2)
    axes[0].axhline(y=25, color='r', linestyle='--', alpha=0.5, label='Saturation')
    axes[0].axhline(y=-25, color='r', linestyle='--', alpha=0.5)
    axes[0].set_ylabel('delta [deg]')
    axes[0].set_title('Steering input')
    axes[0].legend()
    axes[0].grid(True)
    
    axes[1].plot(results['t'], results['u_log'][:, 1], linewidth=1.2)
    axes[1].axhline(y=3, color='r', linestyle='--', alpha=0.5, label='Saturation')
    axes[1].axhline(y=-6, color='r', linestyle='--', alpha=0.5)
    axes[1].set_xlabel('t [s]')
    axes[1].set_ylabel('a_x [m/s^2]')
    axes[1].set_title('Longitudinal acceleration')
    axes[1].legend()
    axes[1].grid(True)
    
    fig.suptitle(title)
    plt.tight_layout()
    return fig

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

# =============================================================================
# Main
# =============================================================================
if __name__ == "__main__":
    import os
    
    print("\n" + "="*60)
    print("Vehicle Trajectory Tracking Simulation - Part 5")
    print("Running 6-case experiment matrix (2 regulators × 3 scales)")
    print("="*60)
    
    # Ensure directories exist
    os.makedirs('results/plots', exist_ok=True)
    os.makedirs('results/logs', exist_ok=True)
    os.makedirs('output/part3', exist_ok=True)
    os.makedirs('output/part4', exist_ok=True)
    os.makedirs('output/part5', exist_ok=True)
    
    # Define experiment matrix: 2 regulators × 3 scales = 6 cases
    configs = [
        ('LQR', 1), ('LQR', 2), ('LQR', 3),
        ('PP', 1), ('PP', 2), ('PP', 3),
    ]
    
    # Store all results
    all_results = {}
    
    # Run all 6 experiments
    print("\nRunning all 6 experiments...")
    for i, (regulator, scale) in enumerate(configs, 1):
        print(f"\n[{i}/6] Running {regulator} controller, scale={scale}x...")
        try:
            K = K_LQR if regulator == 'LQR' else K_PP
            results = run_simulation(controller_type=regulator, scale=scale, K=K)
            all_results[(regulator, scale)] = results
            
            # Generate and save individual plots for each run
            part_dir = 'output/part3' if regulator == 'LQR' else 'output/part4'
            prefix = 'lqr' if regulator == 'LQR' else 'pp'
            
            # Generate plots
            fig1 = plot_trajectory(results, title=f'{regulator} Controller - Trajectory (scale={scale}x)')
            fig2 = plot_errors(results, title=f'{regulator} Controller - Tracking Errors (scale={scale}x)')
            fig3 = plot_inputs(results, title=f'{regulator} Controller - Control Inputs (scale={scale}x)')
            
            # Save plots to output directory
            fig1.savefig(f'{part_dir}/{prefix}_scale{scale}_trajectory.png', dpi=150, bbox_inches='tight')
            fig2.savefig(f'{part_dir}/{prefix}_scale{scale}_errors.png', dpi=150, bbox_inches='tight')
            fig3.savefig(f'{part_dir}/{prefix}_scale{scale}_inputs.png', dpi=150, bbox_inches='tight')
            
            plt.close('all')
            
            # Compute and save numerical outputs
            metrics = compute_metrics(results)
            N = len(results['t'])
            
            output_file = f'{part_dir}/{prefix}_scale{scale}_outputs.txt'
            with open(output_file, 'w') as f:
                f.write(f"{regulator} Controller - Scale {scale}x\n")
                f.write("="*60 + "\n\n")
                
                if regulator == 'LQR':
                    f.write("LQR Controller Design:\n")
                    f.write(f"  Q matrix: diag({Q.diagonal()})\n")
                    f.write(f"  R matrix: diag({R.diagonal()})\n")
                    f.write(f"  K_LQR shape: {K_LQR.shape}\n")
                    f.write(f"  K_LQR:\n{K_LQR}\n\n")
                    f.write(f"  Closed-loop eigenvalues: {eig_cl}\n")
                    f.write(f"  All eigenvalues inside unit circle: {np.all(np.abs(eig_cl) < 1.0)}\n\n")
                else:
                    f.write("Pole Placement Controller Design:\n")
                    f.write(f"  Desired poles: {desired_poles}\n")
                    f.write(f"  K_PP shape: {K_PP.shape}\n")
                    f.write(f"  K_PP:\n{K_PP}\n\n")
                    f.write(f"  Closed-loop eigenvalues: {eig_cl_pp}\n")
                    f.write(f"  All eigenvalues real: {np.all(np.isreal(eig_cl_pp))}\n")
                    f.write(f"  All eigenvalues inside unit circle: {np.all(np.abs(eig_cl_pp) < 1.0)}\n\n")
                
                f.write("Performance Metrics:\n")
                f.write("-"*60 + "\n")
                f.write(f"RMS e_y: {metrics['rms_ey']:.6f} m\n")
                f.write(f"Max |e_y|: {metrics['max_ey']:.6f} m\n")
                f.write(f"RMS e_psi: {np.rad2deg(metrics['rms_epsi']):.6f} deg\n")
                f.write(f"Max |e_psi|: {np.rad2deg(metrics['max_epsi']):.6f} deg\n")
                f.write(f"RMS e_v: {metrics['rms_ev']:.6f} m/s\n")
                f.write(f"Max |e_v|: {metrics['max_ev']:.6f} m/s\n\n")
                
                f.write(f"Final errors:\n")
                f.write(f"  e_y: {metrics['final_ey']:.6f} m\n")
                f.write(f"  e_psi: {np.rad2deg(metrics['final_epsi']):.6f} deg\n")
                f.write(f"  e_v: {metrics['final_ev']:.6f} m/s\n\n")
                
                f.write(f"Control effort:\n")
                f.write(f"  RMS steering: {np.rad2deg(metrics['rms_steering']):.6f} deg\n")
                f.write(f"  RMS acceleration: {metrics['rms_accel']:.6f} m/s²\n\n")
                
                f.write(f"Saturation:\n")
                f.write(f"  Steering saturation: {metrics['steering_sat_count']}/{N} ({100*metrics['steering_sat_count']/N:.2f}%)\n")
                f.write(f"  Acceleration saturation: {metrics['accel_sat_count']}/{N} ({100*metrics['accel_sat_count']/N:.2f}%)\n")
            
            print(f"  ✓ {regulator} scale={scale}x completed successfully")
            print(f"    - Plots saved to {part_dir}/")
            print(f"    - Numerical outputs saved to {output_file}")
        except Exception as e:
            print(f"  ✗ Error in {regulator} scale={scale}x: {e}")
            import traceback
            traceback.print_exc()
    
    print("\n" + "="*60)
    print("Generating comparison plots...")
    print("="*60)
    
    # Generate comparison plots for each scale
    for scale in [1, 2, 3]:
        lqr_key = ('LQR', scale)
        pp_key = ('PP', scale)
        
        if lqr_key in all_results and pp_key in all_results:
            print(f"\nGenerating comparison plots for scale={scale}x...")
            
            results_lqr = all_results[lqr_key]
            results_pp = all_results[pp_key]
            
            # Generate comparison plots
            fig1 = plot_trajectory_comparison(results_lqr, results_pp, scale)
            fig2 = plot_errors_comparison(results_lqr, results_pp, scale)
            fig3 = plot_inputs_comparison(results_lqr, results_pp, scale)
            
            # Save comparison plots to both results and output
            fig1.savefig(f'results/plots/trajectory_scale{scale}.png', dpi=150, bbox_inches='tight')
            fig2.savefig(f'results/plots/errors_scale{scale}.png', dpi=150, bbox_inches='tight')
            fig3.savefig(f'results/plots/inputs_scale{scale}.png', dpi=150, bbox_inches='tight')
            
            # Also save to output/part5
            fig1.savefig(f'output/part5/trajectory_scale{scale}.png', dpi=150, bbox_inches='tight')
            fig2.savefig(f'output/part5/errors_scale{scale}.png', dpi=150, bbox_inches='tight')
            fig3.savefig(f'output/part5/inputs_scale{scale}.png', dpi=150, bbox_inches='tight')
            
            plt.close('all')
            
            print(f"  ✓ Comparison plots saved for scale={scale}x")
            print(f"    - results/plots/ and output/part5/")
        else:
            print(f"  ✗ Missing results for scale={scale}x comparison")
    
    # Generate metrics summary
    print("\n" + "="*60)
    print("Computing metrics and generating summary...")
    print("="*60)
    
    metrics_summary = create_metrics_summary(all_results)
    
    # Save to both locations
    metrics_file_results = 'results/logs/metrics_summary.md'
    metrics_file_output = 'output/part5/metrics_summary.md'
    
    with open(metrics_file_results, 'w') as f:
        f.write(metrics_summary)
    
    with open(metrics_file_output, 'w') as f:
        f.write(metrics_summary)
    
    print(f"\n✓ Metrics summary saved to:")
    print(f"    - {metrics_file_results}")
    print(f"    - {metrics_file_output}")
    
    # Print summary
    print("\n" + "="*60)
    print("Experiment Summary")
    print("="*60)
    print(f"Total runs completed: {len(all_results)}/6")
    print(f"Individual plots: 18 (3 plot types × 6 runs)")
    print(f"Comparison plots: 9 (3 plot types × 3 scales)")
    print(f"All outputs saved to output/ directory:")
    print(f"  - output/part3/: LQR plots and numerical outputs (all scales)")
    print(f"  - output/part4/: PP plots and numerical outputs (all scales)")
    print(f"  - output/part5/: Comparison plots and metrics summary")
    print("\nAll experiments complete!")