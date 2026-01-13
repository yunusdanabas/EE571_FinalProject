# vehicle_dlqr_trajectory_tracking_demo.py
# Discrete-time model + DLQR controller for vehicle trajectory tracking
# - Nonlinear plant: bicycle model (4-wheel proxy)
# - Control design: linearized error model (continuous) -> ZOH discretization -> dlqr
# - Control updated every Ts with zero-order hold
#
# Plant state: [X, Y, psi, vx, vy, r]
# Error state (control design): xe = [vy, r, ey, epsi, ev]
# Input: u = [steering_input, throttle]

import numpy as np
import matplotlib.pyplot as plt
import argparse
import os
from scipy.linalg import expm, solve_discrete_are
from scipy.signal import place_poles

# ======================= Helper functions ================================
def bicycle_dynamics(x, u, params):
    """
    Nonlinear bicycle model with linear tire forces (small slip angles)
    State: x = [X, Y, psi, vx, vy, r]
    Input: u = [delta, ax]
    """
    X = x[0]
    Y = x[1]
    psi = x[2]
    vx = x[3]
    vy = x[4]
    r = x[5]

    delta = u[0]
    ax = u[1]

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


def integrate_reference(t, v_ref, kappa_ref):
    """
    Integrate reference pose from v and curvature:
    psidot = v*kappa, Xdot = v cos psi, Ydot = v sin psi
    """
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


def lateral_heading_error(X, Y, psi, Xr, Yr, psir):
    """
    Cross-track error (in ref normal direction) and heading error
    """
    ep = np.array([X - Xr, Y - Yr])
    n_hat = np.array([-np.sin(psir), np.cos(psir)])
    ey = n_hat.T @ ep
    epsi = wrap_to_pi(psi - psir)
    return ey, epsi


def rk4_step(f, x, h):
    """
    One Runge-Kutta 4 step
    """
    k1 = f(x)
    k2 = f(x + 0.5 * h * k1)
    k3 = f(x + 0.5 * h * k2)
    k4 = f(x + h * k3)
    return x + (h / 6) * (k1 + 2 * k2 + 2 * k3 + k4)


def c2d_zoh_exact(A, B, Ts):
    """
    Exact ZOH discretization using augmented matrix exponential:
    exp([A B; 0 0]*Ts) = [Ad Bd; 0 I]
    """
    n = A.shape[0]
    m = B.shape[1]
    M = np.block([[A, B], [np.zeros((m, n)), np.zeros((m, m))]])
    Md = expm(M * Ts)
    Ad = Md[:n, :n]
    Bd = Md[:n, n:n+m]
    return Ad, Bd


def wrap_to_pi(angle):
    """
    Wrap angle to [-pi, pi]
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi

# ----------------------- Vehicle parameters ------------------------------
params = {
    'm': 1500,      # mass [kg]
    'Iz': 2500,     # yaw inertia [kg m^2]
    'lf': 1.2,      # CG to front axle [m]
    'lr': 1.6,      # CG to rear axle [m]
    'Cf': 80000,    # front cornering stiffness [N/rad]
    'Cr': 80000,    # rear cornering stiffness [N/rad]
}

# ----------------------- Sampling / simulation ---------------------------
Ts = 0.02        # controller sampling time [s] (50 Hz)
Tend = 25        # [s]
t = np.arange(0, Tend + Ts, Ts)
N = len(t)

# Nominal linearization speed
Vx0 = 15           # [m/s]

# ----------------------- Reference trajectory ----------------------------
kappa_ref = 0.01 * np.sin(0.35 * t) + 0.005 * np.sin(0.10 * t)   # [1/m]
v_ref = Vx0 + 1.0 * np.sin(0.15 * t)                              # [m/s]
a_ref = np.append(np.diff(v_ref) / Ts, 0)                         # approx dv/dt

Xref, Yref, psiref = integrate_reference(t, v_ref, kappa_ref)

# ----------------------- Continuous-time linear error model --------------
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

# Continuous domain model of the controller:
# error_state_derivative = Ac*error_state + Bc*input
# Your controller is supposed to regulate the error states!

Ac = np.zeros((5, 5))
Bc = np.zeros((5, 2))

# [vy; r]
Ac[0, 0] = A11
Ac[0, 1] = A12
Ac[1, 0] = A21
Ac[1, 1] = A22
Bc[0, 0] = Bvydelta
Bc[1, 0] = Brdelta

# ey_dot = vy + Vx0*epsi
Ac[2, 0] = 1
Ac[2, 3] = Vx0

# epsi_dot = r   (ref curvature handled by feedforward)
Ac[3, 1] = 1

# ev_dot = ax    (ref accel handled by feedforward)
Bc[4, 1] = 1

# ---------------------------------------------------------------
# DESIGN YOUR CONTROLLER HERE!
# get the error state dynamics (using the associated matrices)
# don't forget to discretize it using c2d_zoh_exact below!
# get the gain vector (compatible to the error state vector), etc.
#---------------------------------------------------------------

# Discretize the continuous error model using ZOH
Ad, Bd = c2d_zoh_exact(Ac, Bc, Ts)

# LQR Controller Design
Q = np.diag([5.0, 5.0, 50.0, 50.0, 30.0])  # 5x5, positive semi-definite
R = np.diag([2.0, 1.0])  # 2x2, positive definite

# Solve discrete algebraic Riccati equation
P = solve_discrete_are(Ad, Bd, Q, R)

# Compute LQR feedback gain
K_LQR = np.linalg.inv(R + Bd.T @ P @ Bd) @ (Bd.T @ P @ Ad)

# Pole Placement Controller Design (for comparison experiments)
desired_poles = np.array([0.85, 0.80, 0.75, 0.70, 0.65])
result = place_poles(Ad, Bd, desired_poles)
K_PP = result.gain_matrix

# ======================= Function for batch experiments ====================
def run_simulation(controller_type='LQR', scale=1, K=None):
    """
    Run vehicle simulation with specified controller.
    
    Args:
        controller_type: 'LQR', 'PP', or 'feedforward_only'
        scale: Initial error scale (1, 2, or 3)
        K: Controller gain matrix (2x5), if None uses controller_type to select
    
    Returns:
        Dictionary with simulation results
    """
    # Use provided K or select based on controller_type
    if K is None:
        if controller_type == 'LQR':
            K = K_LQR
        elif controller_type == 'PP':
            K = K_PP
        else:
            K = None
    
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
        X = x[k, 0]
        Y = x[k, 1]
        psi = x[k, 2]
        vx = x[k, 3]
        vy = x[k, 4]
        r = x[k, 5]
        
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
            xk = rk4_step(lambda xx: bicycle_dynamics(xx, u_input, params), xk, dt_int)
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


# ======================= Batch experiment functions ======================
def run_batch_experiments():
    """
    Run all 6 experiments (2 controllers × 3 scales) and generate comparison plots.
    """
    # Import plotting functions (local import to avoid circular dependencies)
    from plotting import (
        plot_trajectory_comparison,
        plot_errors_comparison,
        plot_inputs_comparison,
        create_metrics_summary,
    )
    
    # Set up paths
    script_dir = os.path.dirname(os.path.abspath(__file__))
    results_dir = os.path.join(script_dir, '..', 'results')
    plots_dir = os.path.join(results_dir, 'plots')
    
    # Create output directory
    os.makedirs(plots_dir, exist_ok=True)
    
    # Experiment matrix: 2 regulators × 3 scales = 6 cases
    configs = [
        ('LQR', 1), ('LQR', 2), ('LQR', 3),
        ('PP', 1), ('PP', 2), ('PP', 3),
    ]
    
    # Store all results
    all_results = {}
    
    # Run all experiments
    print("Running experiments...")
    for regulator, scale in configs:
        print(f"  {regulator} controller, scale={scale}x")
        K = K_LQR if regulator == 'LQR' else K_PP
        results = run_simulation(controller_type=regulator, scale=scale, K=K)
        all_results[(regulator, scale)] = results
    
    # Generate comparison plots for each scale
    print("\nGenerating comparison plots...")
    for scale in [1, 2, 3]:
        results_lqr = all_results[('LQR', scale)]
        results_pp = all_results[('PP', scale)]
        
        # Generate comparison plots
        fig1 = plot_trajectory_comparison(results_lqr, results_pp, scale)
        fig2 = plot_errors_comparison(results_lqr, results_pp, scale)
        fig3 = plot_inputs_comparison(results_lqr, results_pp, scale)
        
        # Save plots
        fig1.savefig(os.path.join(plots_dir, f'trajectory_scale{scale}.png'), dpi=150, bbox_inches='tight')
        fig2.savefig(os.path.join(plots_dir, f'errors_scale{scale}.png'), dpi=150, bbox_inches='tight')
        fig3.savefig(os.path.join(plots_dir, f'inputs_scale{scale}.png'), dpi=150, bbox_inches='tight')
        
        plt.close('all')
        print(f"  Scale {scale}x plots saved")
    
    # Generate metrics summary
    print("\nGenerating metrics summary...")
    metrics_summary = create_metrics_summary(all_results)
    
    metrics_file = os.path.join(results_dir, 'metrics_summary.md')
    with open(metrics_file, 'w') as f:
        f.write(metrics_summary)
    
    print("  Metrics summary saved")
    print("\nAll experiments complete!")


# ======================= Main script execution ============================
if __name__ == "__main__":
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Vehicle trajectory tracking with DLQR/Pole Placement')
    parser.add_argument('--batch', action='store_true',
                        help='Run batch experiments (6 cases: 2 controllers × 3 scales)')
    args = parser.parse_args()
    
    if args.batch:
        # Run batch experiments
        run_batch_experiments()
    else:
        # Run single simulation (default, matches MATLAB behavior)
        # ----------------------- Nonlinear plant simulation -----------------------
        # We'll simulate the nonlinear plant with a smaller integration step inside each Ts
        dt_int = Ts / 10                   # integration step for RK4
        n_int = int(round(Ts / dt_int))

        x = np.zeros((N, 6))                   # [X Y psi vx vy r]

        # Initial condition offset from reference
        x[0, :] = [Xref[0] - 2.0, Yref[0] + 1.0, psiref[0] + np.deg2rad(8), Vx0 - 5, 0.0, 0.0]

        u_log = np.zeros((N, 2))
        ey_log = np.zeros(N)
        epsi_log = np.zeros(N)
        ev_log = np.zeros(N)

        for k in range(N - 1):
            # current plant state
            X = x[k, 0]
            Y = x[k, 1]
            psi = x[k, 2]
            vx = x[k, 3]
            vy = x[k, 4]
            r = x[k, 5]

            # reference at this sample
            Xr = Xref[k]
            Yr = Yref[k]
            psir = psiref[k]
            vr = v_ref[k]
            ar = a_ref[k]
            kap = kappa_ref[k]

            # errors
            ey, epsi = lateral_heading_error(X, Y, psi, Xr, Yr, psir)
            ev = vx - vr

            # simple feedforward (starter) inputs:
            steering_feed_forward = (lf + lr) * kap
            throttle_feed_forward = ar

            #---------------------------------------------------------------
            # CALCULATE THE INPUTS HERE! Don't forget to add the feed-forward inputs to
            # your regulation inputs.
            # steering_input = ...;
            # throttle = ...;
            #---------------------------------------------------------------

            # Build error state vector
            x_e = np.array([vy, r, ey, epsi, ev])
            
            # Compute regulation inputs using LQR
            u_reg = -K_LQR @ x_e
            steering_reg = u_reg[0]
            ax_reg = u_reg[1]
            
            # Combine feedforward and regulation
            steering_input = steering_feed_forward + steering_reg
            throttle = throttle_feed_forward + ax_reg

            # saturations (optional but realistic)
            steering_input = np.clip(steering_input, np.deg2rad(-25), np.deg2rad(25))
            throttle = np.clip(throttle, -6, 3)

            u_log[k, :] = [steering_input, throttle]
            ey_log[k] = ey
            epsi_log[k] = epsi
            ev_log[k] = ev

            # propagate nonlinear plant over [t_k, t_{k+1}] with ZOH input
            xk = x[k, :].copy()
            for j in range(n_int):
                xk = rk4_step(lambda xx: bicycle_dynamics(xx, np.array([steering_input, throttle]), params), xk, dt_int)
                xk[3] = max(xk[3], 0.5)  # keep vx positive
            x[k + 1, :] = xk

        # log last
        u_log[-1, :] = u_log[-2, :]
        ey_log[-1] = ey_log[-2]
        epsi_log[-1] = epsi_log[-2]
        ev_log[-1] = ev_log[-2]

        # ----------------------- Plots -------------------------------------------
        plt.figure('Trajectory')
        plt.plot(Xref, Yref, linewidth=1.6, label='Reference')
        plt.plot(x[:, 0], x[:, 1], '--', linewidth=1.4, label='Vehicle (nonlinear sim)')
        plt.axis('equal')
        plt.grid(True)
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.legend(loc='best')
        plt.title('Trajectory tracking with DLQR (discrete controller, nonlinear plant)')

        plt.figure('Tracking errors')
        plt.subplot(3, 1, 1)
        plt.plot(t, ey_log, linewidth=1.2)
        plt.grid(True)
        plt.ylabel('e_y [m]')
        plt.title('Cross-track error')

        plt.subplot(3, 1, 2)
        plt.plot(t, np.rad2deg(epsi_log), linewidth=1.2)
        plt.grid(True)
        plt.ylabel('e_psi [deg]')
        plt.title('Heading error')

        plt.subplot(3, 1, 3)
        plt.plot(t, ev_log, linewidth=1.2)
        plt.grid(True)
        plt.xlabel('t [s]')
        plt.ylabel('e_v [m/s]')
        plt.title('Speed error')

        plt.figure('Inputs')
        plt.subplot(2, 1, 1)
        plt.plot(t, np.rad2deg(u_log[:, 0]), linewidth=1.2)
        plt.grid(True)
        plt.ylabel('delta [deg]')
        plt.title('Steering input')

        plt.subplot(2, 1, 2)
        plt.plot(t, u_log[:, 1], linewidth=1.2)
        plt.grid(True)
        plt.xlabel('t [s]')
        plt.ylabel('a_x [m/s^2]')
        plt.title('Longitudinal accel command')

        plt.show()
