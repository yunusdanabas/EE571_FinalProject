"""
Part 5: Discrete-Time Steady-State Kalman Filter (LQE)

This script designs and validates a discrete-time steady-state Kalman filter
for the stochastic 6-mass spring system with actuator and sensor noise.

Stochastic model:
    x_{k+1} = Ad x_k + Bd u_k + Bd w_k
    y_k     = C_d x_k + v_k

Noise distributions:
    v ~ N(0, 0.1 I_p), w ~ N(0, 0.05 I_m)
    p = number of outputs (2), m = number of inputs (3)

Source: docs/sources/final_exam_extract.md Section 7 (Part 5 Requirement)
"""

import numpy as np
import matplotlib.pyplot as plt
import os
import sys
import platform

# Add paths for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'utils'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'part2'))

from build_model import build_continuous_model, discretize_zoh
from observer_design import get_part2_C_matrix
from run_observer_sim import get_part2_initial_conditions
from scipy.linalg import solve_discrete_are


def design_kalman_filter(Ad, Cmeas, Qx, Rv):
    """
    Design steady-state Kalman filter using DARE.
    
    Solves the discrete-time algebraic Riccati equation for the estimator:
        P = solve_discrete_are(Ad.T, Cmeas.T, Qx, Rv)
    
    Then computes the steady-state Kalman gain:
        Lk = P @ Cmeas.T @ inv(Cmeas @ P @ Cmeas.T + Rv)
    
    Args:
        Ad: (n, n) discrete-time state matrix
        Cmeas: (p, n) measurement matrix
        Qx: (n, n) process noise covariance matrix
        Rv: (p, p) measurement noise covariance matrix
    
    Returns:
        tuple: (Lk, P, design_info) where
            - Lk: (n, p) Kalman gain matrix
            - P: (n, n) solution to DARE
            - design_info: dict with design parameters and validation
    """
    n = Ad.shape[0]
    p = Cmeas.shape[0]
    
    # Solve discrete-time algebraic Riccati equation (DARE) for estimator
    # DARE form for estimator: P = Ad^T P Ad - Ad^T P C^T (C P C^T + R)^(-1) C P Ad + Q
    # SciPy's solve_discrete_are solves: P = A^T P A - (A^T P B) (R + B^T P B)^(-1) (B^T P A) + Q
    # For estimator: A = Ad^T, B = Cmeas^T, Q = Qx, R = Rv
    try:
        P = solve_discrete_are(Ad.T, Cmeas.T, Qx, Rv)
    except Exception as e:
        raise RuntimeError(f"DARE solver failed: {e}")
    
    # Validate P is finite
    if not np.all(np.isfinite(P)):
        raise RuntimeError("DARE solution P contains non-finite values")
    
    # Compute innovation covariance: S = C P C^T + R
    S = Cmeas @ P @ Cmeas.T + Rv
    
    # Validate S is invertible
    S_cond = np.linalg.cond(S)
    S_det = np.linalg.det(S)
    if S_cond > 1e12 or abs(S_det) < 1e-12:
        raise RuntimeError(f"Innovation covariance S is poorly conditioned: cond(S) = {S_cond:.2e}, det(S) = {S_det:.2e}")
    
    # Compute steady-state Kalman gain: Lk = P @ C^T @ S^(-1)
    Lk = P @ Cmeas.T @ np.linalg.inv(S)
    
    # Validate dimensions
    if Lk.shape != (n, p):
        raise RuntimeError(f"Kalman gain Lk has wrong shape: {Lk.shape}, expected ({n}, {p})")
    
    # Compute estimator closed-loop matrix: Aest = Ad - Lk @ Cmeas
    Aest = Ad - Lk @ Cmeas
    
    # Compute eigenvalues and spectral radius
    eigvals = np.linalg.eigvals(Aest)
    spectral_radius = np.max(np.abs(eigvals))
    
    # Hard gate: spectral radius must be < 1.0
    if spectral_radius >= 1.0:
        raise RuntimeError(
            f"Estimator validation FAILED: spectral_radius = {spectral_radius:.6f} >= 1.0. "
            f"Estimator is unstable."
        )
    
    design_info = {
        'Lk': Lk,
        'Lk_shape': Lk.shape,
        'P': P,
        'S': S,
        'S_condition_number': S_cond,
        'S_determinant': S_det,
        'Aest': Aest,
        'eigenvalues': eigvals,
        'spectral_radius': spectral_radius,
        'is_stable': spectral_radius < 1.0
    }
    
    return Lk, P, design_info


def simulate_stochastic_system_with_kalman(Ad, Bd, Cmeas, Lk, x0, xhat0, u, N, Ts, Qw, Rv, seed=42):
    """
    Simulate stochastic system with Kalman filter.
    
    True system:
        x[k+1] = Ad @ x[k] + Bd @ u[k] + Bd @ w[k]
        y_true[k] = Cmeas @ x[k]  (true output, no noise)
        y_meas[k] = Cmeas @ x[k] + v[k]  (measured output, with noise)
    
    Kalman filter:
        xhat[k+1] = Ad @ xhat[k] + Bd @ u[k] + Lk @ (y_meas[k] - yhat[k])
        yhat[k] = Cmeas @ xhat[k]  (estimated output)
    
    Args:
        Ad: (n, n) discrete-time state matrix
        Bd: (n, m) discrete-time input matrix
        Cmeas: (p, n) measurement matrix
        Lk: (n, p) Kalman gain matrix
        x0: (n,) initial true state
        xhat0: (n,) initial estimated state
        u: (m, N) input sequence (all zeros for Part 5)
        N: Number of simulation steps
        Ts: Sampling time (for time vector)
        Qw: (m, m) actuator noise covariance
        Rv: (p, p) sensor noise covariance
        seed: Random seed for reproducibility (default 42)
    
    Returns:
        dict: Trajectories and noise samples
            - 'x': (n, N+1) true state trajectory
            - 'xhat': (n, N+1) estimated state trajectory
            - 'y_true': (p, N+1) true output trajectory (Cmeas @ x, no noise)
            - 'y_meas': (p, N+1) measured output trajectory (Cmeas @ x + v, with noise)
            - 'yhat': (p, N+1) estimated output trajectory (Cmeas @ xhat)
            - 'u': (m, N) input trajectory
            - 'w': (m, N) process noise samples
            - 'v': (p, N+1) measurement noise samples
            - 'innovations': (p, N) innovation sequence (y_meas - yhat)
            - 't': (N+1,) time vector
            - 'noise_samples': dict with first 3 samples of w and v
    """
    n = Ad.shape[0]
    m = Bd.shape[1]
    p = Cmeas.shape[0]
    
    # Set random seed for reproducibility
    np.random.seed(seed)
    
    # Preallocate arrays (standard convention)
    x = np.zeros((n, N + 1))
    xhat = np.zeros((n, N + 1))
    y_true = np.zeros((p, N + 1))
    y_meas = np.zeros((p, N + 1))
    yhat = np.zeros((p, N + 1))
    innovations = np.zeros((p, N))
    w_samples = np.zeros((m, N))
    v_samples = np.zeros((p, N + 1))
    
    # Initialize
    x[:, 0] = x0
    xhat[:, 0] = xhat0
    y_true[:, 0] = Cmeas @ x[:, 0]
    
    # Store first 3 noise samples for reproducibility logging
    noise_samples = {'w': [], 'v': []}
    
    # Simulate forward
    for k in range(N):
        # Generate process noise: w_k ~ N(0, Qw)
        w_k = np.random.multivariate_normal(np.zeros(m), Qw)
        w_samples[:, k] = w_k
        if k < 3:
            noise_samples['w'].append(w_k.copy())
        
        # Generate measurement noise: v_k ~ N(0, Rv)
        v_k = np.random.multivariate_normal(np.zeros(p), Rv)
        v_samples[:, k] = v_k
        if k < 3:
            noise_samples['v'].append(v_k.copy())
        
        # True output (no noise): y_true = Cmeas @ x
        y_true[:, k] = Cmeas @ x[:, k]
        
        # Measured output (with noise): y_meas = Cmeas @ x + v
        y_meas[:, k] = y_true[:, k] + v_k
        
        # Kalman filter output (prediction): yhat = Cmeas @ xhat
        yhat[:, k] = Cmeas @ xhat[:, k]
        
        # Innovation: r_k = y_meas[k] - yhat[k]
        innovations[:, k] = y_meas[:, k] - yhat[:, k]
        
        # True system update: x[k+1] = Ad @ x[k] + Bd @ u[k] + Bd @ w[k]
        if k < N:
            x[:, k + 1] = Ad @ x[:, k] + Bd @ u[:, k] + Bd @ w_k
        
        # Kalman filter update: xhat[k+1] = Ad @ xhat[k] + Bd @ u[k] + Lk @ innovations[k]
        if k < N:
            xhat[:, k + 1] = Ad @ xhat[:, k] + Bd @ u[:, k] + Lk @ innovations[:, k]
    
    # Final outputs at time N
    v_N = np.random.multivariate_normal(np.zeros(p), Rv)
    v_samples[:, N] = v_N
    y_true[:, N] = Cmeas @ x[:, N]
    y_meas[:, N] = y_true[:, N] + v_N
    yhat[:, N] = Cmeas @ xhat[:, N]
    
    # Time vector
    t = np.arange(N + 1) * Ts
    
    return {
        'x': x,
        'xhat': xhat,
        'y_true': y_true,
        'y_meas': y_meas,
        'yhat': yhat,
        'u': u,
        'w': w_samples,
        'v': v_samples,
        'innovations': innovations,
        't': t,
        'noise_samples': noise_samples
    }


def compute_rms_metrics(x, xhat, y_true, y_meas, yhat, steady_state_window=0.2):
    """
    Compute RMS estimation error metrics (full window and steady-state).
    
    Args:
        x: (n, N+1) true state trajectory
        xhat: (n, N+1) estimated state trajectory
        y_true: (p, N+1) true output trajectory (Cmeas @ x, no noise)
        y_meas: (p, N+1) measured output trajectory (Cmeas @ x + v, with noise)
        yhat: (p, N+1) estimated output trajectory (Cmeas @ xhat)
        steady_state_window: Fraction of samples for steady-state window (default 0.2 = last 20%)
    
    Returns:
        dict: RMS metrics (full window and steady-state)
    """
    n, N_plus_1 = x.shape
    p = y_true.shape[0]
    N = N_plus_1 - 1
    
    # Steady-state window: last steady_state_window fraction of samples
    ss_start_index = int((N + 1) * (1 - steady_state_window))
    
    # Estimation error: e = x - xhat
    e = x - xhat
    
    # Overall RMS estimation error: RMS(||x - xhat||_2)
    error_norm = np.array([np.linalg.norm(e[:, k]) for k in range(N + 1)])
    rms_overall = np.sqrt(np.mean(error_norm**2))
    rms_overall_ss = np.sqrt(np.mean(error_norm[ss_start_index:]**2))
    
    # Per-state RMS: RMS(e_i) for each state i
    rms_per_state = np.sqrt(np.mean(e**2, axis=1))
    rms_per_state_ss = np.sqrt(np.mean(e[:, ss_start_index:]**2, axis=1))
    
    # Position RMS: Focus on x1..x6 (displacement states)
    rms_positions = rms_per_state[:6]
    rms_positions_ss = rms_per_state_ss[:6]
    
    # Specific position errors (for steady-state reporting)
    rms_e_x1 = rms_per_state[0]
    rms_e_x6 = rms_per_state[5]
    rms_e_x1_ss = rms_per_state_ss[0]
    rms_e_x6_ss = rms_per_state_ss[5]
    
    # Output tracking RMS: RMS(y_true - yhat) for y1 and y6 (true vs estimated)
    output_error_true = y_true - yhat
    rms_output_y1 = np.sqrt(np.mean(output_error_true[0, :]**2))
    rms_output_y6 = np.sqrt(np.mean(output_error_true[1, :]**2))
    rms_output_y1_ss = np.sqrt(np.mean(output_error_true[0, ss_start_index:]**2))
    rms_output_y6_ss = np.sqrt(np.mean(output_error_true[1, ss_start_index:]**2))
    
    # Measured vs estimated output RMS (innovation/residual)
    output_error_meas = y_meas - yhat
    rms_output_meas_y1 = np.sqrt(np.mean(output_error_meas[0, :]**2))
    rms_output_meas_y6 = np.sqrt(np.mean(output_error_meas[1, :]**2))
    rms_output_meas_y1_ss = np.sqrt(np.mean(output_error_meas[0, ss_start_index:]**2))
    rms_output_meas_y6_ss = np.sqrt(np.mean(output_error_meas[1, ss_start_index:]**2))
    
    return {
        'rms_overall': rms_overall,
        'rms_overall_ss': rms_overall_ss,
        'rms_per_state': rms_per_state,
        'rms_per_state_ss': rms_per_state_ss,
        'rms_positions': rms_positions,
        'rms_positions_ss': rms_positions_ss,
        'rms_e_x1': rms_e_x1,
        'rms_e_x6': rms_e_x6,
        'rms_e_x1_ss': rms_e_x1_ss,
        'rms_e_x6_ss': rms_e_x6_ss,
        'rms_output_y1': rms_output_y1,
        'rms_output_y6': rms_output_y6,
        'rms_output_y1_ss': rms_output_y1_ss,
        'rms_output_y6_ss': rms_output_y6_ss,
        'rms_output_meas_y1': rms_output_meas_y1,
        'rms_output_meas_y6': rms_output_meas_y6,
        'rms_output_meas_y1_ss': rms_output_meas_y1_ss,
        'rms_output_meas_y6_ss': rms_output_meas_y6_ss,
        'error_norm': error_norm,
        'ss_start_index': ss_start_index,
        'ss_fraction': steady_state_window
    }


if __name__ == '__main__':
    """
    Main runner: Load model, design Kalman filter, run stochastic simulation,
    compute metrics, generate plots and save artifacts.
    """
    print("="*60)
    print("Part 5: Discrete-Time Steady-State Kalman Filter (LQE)")
    print("="*60)
    
    # Load model
    print("\n1. Loading model from Part 0 utilities...")
    A, B, C = build_continuous_model()
    Ad, Bd, Cd, Dd = discretize_zoh(A, B, C, Ts=0.01)
    Ts = 0.01
    print(f"   Ad shape: {Ad.shape}, Bd shape: {Bd.shape}")
    
    # Get Part 2 C matrix (measurement matrix)
    print("\n2. Loading Part 2 sensor matrix (measuring x1 and x6)...")
    Cmeas = get_part2_C_matrix()
    print(f"   Cmeas shape: {Cmeas.shape}")
    print(f"   Cmeas = \n{Cmeas}")
    
    # Get Part 2 initial conditions
    print("\n3. Loading Part 2 initial conditions...")
    x0, xhat0 = get_part2_initial_conditions()
    print(f"   x0 = {x0}")
    print(f"   xhat0 = {xhat0}")
    
    # Define noise covariances
    print("\n4. Defining noise covariances...")
    m = Bd.shape[1]  # Number of inputs
    p = Cmeas.shape[0]  # Number of outputs
    
    Qw = 0.05 * np.eye(m)  # Actuator noise: w ~ N(0, 0.05 I_m)
    Rv = 0.1 * np.eye(p)   # Sensor noise: v ~ N(0, 0.1 I_p)
    Qx = Bd @ Qw @ Bd.T    # Process noise covariance: Qx = Bd @ Qw @ Bd.T
    
    print(f"   Qw shape: {Qw.shape} (actuator noise covariance)")
    print(f"   Rv shape: {Rv.shape} (sensor noise covariance)")
    print(f"   Qx shape: {Qx.shape} (process noise covariance)")
    
    # Validate covariances are PSD
    Qw_eigvals = np.linalg.eigvals(Qw)
    Rv_eigvals = np.linalg.eigvals(Rv)
    Qx_eigvals = np.linalg.eigvals(Qx)
    
    Qw_is_psd = np.all(Qw_eigvals >= -1e-10)
    Rv_is_psd = np.all(Rv_eigvals >= -1e-10)
    Qx_is_psd = np.all(Qx_eigvals >= -1e-10)
    
    print(f"   Qw is PSD: {Qw_is_psd} (min eigenvalue: {np.min(Qw_eigvals):.6e})")
    print(f"   Rv is PSD: {Rv_is_psd} (min eigenvalue: {np.min(Rv_eigvals):.6e})")
    print(f"   Qx is PSD: {Qx_is_psd} (min eigenvalue: {np.min(Qx_eigvals):.6e})")
    
    # Validation gate: Covariances must be PSD
    if not (Qw_is_psd and Rv_is_psd and Qx_is_psd):
        print(f"   ERROR: Covariance validation FAILED")
        sys.exit(1)
    
    # Validation gate: Correct shapes
    if Qw.shape != (3, 3) or Rv.shape != (2, 2) or Qx.shape != (12, 12):
        print(f"   ERROR: Covariance shape validation FAILED")
        sys.exit(1)
    
    print("   Noise covariances validated ✓")
    
    # Design Kalman filter
    print("\n5. Designing Kalman filter (solving DARE)...")
    try:
        Lk, P, kalman_info = design_kalman_filter(Ad, Cmeas, Qx, Rv)
        print(f"   Kalman gain Lk shape: {kalman_info['Lk_shape']}")
        print(f"   Innovation covariance S condition number: {kalman_info['S_condition_number']:.2e}")
        print(f"   Estimator spectral radius: {kalman_info['spectral_radius']:.6f}")
        print(f"   Estimator stable: {kalman_info['is_stable']}")
        
        # Validation gate: Estimator must be stable
        if not kalman_info['is_stable']:
            print(f"   ERROR: Estimator validation FAILED - spectral radius >= 1.0")
            sys.exit(1)
        
        print("   Kalman filter design validated ✓")
    except Exception as e:
        print(f"   ERROR: Kalman filter design failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    
    # Simulation parameters
    print("\n6. Setting up stochastic simulation...")
    N = 1000  # Frozen invariant
    seed = 42  # Reproducibility
    
    # Input: zero input (open-loop)
    u = np.zeros((m, N))
    print(f"   Simulation horizon: N = {N} steps ({N * Ts:.1f} seconds)")
    print(f"   Random seed: {seed}")
    print(f"   Input signal: u[k] = 0 (zero input, open-loop)")
    
    # Run stochastic simulation
    print("\n7. Running stochastic simulation with Kalman filter...")
    results = simulate_stochastic_system_with_kalman(
        Ad, Bd, Cmeas, Lk, x0, xhat0, u, N, Ts, Qw, Rv, seed=seed
    )
    
    # Validation gate: Array shapes
    assert results['x'].shape == (12, N + 1), f"x shape mismatch: {results['x'].shape}"
    assert results['xhat'].shape == (12, N + 1), f"xhat shape mismatch: {results['xhat'].shape}"
    assert results['u'].shape == (3, N), f"u shape mismatch: {results['u'].shape}"
    assert results['y_true'].shape == (2, N + 1), f"y_true shape mismatch: {results['y_true'].shape}"
    assert results['y_meas'].shape == (2, N + 1), f"y_meas shape mismatch: {results['y_meas'].shape}"
    assert results['yhat'].shape == (2, N + 1), f"yhat shape mismatch: {results['yhat'].shape}"
    assert results['w'].shape == (3, N), f"w shape mismatch: {results['w'].shape}"
    assert results['v'].shape == (2, N + 1), f"v shape mismatch: {results['v'].shape}"
    assert results['innovations'].shape == (2, N), f"innovations shape mismatch: {results['innovations'].shape}"
    print("   Array dimensions validated ✓")
    
    # Compute innovation consistency check
    print("\n8. Computing innovation consistency check...")
    innovations = results['innovations']
    S_theoretical = kalman_info['S']  # Cmeas @ P @ Cmeas.T + Rv
    
    # Sample covariance of innovations
    innovations_mean = np.mean(innovations, axis=1, keepdims=True)
    innovations_centered = innovations - innovations_mean
    S_sample = (innovations_centered @ innovations_centered.T) / (N - 1)
    
    # Normalized Innovation Squared (NIS) for each time step
    NIS = np.zeros(N)
    S_inv = np.linalg.inv(S_theoretical)
    for k in range(N):
        r_k = innovations[:, k]
        NIS[k] = r_k.T @ S_inv @ r_k
    
    NIS_mean = np.mean(NIS)
    NIS_expected = p  # Expected NIS for p-dimensional output
    
    print(f"   Innovation covariance (theoretical) S shape: {S_theoretical.shape}")
    print(f"   Innovation covariance (sample) shape: {S_sample.shape}")
    print(f"   S theoretical = \n{S_theoretical}")
    print(f"   S sample = \n{S_sample}")
    print(f"   Mean NIS: {NIS_mean:.6f} (expected: {NIS_expected:.2f} for {p}-dimensional output)")
    print(f"   NIS ratio (mean/expected): {NIS_mean / NIS_expected:.4f}")
    
    # Compute metrics
    print("\n9. Computing RMS metrics...")
    rms_metrics = compute_rms_metrics(
        results['x'], results['xhat'], results['y_true'], results['y_meas'], results['yhat']
    )
    
    print(f"   Overall RMS estimation error (full window): {rms_metrics['rms_overall']:.6e}")
    print(f"   Overall RMS estimation error (steady-state, last {rms_metrics['ss_fraction']*100:.0f}%): {rms_metrics['rms_overall_ss']:.6e}")
    print(f"   RMS error e_x1 (full): {rms_metrics['rms_e_x1']:.6e}")
    print(f"   RMS error e_x1 (steady-state): {rms_metrics['rms_e_x1_ss']:.6e}")
    print(f"   RMS error e_x6 (full): {rms_metrics['rms_e_x6']:.6e}")
    print(f"   RMS error e_x6 (steady-state): {rms_metrics['rms_e_x6_ss']:.6e}")
    print(f"   RMS output tracking error y1 (true vs estimated, full): {rms_metrics['rms_output_y1']:.6e}")
    print(f"   RMS output tracking error y1 (steady-state): {rms_metrics['rms_output_y1_ss']:.6e}")
    print(f"   RMS output tracking error y6 (true vs estimated, full): {rms_metrics['rms_output_y6']:.6e}")
    print(f"   RMS output tracking error y6 (steady-state): {rms_metrics['rms_output_y6_ss']:.6e}")
    print(f"   RMS innovation (y_meas vs yhat) y1 (full): {rms_metrics['rms_output_meas_y1']:.6e}")
    print(f"   RMS innovation (y_meas vs yhat) y1 (steady-state): {rms_metrics['rms_output_meas_y1_ss']:.6e}")
    print(f"   RMS innovation (y_meas vs yhat) y6 (full): {rms_metrics['rms_output_meas_y6']:.6e}")
    print(f"   RMS innovation (y_meas vs yhat) y6 (steady-state): {rms_metrics['rms_output_meas_y6_ss']:.6e}")
    print(f"   Per-state RMS (positions x1..x6, full window):")
    for i in range(6):
        print(f"     x{i+1}: {rms_metrics['rms_per_state'][i]:.6e}")
    
    # Create output directory
    output_dir = os.path.join(os.path.dirname(__file__), 'outputs')
    os.makedirs(output_dir, exist_ok=True)
    
    # Generate plots
    print("\n9. Generating plots...")
    
    # Plot 1: True vs Estimated (y_true and yhat), with y_meas shown faintly
    fig1, ax1 = plt.subplots(2, 1, figsize=(10, 8))
    
    ax1[0].plot(results['t'], results['y_true'][0, :], 'g-', label='y_true (true output, no noise)', linewidth=2, alpha=0.8)
    ax1[0].plot(results['t'], results['yhat'][0, :], 'r--', label='yhat (estimated output)', linewidth=2)
    ax1[0].plot(results['t'], results['y_meas'][0, :], 'b-', label='y_meas (noisy measurement)', linewidth=1, alpha=0.3)
    ax1[0].set_xlabel('Time (s)')
    ax1[0].set_ylabel('Displacement')
    ax1[0].set_title('Output 1: Displacement of Mass 1 (x1) - True vs Estimated')
    ax1[0].legend()
    ax1[0].grid(True)
    
    ax1[1].plot(results['t'], results['y_true'][1, :], 'g-', label='y_true (true output, no noise)', linewidth=2, alpha=0.8)
    ax1[1].plot(results['t'], results['yhat'][1, :], 'r--', label='yhat (estimated output)', linewidth=2)
    ax1[1].plot(results['t'], results['y_meas'][1, :], 'b-', label='y_meas (noisy measurement)', linewidth=1, alpha=0.3)
    ax1[1].set_xlabel('Time (s)')
    ax1[1].set_ylabel('Displacement')
    ax1[1].set_title('Output 2: Displacement of Mass 6 (x6) - True vs Estimated')
    ax1[1].legend()
    ax1[1].grid(True)
    
    plt.tight_layout()
    outputs_file = os.path.join(output_dir, 'outputs_y_vs_yhat.png')
    plt.savefig(outputs_file, dpi=150)
    print(f"   Saved: {outputs_file}")
    plt.close()
    
    # Plot 2: Estimation error norm
    fig2, ax2 = plt.subplots(figsize=(10, 6))
    ax2.plot(results['t'], rms_metrics['error_norm'], 'g-', label='||x - xhat||', linewidth=2)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Estimation Error Norm')
    ax2.set_title('Estimation Error Norm: ||x - xhat||')
    ax2.legend()
    ax2.grid(True)
    
    plt.tight_layout()
    error_file = os.path.join(output_dir, 'estimation_error_norm.png')
    plt.savefig(error_file, dpi=150)
    print(f"   Saved: {error_file}")
    plt.close()
    
    # Plot 3: Estimation errors for x1 and x6
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
    error_x1_x6_file = os.path.join(output_dir, 'estimation_error_x1_x6.png')
    plt.savefig(error_x1_x6_file, dpi=150)
    print(f"   Saved: {error_x1_x6_file}")
    plt.close()
    
    # Plot 4: Per-state RMS (bar chart, optional)
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
    
    per_state_file = os.path.join(output_dir, 'per_state_rms_bar.png')
    plt.savefig(per_state_file, dpi=150)
    print(f"   Saved: {per_state_file}")
    plt.close()
    
    # Get version information for reproducibility
    python_version = sys.version.split()[0]
    numpy_version = np.__version__
    scipy_version = None
    try:
        import scipy
        scipy_version = scipy.__version__
    except:
        pass
    
    # Save results to text file
    print("\n10. Saving results to file...")
    results_file = os.path.join(output_dir, 'results.txt')
    with open(results_file, 'w') as f:
        f.write("Part 5: Discrete-Time Steady-State Kalman Filter (LQE) - Results\n")
        f.write("="*60 + "\n\n")
        
        f.write("Reproducibility Information:\n")
        f.write(f"  Python version: {python_version}\n")
        f.write(f"  NumPy version: {numpy_version}\n")
        if scipy_version:
            f.write(f"  SciPy version: {scipy_version}\n")
        f.write(f"  Platform: {platform.platform()}\n")
        f.write(f"  Random seed: {seed}\n\n")
        
        f.write("Simulation Parameters:\n")
        f.write(f"  N (number of input samples) = {N}\n")
        f.write(f"  Ts (sampling time) = {Ts} s\n")
        f.write(f"  Time span = {N * Ts:.1f} s\n")
        f.write(f"  Array dimensions (standard convention):\n")
        f.write(f"    - x: (12, N+1) stores x[0] through x[N]\n")
        f.write(f"    - xhat: (12, N+1) stores xhat[0] through xhat[N]\n")
        f.write(f"    - u: (3, N) stores u[0] through u[N-1]\n")
        f.write(f"    - y_true: (2, N+1) stores y_true[0] through y_true[N] (true output, Cmeas @ x, no noise)\n")
        f.write(f"    - y_meas: (2, N+1) stores y_meas[0] through y_meas[N] (measured output, Cmeas @ x + v, with noise)\n")
        f.write(f"    - yhat: (2, N+1) stores yhat[0] through yhat[N] (estimated output, Cmeas @ xhat)\n")
        f.write(f"    - w: (3, N) stores w[0] through w[N-1]\n")
        f.write(f"    - v: (2, N+1) stores v[0] through v[N]\n")
        f.write(f"    - innovations: (2, N) stores innovations[0] through innovations[N-1] (y_meas - yhat)\n\n")
        
        f.write("Initial Conditions:\n")
        f.write(f"  x0 (actual) = {x0}\n")
        f.write(f"  xhat0 (estimator) = {xhat0}\n")
        f.write(f"  Exact values for reproducibility:\n")
        f.write(f"    x0 = [{', '.join([f'{val:.15e}' for val in x0])}]\n")
        f.write(f"    xhat0 = [{', '.join([f'{val:.15e}' for val in xhat0])}]\n\n")
        
        f.write("Measurement Matrix:\n")
        f.write(f"  Cmeas shape: {Cmeas.shape}\n")
        f.write(f"  Cmeas = \n{Cmeas}\n")
        f.write(f"  Cmeas measures x1 and x6 (Part 2 sensor configuration)\n\n")
        
        f.write("Output Definitions (Metrics):\n")
        f.write(f"  y_true[k] = Cmeas @ x[k] (true output, no noise)\n")
        f.write(f"  y_meas[k] = Cmeas @ x[k] + v[k] (measured output, with noise)\n")
        f.write(f"  yhat[k] = Cmeas @ xhat[k] (estimated output from Kalman filter)\n")
        f.write(f"  Note: For plots, 'True vs Estimated' shows y_true and yhat.\n")
        f.write(f"        'Measurement vs Estimate' shows y_meas and yhat.\n")
        f.write(f"        y_meas is the noisy measurement, not the true output.\n\n")
        
        f.write("Noise Covariances:\n")
        f.write(f"  Qw (actuator noise) shape: {Qw.shape}\n")
        f.write(f"  Qw = 0.05 * I_3\n")
        f.write(f"  Qw eigenvalues: {Qw_eigvals}\n")
        f.write(f"  Qw is PSD: {Qw_is_psd}\n\n")
        
        f.write(f"  Rv (sensor noise) shape: {Rv.shape}\n")
        f.write(f"  Rv = 0.1 * I_2\n")
        f.write(f"  Rv eigenvalues: {Rv_eigvals}\n")
        f.write(f"  Rv is PSD: {Rv_is_psd}\n\n")
        
        f.write(f"  Qx (process noise) shape: {Qx.shape}\n")
        f.write(f"  Qx = Bd @ Qw @ Bd.T\n")
        f.write(f"  Qx minimum eigenvalue: {np.min(Qx_eigvals):.6e}\n")
        f.write(f"  Qx maximum eigenvalue: {np.max(Qx_eigvals):.6e}\n")
        f.write(f"  Qx is PSD: {Qx_is_psd}\n\n")
        
        f.write("Kalman Filter Design:\n")
        f.write(f"  Method: Steady-state Kalman filter via DARE\n")
        f.write(f"  DARE: P = solve_discrete_are(Ad.T, Cmeas.T, Qx, Rv)\n")
        f.write(f"  Kalman gain: Lk = P @ Cmeas.T @ inv(Cmeas @ P @ Cmeas.T + Rv)\n")
        f.write(f"  Lk shape: {kalman_info['Lk_shape']}\n")
        f.write(f"  Innovation covariance S shape: {kalman_info['S'].shape}\n")
        f.write(f"  S condition number: {kalman_info['S_condition_number']:.2e}\n")
        f.write(f"  S determinant: {kalman_info['S_determinant']:.6e}\n")
        f.write(f"  Estimator closed-loop matrix: Aest = Ad - Lk @ Cmeas\n")
        f.write(f"  Estimator spectral radius: {kalman_info['spectral_radius']:.6f}\n")
        f.write(f"  Estimator stable: {kalman_info['is_stable']}\n")
        f.write(f"  Estimator eigenvalues:\n")
        for i, eig in enumerate(kalman_info['eigenvalues']):
            f.write(f"    λ_{i+1} = {eig:.6f} (|λ| = {np.abs(eig):.6f})\n")
        f.write(f"\n  Kalman gain Lk (full matrix, 12×2):\n")
        for i in range(Lk.shape[0]):
            f.write(f"    Row {i+1}: [{Lk[i, 0]:.6e}, {Lk[i, 1]:.6e}]\n")
        f.write("\n")
        
        f.write("Innovation Consistency Check:\n")
        f.write(f"  Innovation covariance (theoretical) S = Cmeas @ P @ Cmeas.T + Rv:\n")
        for i in range(S_theoretical.shape[0]):
            f.write(f"    Row {i+1}: {S_theoretical[i, :]}\n")
        f.write(f"  Innovation covariance (sample) from data:\n")
        for i in range(S_sample.shape[0]):
            f.write(f"    Row {i+1}: {S_sample[i, :]}\n")
        f.write(f"  Mean NIS (Normalized Innovation Squared): {NIS_mean:.6f}\n")
        f.write(f"  Expected NIS (for {p}-dimensional output): {NIS_expected:.2f}\n")
        f.write(f"  NIS ratio (mean/expected): {NIS_mean / NIS_expected:.4f}\n\n")
        
        f.write("RMS Metrics:\n")
        f.write(f"  Overall RMS estimation error (full window): {rms_metrics['rms_overall']:.6e}\n")
        f.write(f"  Overall RMS estimation error (steady-state, last {rms_metrics['ss_fraction']*100:.0f}%): {rms_metrics['rms_overall_ss']:.6e}\n")
        f.write(f"  Steady-state window: indices {rms_metrics['ss_start_index']} to {N} (last {rms_metrics['ss_fraction']*100:.0f}%)\n\n")
        
        f.write(f"  Per-state RMS estimation error (full window):\n")
        for i in range(12):
            f.write(f"    x{i+1}: {rms_metrics['rms_per_state'][i]:.6e}\n")
        f.write(f"\n  Per-state RMS estimation error (steady-state):\n")
        for i in range(12):
            f.write(f"    x{i+1}: {rms_metrics['rms_per_state_ss'][i]:.6e}\n")
        
        f.write(f"\n  Position RMS (x1..x6, full window):\n")
        for i in range(6):
            f.write(f"    x{i+1}: {rms_metrics['rms_positions'][i]:.6e}\n")
        f.write(f"\n  Position RMS (x1..x6, steady-state):\n")
        for i in range(6):
            f.write(f"    x{i+1}: {rms_metrics['rms_positions_ss'][i]:.6e}\n")
        
        f.write(f"\n  Estimation error e_x1 (full window): {rms_metrics['rms_e_x1']:.6e}\n")
        f.write(f"  Estimation error e_x1 (steady-state): {rms_metrics['rms_e_x1_ss']:.6e}\n")
        f.write(f"  Estimation error e_x6 (full window): {rms_metrics['rms_e_x6']:.6e}\n")
        f.write(f"  Estimation error e_x6 (steady-state): {rms_metrics['rms_e_x6_ss']:.6e}\n\n")
        
        f.write(f"  Output tracking RMS (y_true vs yhat, full window):\n")
        f.write(f"    y1: {rms_metrics['rms_output_y1']:.6e}\n")
        f.write(f"    y6: {rms_metrics['rms_output_y6']:.6e}\n")
        f.write(f"\n  Output tracking RMS (y_true vs yhat, steady-state):\n")
        f.write(f"    y1: {rms_metrics['rms_output_y1_ss']:.6e}\n")
        f.write(f"    y6: {rms_metrics['rms_output_y6_ss']:.6e}\n")
        f.write(f"\n  Innovation RMS (y_meas vs yhat, full window):\n")
        f.write(f"    y1: {rms_metrics['rms_output_meas_y1']:.6e}\n")
        f.write(f"    y6: {rms_metrics['rms_output_meas_y6']:.6e}\n")
        f.write(f"\n  Innovation RMS (y_meas vs yhat, steady-state):\n")
        f.write(f"    y1: {rms_metrics['rms_output_meas_y1_ss']:.6e}\n")
        f.write(f"    y6: {rms_metrics['rms_output_meas_y6_ss']:.6e}\n")
        f.write(f"\n  Note: Tracking RMS (y_true vs yhat) measures how well filter tracks true output.\n")
        f.write(f"        Innovation RMS (y_meas vs yhat) measures measurement residual.\n\n")
        
        f.write("Reproducibility - Noise Samples:\n")
        f.write(f"  First 3 samples of w (process noise):\n")
        for i, w_sample in enumerate(results['noise_samples']['w']):
            f.write(f"    w[{i}] = [{', '.join([f'{val:.15e}' for val in w_sample])}]\n")
        f.write(f"  First 3 samples of v (measurement noise):\n")
        for i, v_sample in enumerate(results['noise_samples']['v']):
            f.write(f"    v[{i}] = [{', '.join([f'{val:.15e}' for val in v_sample])}]\n")
        f.write("\n")
        
        f.write("Source Citations:\n")
        f.write(f"  Part 5 requirement: docs/sources/final_exam_extract.md Section 7\n")
        f.write(f"  Part 2 C matrix and initial conditions: docs/sources/final_exam_extract.md Section 4\n")
        f.write(f"  Frozen invariants: docs/07_comprehensive_review_part0_to_part4/cross_part_invariants.md\n")
    
    print(f"   Saved: {results_file}")
    
    # Save Kalman gain matrix
    print("\n11. Saving Kalman gain matrix...")
    Lk_file = os.path.join(output_dir, 'Lk_matrix.npy')
    np.save(Lk_file, Lk)
    print(f"   Saved: {Lk_file}")
    
    # Save trajectories
    print("\n12. Saving trajectories...")
    traj_file = os.path.join(output_dir, 'traj.npz')
    np.savez(traj_file,
             x=results['x'],          # (12, N+1) true state trajectory
             xhat=results['xhat'],    # (12, N+1) estimated state trajectory
             y_true=results['y_true'],  # (2, N+1) true output trajectory (Cmeas @ x, no noise)
             y_meas=results['y_meas'],  # (2, N+1) measured output trajectory (Cmeas @ x + v, with noise)
             yhat=results['yhat'],    # (2, N+1) estimated output trajectory (Cmeas @ xhat)
             u=results['u'],          # (3, N) input trajectory (all zeros)
             w=results['w'],          # (3, N) process noise samples
             v=results['v'],          # (2, N+1) measurement noise samples
             innovations=results['innovations'],  # (2, N) innovation sequence (y_meas - yhat)
             t=results['t'])          # (N+1,) time vector
    print(f"   Saved: {traj_file}")
    print(f"   Trajectory shapes documented in results.txt\n")
    
    print("="*60)
    print("Part 5 simulation complete!")
    print("="*60)
    print(f"\nOutput files saved to: {output_dir}/")
    print("  - results.txt")
    print("  - Lk_matrix.npy")
    print("  - traj.npz")
    print("  - outputs_y_vs_yhat.png")
    print("  - estimation_error_norm.png")
    print("  - estimation_error_x1_x6.png")
    print("  - per_state_rms_bar.png")
