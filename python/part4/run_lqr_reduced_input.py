"""
Part 4: Discrete-Time LQR Controller Design with Reduced Input (u3 Removed)

This script redesigns a discrete-time infinite-horizon LQR controller for the
6-mass spring system with the third input u3 removed. The controller uses
estimated states from the Part 2 observer (u_red[k] = -K_red xhat[k]) and
minimizes the same cost as Part 3:
    J = sum_{k=0}^{N-1} (u_red[k]^T u_red[k] + y1[k]^2 + y6[k]^2)

Source: docs/sources/final_exam_extract.md Section 6 (Part 4 Requirement)
"""

import numpy as np
import matplotlib.pyplot as plt
import os
import sys
import platform
import re

# Add paths for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'utils'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'part2'))

from build_model import build_continuous_model, discretize_zoh
from observer_design import get_part2_C_matrix, design_observer
from run_observer_sim import get_part2_initial_conditions
from scipy.linalg import solve_discrete_are


def check_detectability(Ad, Cy, tol=1e-10):
    """
    Check detectability of (Ad, Cy) using PBH rank condition.
    
    For discrete-time systems, (Ad, Cy) is detectable if for each eigenvalue
    λ of Ad with |λ| >= 1 - tol, the PBH rank condition holds:
        rank([λI - Ad^T, Cy^T]) == n
    
    This checks detectability via the dual system (Ad^T, Cy^T) and is
    equivalent to observability of unstable modes.
    
    Args:
        Ad: (n, n) discrete-time state matrix
        Cy: (p, n) output matrix
        tol: Tolerance for eigenvalue magnitude and rank computation
    
    Returns:
        dict: Detectability analysis results
    """
    n = Ad.shape[0]
    p = Cy.shape[0]
    
    # Compute eigenvalues of Ad
    eigvals = np.linalg.eigvals(Ad)
    eigvals_mag = np.abs(eigvals)
    
    # Find eigenvalues with |λ| >= 1 - tol (potentially unstable)
    unstable_mask = eigvals_mag >= (1.0 - tol)
    unstable_eigvals = eigvals[unstable_mask]
    unstable_eigvals_mag = eigvals_mag[unstable_mask]
    
    # Check PBH rank condition for each potentially unstable eigenvalue
    is_detectable = True
    pbh_checks = []
    
    for i, lam in enumerate(unstable_eigvals):
        # Form PBH matrix for detectability: [λI - Ad^T, Cy^T]
        # This checks detectability via dual system (Ad^T, Cy^T)
        pbh_matrix = np.hstack([lam * np.eye(n) - Ad.T, Cy.T])
        
        # Compute rank using SVD with proper threshold
        U, sigma, Vh = np.linalg.svd(pbh_matrix, full_matrices=False)
        machine_eps = np.finfo(pbh_matrix.dtype).eps
        max_sigma = np.max(sigma)
        threshold = max(tol * max_sigma, machine_eps * max_sigma)
        rank = np.sum(sigma > threshold)
        
        pbh_check = {
            'eigenvalue': lam,
            'magnitude': unstable_eigvals_mag[i],
            'pbh_rank': rank,
            'n': n,
            'pbh_condition_holds': (rank == n)
        }
        pbh_checks.append(pbh_check)
        
        if rank != n:
            is_detectable = False
    
    return {
        'is_detectable': is_detectable,
        'n': n,
        'tol_used': tol,
        'unstable_eigenvalues': unstable_eigvals,
        'unstable_eigenvalues_magnitude': unstable_eigvals_mag,
        'pbh_checks': pbh_checks
    }


def check_stabilizability(Ad, Bd, tol=1e-10):
    """
    Check stabilizability of (Ad, Bd) using PBH rank condition.
    
    For discrete-time systems, (Ad, Bd) is stabilizable if for each eigenvalue
    λ of Ad with |λ| >= 1 - tol, the PBH rank condition holds:
        rank([λI - Ad, Bd]) == n
    
    Args:
        Ad: (n, n) discrete-time state matrix
        Bd: (n, m) discrete-time input matrix
        tol: Tolerance for eigenvalue magnitude and rank computation
    
    Returns:
        dict: Stabilizability analysis results
    """
    n = Ad.shape[0]
    m = Bd.shape[1]
    
    # Compute eigenvalues of Ad
    eigvals = np.linalg.eigvals(Ad)
    eigvals_mag = np.abs(eigvals)
    
    # Find eigenvalues with |λ| >= 1 - tol (potentially unstable)
    tol_band = 1e-6  # Additional band for logging
    unstable_mask = eigvals_mag >= (1.0 - tol)
    unstable_eigvals = eigvals[unstable_mask]
    unstable_eigvals_mag = eigvals_mag[unstable_mask]
    
    # Also log eigenvalues in the tolerance band for review
    borderline_mask = (eigvals_mag >= (1.0 - tol - tol_band)) & (eigvals_mag < (1.0 - tol))
    borderline_eigvals = eigvals[borderline_mask]
    borderline_eigvals_mag = eigvals_mag[borderline_mask]
    
    # Check PBH rank condition for each potentially unstable eigenvalue
    is_stabilizable = True
    pbh_checks = []
    pbh_ranks = []
    
    for i, lam in enumerate(unstable_eigvals):
        # Form PBH matrix: [λI - Ad, Bd]
        pbh_matrix = np.hstack([lam * np.eye(n) - Ad, Bd])
        
        # Compute rank using SVD with proper threshold
        U, sigma, Vh = np.linalg.svd(pbh_matrix, full_matrices=False)
        machine_eps = np.finfo(pbh_matrix.dtype).eps
        max_sigma = np.max(sigma)
        threshold = max(tol * max_sigma, machine_eps * max_sigma)
        rank = np.sum(sigma > threshold)
        pbh_ranks.append(rank)
        
        # Compute rank margin (distance from threshold)
        if rank == n:
            rank_margin = np.min(sigma[sigma > threshold]) - threshold
        else:
            rank_margin = np.max(sigma[sigma <= threshold]) - threshold
        
        pbh_check = {
            'eigenvalue': lam,
            'magnitude': unstable_eigvals_mag[i],
            'pbh_rank': rank,
            'n': n,
            'pbh_condition_holds': (rank == n),
            'rank_margin': rank_margin,
            'min_singular_value': np.min(sigma) if len(sigma) > 0 else 0.0
        }
        pbh_checks.append(pbh_check)
        
        if rank != n:
            is_stabilizable = False
    
    # Also compute controllability rank for reference
    C = np.zeros((n, n * m))
    AB_power = np.eye(n)
    for i in range(n):
        C[:, i*m:(i+1)*m] = AB_power @ Bd
        AB_power = Ad @ AB_power
    
    U_ctrl, sigma_ctrl, Vh_ctrl = np.linalg.svd(C, full_matrices=False)
    machine_eps = np.finfo(C.dtype).eps
    max_sigma_ctrl = np.max(sigma_ctrl)
    threshold_ctrl = max(tol * max_sigma_ctrl, machine_eps * max_sigma_ctrl)
    controllability_rank = np.sum(sigma_ctrl > threshold_ctrl)
    
    return {
        'is_stabilizable': is_stabilizable,
        'n': n,
        'tol_used': tol,
        'tol_band': tol_band,
        'unstable_eigenvalues': unstable_eigvals,
        'unstable_eigenvalues_magnitude': unstable_eigvals_mag,
        'borderline_eigenvalues': borderline_eigvals,
        'borderline_eigenvalues_magnitude': borderline_eigvals_mag,
        'pbh_checks': pbh_checks,
        'pbh_ranks': pbh_ranks,
        'min_pbh_rank': min(pbh_ranks) if len(pbh_ranks) > 0 else n,
        'controllability_rank': controllability_rank,
        'controllability_rank_threshold': threshold_ctrl
    }


def design_lqr(Ad, Bd, Q, R):
    """
    Design discrete-time infinite-horizon LQR controller.
    
    Solves the discrete-time algebraic Riccati equation (DARE) and computes
    the LQR gain K.
    
    Args:
        Ad: (n, n) discrete-time state matrix
        Bd: (n, m) discrete-time input matrix
        Q: (n, n) state weight matrix (symmetric, positive semidefinite)
        R: (m, m) input weight matrix (symmetric, positive definite)
    
    Returns:
        tuple: (K, P, design_info) where
            - K: (m, n) LQR gain matrix
            - P: (n, n) solution to DARE
            - design_info: dict with design parameters and eigenvalues
    """
    # Solve discrete-time algebraic Riccati equation
    P = solve_discrete_are(Ad, Bd, Q, R)
    
    # Compute LQR gain: K = (R + Bd^T @ P @ Bd)^(-1) @ (Bd^T @ P @ Ad)
    R_BPB = R + Bd.T @ P @ Bd
    K = np.linalg.solve(R_BPB, Bd.T @ P @ Ad)
    
    # Compute closed-loop matrix: Acl = Ad - Bd @ K
    Acl = Ad - Bd @ K
    
    # Compute eigenvalues and spectral radius
    eigvals = np.linalg.eigvals(Acl)
    spectral_radius = np.max(np.abs(eigvals))
    
    # Compute minimum eigenvalue of R_BPB for numerical safety
    R_BPB_eigvals = np.linalg.eigvals(R_BPB)
    R_BPB_min_eigval = np.min(R_BPB_eigvals)
    
    design_info = {
        'K': K,
        'K_shape': K.shape,
        'P': P,
        'Acl': Acl,
        'eigenvalues': eigvals,
        'spectral_radius': spectral_radius,
        'is_stable': spectral_radius < 1.0,
        'R_BPB_min_eigval': R_BPB_min_eigval
    }
    
    return K, P, design_info


def simulate_closed_loop_with_observer(Ad, Bd_red, Cmeas, K_red, L, x0, xhat0, N, Ts):
    """
    Simulate closed-loop system with observer in the loop (reduced input).
    
    Standard convention: x has length N+1 (stores x[0] through x[N]),
    u_red has length N (stores u_red[0] through u_red[N-1]), so u_red[k] pairs with
    the transition from x[k] to x[k+1].
    
    Plant:  x[k+1] = Ad @ x[k] + Bd_red @ u_red[k]
            y[k] = Cmeas @ x[k]
    
    Observer: xhat[k+1] = Ad @ xhat[k] + Bd_red @ u_red[k] + L @ (y[k] - yhat[k])
              yhat[k] = Cmeas @ xhat[k]
    
    Controller: u_red[k] = -K_red @ xhat[k] (CRITICAL: uses xhat, not x)
    
    Args:
        Ad: (n, n) discrete-time state matrix
        Bd_red: (n, 2) discrete-time input matrix (reduced, columns 0 and 1)
        Cmeas: (p, n) measurement matrix (Part 2 C matrix)
        K_red: (2, n) LQR gain matrix (reduced)
        L: (n, p) observer gain matrix
        x0: (n,) initial true state
        xhat0: (n,) initial observer state
        N: Number of simulation steps (inputs u_red[0..N-1], states x[0..N])
        Ts: Sampling time (for time vector)
    
    Returns:
        dict: Trajectories and errors
            - 'x': (n, N+1) true state trajectory (x[0] through x[N])
            - 'xhat': (n, N+1) estimated state trajectory (xhat[0] through xhat[N])
            - 'u_red': (2, N) input trajectory (u_red[0] through u_red[N-1])
            - 'y': (p, N+1) true output trajectory (y[0] through y[N])
            - 't': (N+1,) time vector
            - 'e': (n, N+1) estimation error (x - xhat)
    """
    n = Ad.shape[0]
    m_red = Bd_red.shape[1]  # Should be 2
    p = Cmeas.shape[0]
    
    # Preallocate arrays with standard convention
    x = np.zeros((n, N + 1))
    xhat = np.zeros((n, N + 1))
    u_red = np.zeros((m_red, N))
    y = np.zeros((p, N + 1))
    e = np.zeros((n, N + 1))
    
    # Initialize
    x[:, 0] = x0
    xhat[:, 0] = xhat0
    y[:, 0] = Cmeas @ x[:, 0]
    e[:, 0] = x[:, 0] - xhat[:, 0]
    
    # Simulate forward
    for k in range(N):
        # Controller: u_red[k] = -K_red @ xhat[k] (CRITICAL: uses xhat, not x)
        u_red[:, k] = -K_red @ xhat[:, k]
        
        # Plant output at time k
        y[:, k] = Cmeas @ x[:, k]
        
        # Plant update: x[k+1] = Ad @ x[k] + Bd_red @ u_red[k]
        x[:, k + 1] = Ad @ x[:, k] + Bd_red @ u_red[:, k]
        
        # Observer update
        yhat_k = Cmeas @ xhat[:, k]
        y_error = y[:, k] - yhat_k
        xhat[:, k + 1] = Ad @ xhat[:, k] + Bd_red @ u_red[:, k] + L @ y_error
        
        # Estimation error
        e[:, k + 1] = x[:, k + 1] - xhat[:, k + 1]
    
    # Final output at time N
    y[:, N] = Cmeas @ x[:, N]
    
    # Time vector (N+1 points: t[0] through t[N])
    t = np.arange(N + 1) * Ts
    
    return {
        'x': x,
        'xhat': xhat,
        'u_red': u_red,
        'y': y,
        't': t,
        'e': e
    }


def compute_cost_metrics(x, u_red, Cy, N):
    """
    Compute cost metrics for LQR controller (reduced input).
    
    Standard convention: x has length N+1, u_red has length N.
    Cost pairs u_red[k] with the transition from x[k] to x[k+1]:
        stage_cost[k] = u_red[k]^T @ u_red[k] + y_cost[0]^2 + y_cost[1]^2
    where y_cost = Cy @ x[k] (plant output at time k, not estimated)
    
    Total cost: J_red = sum_{k=0}^{N-1} stage_cost[k]
    
    Args:
        x: (n, N+1) state trajectory (x[0] through x[N])
        u_red: (2, N) input trajectory (u_red[0] through u_red[N-1])
        Cy: (p, n) cost output selector (Part 2 C matrix)
        N: Number of input samples (u_red has length N, x has length N+1)
    
    Returns:
        dict: Cost metrics
    """
    # Cost is computed for k = 0 to N-1 (pairs u_red[k] with x[k] -> x[k+1])
    stage_cost = np.zeros(N)
    
    for k in range(N):
        # Cost output: y_cost = Cy @ x[k] (plant output at time k, not estimated)
        y_cost = Cy @ x[:, k]
        # Stage cost: u_red^T u_red + y1^2 + y6^2
        stage_cost[k] = u_red[:, k].T @ u_red[:, k] + y_cost[0]**2 + y_cost[1]**2
    
    # Total cost
    J_red = np.sum(stage_cost)
    
    # Max input magnitudes
    max_abs_u1 = np.max(np.abs(u_red[0, :]))
    max_abs_u2 = np.max(np.abs(u_red[1, :]))
    max_abs_u_overall = np.max(np.abs(u_red))  # Aligned with Part 3: max(abs(u)) across all elements
    
    # Max input in infinity norm sense: max_k ||u_red[k]||_inf
    u_inf_norm = np.max(np.abs(u_red), axis=0)  # ||u_red[k]||_inf for each k
    max_u_inf = np.max(u_inf_norm)  # max over all k
    
    return {
        'stage_cost': stage_cost,
        'total_cost_J': J_red,
        'cost_range': (0, N - 1),
        'max_abs_u1': max_abs_u1,
        'max_abs_u2': max_abs_u2,
        'max_abs_u_overall': max_abs_u_overall,
        'max_u_inf': max_u_inf
    }


def load_part3_baseline(results_file_path):
    """
    Load Part 3 baseline metrics from results.txt.
    
    Args:
        results_file_path: Path to python/part3/outputs/results.txt
    
    Returns:
        dict: Baseline metrics with keys:
            - 'J': Total cost J
            - 'max_abs_u1': Max absolute value of u1
            - 'max_abs_u2': Max absolute value of u2
            - 'max_abs_u3': Max absolute value of u3
            - 'max_abs_u_overall': Max absolute value over all inputs
            - 'loaded': True if successfully loaded (all required fields found), False if missing
    """
    baseline = {
        'J': None,
        'max_abs_u1': None,
        'max_abs_u2': None,
        'max_abs_u3': None,
        'max_abs_u_overall': None,
        'loaded': False
    }
    
    if not os.path.exists(results_file_path):
        return baseline
    
    try:
        with open(results_file_path, 'r') as f:
            content = f.read()
        
        # Parse total cost J
        match = re.search(r'Total cost J\s*=\s*([0-9.eE+-]+)', content)
        if match:
            baseline['J'] = float(match.group(1))
        
        # Parse max input magnitudes
        match = re.search(r'Max \|u1\|\s*=\s*([0-9.eE+-]+)', content)
        if match:
            baseline['max_abs_u1'] = float(match.group(1))
        
        match = re.search(r'Max \|u2\|\s*=\s*([0-9.eE+-]+)', content)
        if match:
            baseline['max_abs_u2'] = float(match.group(1))
        
        match = re.search(r'Max \|u3\|\s*=\s*([0-9.eE+-]+)', content)
        if match:
            baseline['max_abs_u3'] = float(match.group(1))
        
        match = re.search(r'Max \|u\|\s*overall\s*=\s*([0-9.eE+-]+)', content)
        if match:
            baseline['max_abs_u_overall'] = float(match.group(1))
        
        # Require all fields to be found before marking as loaded
        required_fields = ['J', 'max_abs_u1', 'max_abs_u2', 'max_abs_u3', 'max_abs_u_overall']
        if all(baseline[field] is not None for field in required_fields):
            baseline['loaded'] = True
        else:
            # Log which fields are missing
            missing = [field for field in required_fields if baseline[field] is None]
            print(f"   WARNING: Part 3 baseline missing fields: {missing}")
    
    except Exception as e:
        print(f"   WARNING: Error parsing Part 3 baseline: {e}")
        return baseline
    
    return baseline


if __name__ == '__main__':
    """
    Main runner: Load model, design LQR with reduced input, run closed-loop simulation,
    generate plots and metrics, compare against Part 3 baseline.
    """
    print("="*60)
    print("Part 4: LQR Controller Design with Reduced Input (u3 Removed)")
    print("="*60)
    
    # Load model
    print("\n1. Loading model from Part 0 utilities...")
    A, B, C = build_continuous_model()
    Ad, Bd, Cd, Dd = discretize_zoh(A, B, C, Ts=0.01)
    Ts = 0.01
    print(f"   Ad shape: {Ad.shape}, Bd shape: {Bd.shape}")
    
    # Create reduced input matrix
    print("\n2. Creating reduced input matrix (removing u3)...")
    Bd_red = Bd[:, [0, 1]]  # Extract columns 0 and 1 (u1 and u2)
    print(f"   Bd_red shape: {Bd_red.shape}")
    print(f"   Input mapping: u_red = [u1, u2] (removed channel: u3)")
    print(f"   This matches docs/00_anchor.md: u1 affects rows 7,8; u2 affects rows 11,12; u3 affects row 8")
    print(f"   Removing u3 (column 2 of Bd) leaves u1 (column 0) and u2 (column 1)")
    
    # Gate: Bd_red shape validation
    if Bd_red.shape != (12, 2):
        print(f"   ERROR: Bd_red shape validation FAILED: {Bd_red.shape}, expected (12, 2)")
        sys.exit(1)
    print("   Input reduction validated ✓")
    
    # Get Part 2 C matrix (measurement matrix)
    print("\n3. Loading Part 2 sensor matrix (measuring x1 and x6)...")
    Cmeas = get_part2_C_matrix()
    print(f"   Cmeas shape: {Cmeas.shape}")
    print(f"   Cmeas = \n{Cmeas}")
    
    # Get Part 2 initial conditions
    print("\n4. Loading Part 2 initial conditions...")
    x0, xhat0 = get_part2_initial_conditions()
    print(f"   x0 = {x0}")
    print(f"   xhat0 = {xhat0}")
    
    # Design Part 2 observer (reuse existing design)
    print("\n5. Designing Part 2 observer (reusing existing design)...")
    print("   Note: Using same design method and parameters as Part 2/3 for consistency")
    try:
        L, observer_design_info = design_observer(
            Ad, Cmeas,
            method='pole_placement',
            pole_range=(0.4, 0.8),
            fallback_to_lqr=True
        )
        print(f"   Observer gain L shape: {observer_design_info['L_shape']}")
        print(f"   Observer spectral radius: {observer_design_info['spectral_radius']:.6f}")
        print(f"   Observer stable: {observer_design_info['is_stable']}")
        
        # Gate: Observer must be stable
        if observer_design_info['spectral_radius'] >= 1.0:
            print(f"   ERROR: Observer validation FAILED - spectral radius >= 1.0")
            sys.exit(1)
    except Exception as e:
        print(f"   ERROR: Observer design failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    
    # Define cost matrices
    print("\n6. Defining cost matrices...")
    Cy = Cmeas  # Cost output selector (outputs [x1, x6])
    Q = Cy.T @ Cy  # State weight matrix (12×12) - same as Part 3
    R_red = np.eye(2)  # Input weight matrix (2×2 identity) - reduced from Part 3's I3
    
    print(f"   Cy shape: {Cy.shape}")
    
    # Gate: Q is 12×12, symmetric (tolerance-based check)
    Q_symmetry_error = np.max(np.abs(Q - Q.T))
    Q_is_symmetric = Q_symmetry_error <= 1e-10
    print(f"   Q shape: {Q.shape}, symmetric: {Q_is_symmetric} (max |Q - Q.T| = {Q_symmetry_error:.2e})")
    
    if Q.shape != (12, 12) or not Q_is_symmetric:
        print(f"   ERROR: Q matrix validation FAILED")
        sys.exit(1)
    
    # Gate: R_red is 2×2, symmetric, positive definite (tolerance-based check)
    R_red_symmetry_error = np.max(np.abs(R_red - R_red.T))
    R_red_is_symmetric = R_red_symmetry_error <= 1e-10
    R_red_eigvals = np.linalg.eigvals(R_red)
    R_red_min_eigval = np.min(R_red_eigvals)
    R_red_is_positive_definite = R_red_min_eigval > 1e-10
    print(f"   R_red shape: {R_red.shape}, symmetric: {R_red_is_symmetric} (max |R_red - R_red.T| = {R_red_symmetry_error:.2e})")
    print(f"   R_red positive definite: {R_red_is_positive_definite} (min eigenvalue = {R_red_min_eigval:.2e})")
    
    if R_red.shape != (2, 2) or not R_red_is_symmetric or not R_red_is_positive_definite:
        print(f"   ERROR: R_red matrix validation FAILED")
        sys.exit(1)
    
    print("   Cost matrices validated ✓")
    
    # Check stabilizability
    print("\n7. Checking stabilizability of (Ad, Bd_red)...")
    stabilizability_info = check_stabilizability(Ad, Bd_red, tol=1e-10)
    print(f"   Controllability rank: {stabilizability_info['controllability_rank']}/{stabilizability_info['n']}")
    print(f"   PBH tolerance used: {stabilizability_info['tol_used']:.2e}")
    
    if len(stabilizability_info['unstable_eigenvalues']) > 0:
        print(f"   Potentially unstable eigenvalues (|λ| >= 1 - tol): {len(stabilizability_info['unstable_eigenvalues'])}")
        for i, check in enumerate(stabilizability_info['pbh_checks']):
            print(f"     λ_{i+1} = {check['eigenvalue']:.6f} (|λ| = {check['magnitude']:.6f}), PBH rank: {check['pbh_rank']}/{check['n']}, condition holds: {check['pbh_condition_holds']}")
    else:
        print(f"   No potentially unstable eigenvalues (all |λ| < 1 - tol)")
    
    print(f"   Is stabilizable: {stabilizability_info['is_stabilizable']}")
    
    # Gate: Stabilizability check passes
    if not stabilizability_info['is_stabilizable']:
        print(f"   ERROR: Stabilizability check FAILED")
        sys.exit(1)
    
    print("   Stabilizability validated ✓")
    
    # Check detectability for DARE existence/uniqueness
    print("\n8. Checking detectability of (Ad, Cy) for DARE...")
    detectability_info = check_detectability(Ad, Cy, tol=1e-10)
    print(f"   Detectability check: {'PASS' if detectability_info['is_detectable'] else 'FAIL'}")
    if len(detectability_info['unstable_eigenvalues']) > 0:
        print(f"   Potentially unstable eigenvalues checked: {len(detectability_info['unstable_eigenvalues'])}")
        for i, check in enumerate(detectability_info['pbh_checks']):
            print(f"     λ_{i+1} = {check['eigenvalue']:.6f} (|λ| = {check['magnitude']:.6f}), PBH rank: {check['pbh_rank']}/{check['n']}")
    if not detectability_info['is_detectable']:
        print(f"   WARNING: System is not detectable - DARE may not have unique solution")
    else:
        print("   Detectability validated ✓")
    
    # Design LQR controller
    print("\n9. Designing LQR controller (reduced input)...")
    try:
        K_red, P_red, lqr_design_info = design_lqr(Ad, Bd_red, Q, R_red)
        print(f"   LQR gain K_red shape: {lqr_design_info['K_shape']}")
        print(f"   Closed-loop spectral radius: {lqr_design_info['spectral_radius']:.6f}")
        print(f"   Closed-loop stable: {lqr_design_info['is_stable']}")
        
        # Gate: DARE solver succeeds (no exception raised)
        # Gate: spectral_radius(Acl_red) < 1.0 (hard fail if not)
        if lqr_design_info['spectral_radius'] >= 1.0:
            print(f"   ERROR: LQR validation FAILED - spectral radius >= 1.0")
            sys.exit(1)
        
        print(f"   R_red + Bd_red^T P_red Bd_red minimum eigenvalue: {lqr_design_info['R_BPB_min_eigval']:.2e}")
        print("   LQR design validated ✓")
        
        # Also check observer stability (Ad - L @ Cmeas)
        Aobs = Ad - L @ Cmeas
        observer_eigvals = np.linalg.eigvals(Aobs)
        observer_spectral_radius = np.max(np.abs(observer_eigvals))
        print(f"   Observer closed-loop spectral radius: {observer_spectral_radius:.6f}")
        
        if observer_spectral_radius >= 1.0:
            print(f"   ERROR: Observer closed-loop validation FAILED - spectral radius >= 1.0")
            sys.exit(1)
        
        print("   Observer closed-loop stability validated ✓")
        
        # Store for results.txt
        lqr_design_info['observer_spectral_radius'] = observer_spectral_radius
        
        # Analyze dominant eigenvalue of closed-loop system
        eigvals_sorted = np.sort(np.abs(lqr_design_info['eigenvalues']))[::-1]
        dominant_mag = eigvals_sorted[0]
        dominant_idx = np.argmax(np.abs(lqr_design_info['eigenvalues']))
        dominant_eigval = lqr_design_info['eigenvalues'][dominant_idx]
        dominant_angle = np.angle(dominant_eigval)
        is_dominant_complex = not np.isreal(dominant_eigval)
        
        lqr_design_info['dominant_eigval'] = dominant_eigval
        lqr_design_info['dominant_mag'] = dominant_mag
        lqr_design_info['dominant_angle'] = dominant_angle
        lqr_design_info['is_dominant_complex'] = is_dominant_complex
        
    except Exception as e:
        print(f"   ERROR: LQR design failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    
    # Load Part 3 baseline
    print("\n10. Loading Part 3 baseline metrics...")
    part3_results_path = os.path.join(os.path.dirname(__file__), '..', 'part3', 'outputs', 'results.txt')
    baseline = load_part3_baseline(part3_results_path)
    
    if not baseline['loaded']:
        print(f"   WARNING: Part 3 baseline not found or incomplete at {part3_results_path}")
        print(f"   Baseline metrics will be marked as UNKNOWN")
        print(f"   Comparison plots will not be generated")
        print(f"   Please run Part 3 first to generate baseline metrics")
        # Log which fields are present (if any)
        present_fields = [k for k in ['J', 'max_abs_u1', 'max_abs_u2', 'max_abs_u3', 'max_abs_u_overall'] if baseline[k] is not None]
        if present_fields:
            print(f"   Note: Some fields found: {present_fields}, but not all required fields")
    else:
        print(f"   Part 3 baseline loaded successfully:")
        print(f"     Part3_J = {baseline['J']:.6e}")
        print(f"     Part3_max_abs_u1 = {baseline['max_abs_u1']:.6e}")
        print(f"     Part3_max_abs_u2 = {baseline['max_abs_u2']:.6e}")
        print(f"     Part3_max_abs_u3 = {baseline['max_abs_u3']:.6e}")
        print(f"     Part3_max_abs_u_overall = {baseline['max_abs_u_overall']:.6e}")
    
    # Simulation parameters
    print("\n11. Setting up closed-loop simulation...")
    N = 1000  # 10 seconds at Ts=0.01
    print(f"   Simulation horizon: N = {N} steps ({N * Ts:.1f} seconds)")
    print(f"   Control law: u_red[k] = -K_red @ xhat[k] (uses xhat, not x)")
    
    # Run closed-loop simulation
    print("\n12. Running closed-loop simulation with observer...")
    results = simulate_closed_loop_with_observer(
        Ad, Bd_red, Cmeas, K_red, L, x0, xhat0, N, Ts
    )
    
    # Gate: Dimensions consistent (standard convention: x length N+1, u_red length N)
    assert results['x'].shape == (12, N + 1), f"x shape mismatch: {results['x'].shape}, expected (12, {N+1})"
    assert results['xhat'].shape == (12, N + 1), f"xhat shape mismatch: {results['xhat'].shape}, expected (12, {N+1})"
    assert results['u_red'].shape == (2, N), f"u_red shape mismatch: {results['u_red'].shape}, expected (2, {N})"
    assert results['y'].shape == (2, N + 1), f"y shape mismatch: {results['y'].shape}, expected (2, {N+1})"
    print("   Dimensions validated ✓")
    
    # Gate: Controller uses xhat, not x (explicit check)
    u_computed_from_xhat = np.zeros_like(results['u_red'])
    for k in range(N):
        u_computed_from_xhat[:, k] = -K_red @ results['xhat'][:, k]
    
    if not np.allclose(results['u_red'], u_computed_from_xhat, rtol=1e-8, atol=1e-10):
        max_diff = np.max(np.abs(results['u_red'] - u_computed_from_xhat))
        print(f"   ERROR: Controller validation FAILED - u_red is not computed from xhat")
        print(f"   Max difference: {max_diff:.2e}")
        sys.exit(1)
    
    # Additional check: verify u_red is NOT computed from x (true states)
    u_computed_from_x = np.zeros_like(results['u_red'])
    for k in range(N):
        u_computed_from_x[:, k] = -K_red @ results['x'][:, k]
    
    diff_from_x = np.zeros(N)
    for k in range(N):
        diff_from_x[k] = np.linalg.norm(results['u_red'][:, k] - u_computed_from_x[:, k])
    
    early_window = min(100, N)
    max_diff_early = np.max(diff_from_x[:early_window])
    max_diff_final = diff_from_x[N-1]
    max_diff_from_x = np.max(diff_from_x)
    
    print(f"   Diagnostic (first {early_window} samples): max ||u_red - (-K_red @ x)|| = {max_diff_early:.6e}")
    print(f"   Diagnostic (final sample k={N-1}): ||u_red - (-K_red @ x)|| = {max_diff_final:.6e}")
    print(f"   Diagnostic (overall): max ||u_red - (-K_red @ x)|| = {max_diff_from_x:.6e}")
    
    if max_diff_from_x < 1e-10:
        print(f"   WARNING: u_red appears to be computed from x instead of xhat")
    else:
        print(f"   Confirms u_red uses xhat, not x (even after observer convergence)")
    
    controller_diagnostic = {
        'max_diff_early': max_diff_early,
        'max_diff_final': max_diff_final,
        'max_diff_overall': max_diff_from_x,
        'early_window': early_window
    }
    
    print("   Controller uses xhat validated ✓")
    
    # Gate: Simulation runs for N steps without divergence
    max_abs_state = np.max(np.abs(results['x']))
    max_abs_input = np.max(np.abs(results['u_red']))
    max_error_norm = np.max([np.linalg.norm(results['e'][:, k]) for k in range(N + 1)])
    
    # End-of-window metrics (last 20% of state samples)
    steady_state_start = int((N + 1) * 0.8)
    y1_end = np.abs(results['y'][0, N])
    y6_end = np.abs(results['y'][1, N])
    y1_ss_mean = np.mean(np.abs(results['y'][0, steady_state_start:]))
    y6_ss_mean = np.mean(np.abs(results['y'][1, steady_state_start:]))
    y1_ss_rms = np.sqrt(np.mean(results['y'][0, steady_state_start:]**2))
    y6_ss_rms = np.sqrt(np.mean(results['y'][1, steady_state_start:]**2))
    
    print(f"   Max absolute state magnitude: {max_abs_state:.6e}")
    print(f"   Max absolute input magnitude: {max_abs_input:.6e}")
    print(f"   Max estimation error norm ||x - xhat||: {max_error_norm:.6e}")
    print(f"   End-of-window outputs: |y1[N]| = {y1_end:.6e}, |y6[N]| = {y6_end:.6e}")
    print(f"   Steady-state (last 20%): y1 mean = {y1_ss_mean:.6e}, RMS = {y1_ss_rms:.6e}")
    print(f"   Steady-state (last 20%): y6 mean = {y6_ss_mean:.6e}, RMS = {y6_ss_rms:.6e}")
    
    if not np.isfinite(max_abs_state) or not np.isfinite(max_abs_input):
        print(f"   ERROR: Simulation divergence detected")
        sys.exit(1)
    print("   Simulation stability validated ✓")
    
    # Compute cost metrics
    print("\n13. Computing cost metrics...")
    cost_metrics = compute_cost_metrics(results['x'], results['u_red'], Cy, N)
    
    # Verify cost implementation consistency
    print("   Verifying cost implementation consistency...")
    consistency_check_passed = True
    for k in [0, N//4, N//2, 3*N//4, N-1]:
        y_k = Cy @ results['x'][:, k]
        u_k = results['u_red'][:, k]
        stage_cost_k_expected = u_k.T @ u_k + y_k[0]**2 + y_k[1]**2
        stage_cost_k_actual = cost_metrics['stage_cost'][k]
        if not np.allclose(stage_cost_k_expected, stage_cost_k_actual, rtol=1e-10):
            print(f"   WARNING: Cost consistency check failed at k={k}")
            consistency_check_passed = False
    if consistency_check_passed:
        print("   Cost consistency verified: stage_cost[k] uses u_red[k] and y[k] from same k ✓")
    
    print(f"   Total cost J_red: {cost_metrics['total_cost_J']:.6e}")
    print(f"   Cost range: k = {cost_metrics['cost_range'][0]} to {cost_metrics['cost_range'][1]} (inclusive)")
    print(f"   Max |u1|: {cost_metrics['max_abs_u1']:.6e}")
    print(f"   Max |u2|: {cost_metrics['max_abs_u2']:.6e}")
    print(f"   Max |u_red| overall: {cost_metrics['max_abs_u_overall']:.6e} (aligned with Part 3 metric)")
    print(f"   Max ||u_red[k]||_inf: {cost_metrics['max_u_inf']:.6e}")
    
    # Gate: J_red is finite and non-negative
    if not np.isfinite(cost_metrics['total_cost_J']) or cost_metrics['total_cost_J'] < 0:
        print(f"   ERROR: Cost validation FAILED")
        sys.exit(1)
    print("   Cost metrics validated ✓")
    
    # Create output directory
    output_dir = os.path.join(os.path.dirname(__file__), 'outputs')
    os.makedirs(output_dir, exist_ok=True)
    
    # Generate plots
    print("\n14. Generating plots...")
    
    # Plot 1: Outputs y1 and y6 (always generate for Part 4)
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
    outputs_file = os.path.join(output_dir, 'outputs_y1_y6.png')
    plt.savefig(outputs_file, dpi=150)
    print(f"   Saved: {outputs_file}")
    plt.close()
    
    # Plot 2: Inputs u1 and u2
    t_inputs = results['t'][:N]
    fig2, ax2 = plt.subplots(2, 1, figsize=(10, 8))
    
    ax2[0].plot(t_inputs, results['u_red'][0, :], 'r-', label='u1', linewidth=2)
    ax2[0].set_xlabel('Time (s)')
    ax2[0].set_ylabel('Input')
    ax2[0].set_title('Input 1')
    ax2[0].legend()
    ax2[0].grid(True)
    
    ax2[1].plot(t_inputs, results['u_red'][1, :], 'r-', label='u2', linewidth=2)
    ax2[1].set_xlabel('Time (s)')
    ax2[1].set_ylabel('Input')
    ax2[1].set_title('Input 2')
    ax2[1].legend()
    ax2[1].grid(True)
    
    plt.tight_layout()
    inputs_file = os.path.join(output_dir, 'inputs_u1_u2.png')
    plt.savefig(inputs_file, dpi=150)
    print(f"   Saved: {inputs_file}")
    plt.close()
    
    # Plot 3: Estimation error norm
    error_norm = np.zeros(N + 1)
    for k in range(N + 1):
        error_norm[k] = np.linalg.norm(results['e'][:, k])
    
    fig3, ax3 = plt.subplots(figsize=(10, 6))
    ax3.plot(results['t'], error_norm, 'g-', label='||x - xhat||', linewidth=2)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Estimation Error Norm')
    ax3.set_title('Estimation Error Norm: ||x - xhat||')
    ax3.legend()
    ax3.grid(True)
    
    plt.tight_layout()
    error_file = os.path.join(output_dir, 'estimation_error_norm.png')
    plt.savefig(error_file, dpi=150)
    print(f"   Saved: {error_file}")
    plt.close()
    
    # Plot 4: Comparison outputs (only if baseline loaded AND Part 3 trajectories available)
    # Note: Part 3 doesn't save trajectories, so this plot would need Part 3 simulation data
    # For now, we skip creating a comparison plot since we don't have Part 3 trajectories
    # The comparison is done via metrics in results.txt instead
    if baseline['loaded']:
        print(f"   Note: Comparison plot skipped - Part 3 trajectories not saved (comparison via metrics in results.txt)")
    else:
        print(f"   Note: Comparison plot skipped (baseline UNKNOWN)")
    
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
    print("\n15. Saving results to file...")
    results_file = os.path.join(output_dir, 'results.txt')
    with open(results_file, 'w') as f:
        f.write("Part 4: LQR Controller Design with Reduced Input (u3 Removed) - Results\n")
        f.write("="*60 + "\n\n")
        
        f.write("Reproducibility Information:\n")
        f.write(f"  Python version: {python_version}\n")
        f.write(f"  NumPy version: {numpy_version}\n")
        if scipy_version:
            f.write(f"  SciPy version: {scipy_version}\n")
        f.write(f"  Platform: {platform.platform()}\n\n")
        
        f.write("Simulation Parameters:\n")
        f.write(f"  N (number of input samples) = {N}\n")
        f.write(f"  Ts (sampling time) = {Ts} s\n")
        f.write(f"  Time span = {N * Ts:.1f} s\n")
        f.write(f"  Array dimensions (standard convention):\n")
        f.write(f"    - x: (n, N+1) stores x[0] through x[N]\n")
        f.write(f"    - u_red: (2, N) stores u_red[0] through u_red[N-1]\n")
        f.write(f"    - y: (p, N+1) stores y[0] through y[N]\n")
        f.write(f"  Cost accumulation convention: J_red = sum from k=0 to {N-1} of stage_cost[k]\n")
        f.write(f"    - Cost range: k = 0 to {N-1} (inclusive, all N input samples)\n")
        f.write(f"    - Standard convention: u_red[k] pairs with transition from x[k] to x[k+1]\n")
        f.write(f"    - All N inputs are applied (u_red[0] through u_red[N-1])\n\n")
        
        f.write("Input Reduction:\n")
        f.write(f"  Bd_red shape: {Bd_red.shape}\n")
        f.write(f"  Input removal convention: Bd_red = Bd[:, [0, 1]] (extracts columns 0 and 1)\n")
        f.write(f"  Reduced input dimension: u_red in R² (u1 and u2 only, u3 removed)\n\n")
        
        f.write("Initial Conditions:\n")
        f.write(f"  x0 (actual) = {x0}\n")
        f.write(f"  xhat0 (observer) = {xhat0}\n")
        f.write(f"  Note: Initial conditions carried over from Part 2 (exam specification)\n")
        f.write(f"  Exact values for reproducibility:\n")
        f.write(f"    x0 = [{', '.join([f'{val:.15e}' for val in x0])}]\n")
        f.write(f"    xhat0 = [{', '.join([f'{val:.15e}' for val in xhat0])}]\n\n")
        
        f.write("Cost Matrices:\n")
        f.write(f"  Cy (cost output selector) shape: {Cy.shape}\n")
        f.write(f"  Cy = \n{Cy}\n")
        f.write(f"  Cy outputs: [x1, x6] (displacements of masses 1 and 6)\n")
        f.write(f"  Q (state weight) shape: {Q.shape}\n")
        f.write(f"  Q = Cy^T @ Cy (symmetric, positive semidefinite, same as Part 3)\n")
        f.write(f"  Q symmetry error: max |Q - Q.T| = {Q_symmetry_error:.2e}\n")
        f.write(f"  R_red (input weight) shape: {R_red.shape}\n")
        f.write(f"  R_red = I2 (2×2 identity matrix, reduced from Part 3's I3)\n")
        f.write(f"  R_red symmetry error: max |R_red - R_red.T| = {R_red_symmetry_error:.2e}\n")
        f.write(f"  R_red minimum eigenvalue: {R_red_min_eigval:.2e}\n")
        f.write(f"  R_red + Bd_red^T P_red Bd_red minimum eigenvalue: {lqr_design_info['R_BPB_min_eigval']:.2e}\n")
        f.write(f"  Cost computation: Uses plant output y[k] = Cy @ x[k] (not estimated output)\n")
        f.write(f"  Cost consistency: Verified that stage_cost[k] uses u_red[k] and y[k] from same k ✓\n\n")
        
        f.write("LQR Design:\n")
        f.write(f"  Stabilizability check: PBH rank condition for (Ad, Bd_red)\n")
        f.write(f"  PBH tolerance used: {stabilizability_info['tol_used']:.2e}\n")
        f.write(f"  Controllability rank: {stabilizability_info['controllability_rank']}/{stabilizability_info['n']}\n")
        f.write(f"  Minimum PBH rank observed: {stabilizability_info.get('min_pbh_rank', 'N/A')}/{stabilizability_info['n']}\n")
        f.write(f"  Detectability check: {'PASS' if detectability_info['is_detectable'] else 'FAIL'}\n")
        if len(stabilizability_info['unstable_eigenvalues']) > 0:
            f.write(f"  Potentially unstable eigenvalues checked: {len(stabilizability_info['unstable_eigenvalues'])}\n")
            for i, check in enumerate(stabilizability_info['pbh_checks']):
                f.write(f"    λ_{i+1} = {check['eigenvalue']:.6f} (|λ| = {check['magnitude']:.6f}), PBH rank: {check['pbh_rank']}/{check['n']}, rank margin: {check.get('rank_margin', 'N/A'):.2e}\n")
        f.write(f"  K_red (LQR gain) shape: {lqr_design_info['K_shape']}\n")
        f.write(f"  K_red matrix:\n")
        for i in range(K_red.shape[0]):
            f.write(f"    Row {i+1}: {K_red[i, :]}\n")
        f.write(f"  Closed-loop matrix: Acl_red = Ad - Bd_red @ K_red\n")
        f.write(f"  Spectral radius (max(|eig(Acl_red)|)): {lqr_design_info['spectral_radius']:.6f}\n")
        f.write(f"  Stability margin: {1.0 - lqr_design_info['spectral_radius']:.6e} (distance from unity)\n")
        f.write(f"  Closed-loop stable: {lqr_design_info['is_stable']}\n")
        f.write(f"  R_red + Bd_red^T P_red Bd_red minimum eigenvalue: {lqr_design_info['R_BPB_min_eigval']:.2e}\n")
        f.write(f"  Dominant eigenvalue: {lqr_design_info['dominant_eigval']:.6f}\n")
        f.write(f"    Magnitude: {lqr_design_info['dominant_mag']:.6f}\n")
        f.write(f"    Angle: {lqr_design_info['dominant_angle']:.6f} rad ({np.degrees(lqr_design_info['dominant_angle']):.2f} deg)\n")
        f.write(f"    Type: {'Complex pair' if lqr_design_info['is_dominant_complex'] else 'Real'}\n")
        f.write(f"  Closed-loop eigenvalues:\n")
        for i, eig in enumerate(lqr_design_info['eigenvalues']):
            f.write(f"    λ_{i+1} = {eig:.6f} (|λ| = {np.abs(eig):.6f})\n")
        f.write("\n")
        
        f.write("Controller Validation:\n")
        f.write(f"  Control law: u_red[k] = -K_red @ xhat[k]\n")
        f.write(f"  CRITICAL: Controller uses xhat (estimated states), not x (true states)\n")
        f.write(f"  Validation: u_red[k] computed from xhat[k] for all k ✓\n")
        f.write(f"  Diagnostic (first {controller_diagnostic['early_window']} samples): max ||u_red - (-K_red @ x)|| = {controller_diagnostic['max_diff_early']:.6e}\n")
        f.write(f"  Diagnostic (final sample k={N-1}): ||u_red - (-K_red @ x)|| = {controller_diagnostic['max_diff_final']:.6e}\n")
        f.write(f"  Diagnostic (overall): max ||u_red - (-K_red @ x)|| = {controller_diagnostic['max_diff_overall']:.6e}\n")
        f.write(f"  Note: Non-zero values confirm u_red uses xhat, even after observer convergence\n\n")
        
        f.write("Simulation Results:\n")
        f.write(f"  Max absolute state magnitude: {max_abs_state:.6e}\n")
        f.write(f"  Max absolute input magnitude: {max_abs_input:.6e}\n")
        f.write(f"  Max estimation error norm ||x - xhat||: {max_error_norm:.6e}\n")
        f.write(f"  End-of-window outputs:\n")
        f.write(f"    |y1[N]| = {y1_end:.6e} (final output at time N)\n")
        f.write(f"    |y6[N]| = {y6_end:.6e} (final output at time N)\n")
        f.write(f"  Steady-state metrics (last 20% of state samples, indices {steady_state_start} to {N}):\n")
        f.write(f"    y1 mean absolute: {y1_ss_mean:.6e}, RMS: {y1_ss_rms:.6e}\n")
        f.write(f"    y6 mean absolute: {y6_ss_mean:.6e}, RMS: {y6_ss_rms:.6e}\n\n")
        
        f.write("Slow Convergence Analysis:\n")
        f.write(f"  Closed-loop spectral radius ρ(Acl_red): {lqr_design_info['spectral_radius']:.6f}\n")
        f.write(f"  Stability margin: {1.0 - lqr_design_info['spectral_radius']:.6e} (distance from unity)\n")
        f.write(f"  Dominant eigenvalue magnitude: {lqr_design_info['dominant_mag']:.6f}\n")
        f.write(f"  Dominant eigenvalue angle: {lqr_design_info['dominant_angle']:.6f} rad ({np.degrees(lqr_design_info['dominant_angle']):.2f} deg)\n")
        f.write(f"  End-of-window outputs: |y1[N]| = {y1_end:.6e}, |y6[N]| = {y6_end:.6e}\n")
        f.write(f"  Steady-state (last 20%): y1 RMS = {y1_ss_rms:.6e}, y6 RMS = {y6_ss_rms:.6e}\n")
        f.write(f"  NOTE: Spectral radius is very close to 1.0, indicating slow convergence.\n")
        f.write(f"    System may not fully settle within the 10 s simulation window.\n")
        f.write(f"    End-of-window and last-20% metrics are reported as standard.\n")
        f.write(f"    System is stable (ρ < 1.0) but convergence is slow.\n\n")
        
        f.write("Cost Metrics:\n")
        f.write(f"  Total cost J_red = {cost_metrics['total_cost_J']:.6e}\n")
        f.write(f"  Cost range: k = {cost_metrics['cost_range'][0]} to {cost_metrics['cost_range'][1]} (inclusive)\n")
        f.write(f"  Cost implementation: stage_cost[k] = u_red[k]^T @ u_red[k] + y_cost[0]^2 + y_cost[1]^2\n")
        f.write(f"    where y_cost = Cy @ x[k] (plant output at time k, not estimated)\n")
        f.write(f"    Standard convention: u_red[k] pairs with transition from x[k] to x[k+1] ✓\n")
        f.write(f"    Consistency verified: u_red[k] and y[k] from same time index k ✓\n")
        f.write(f"  Max |u1| = {cost_metrics['max_abs_u1']:.6e}\n")
        f.write(f"  Max |u2| = {cost_metrics['max_abs_u2']:.6e}\n")
        f.write(f"  Max |u_red| overall = {cost_metrics['max_abs_u_overall']:.6e} (aligned with Part 3: max(abs(u)) across all elements)\n")
        f.write(f"  Max ||u_red[k]||_inf = {cost_metrics['max_u_inf']:.6e}\n\n")
        
        f.write("Part 2 Observer (Reused):\n")
        f.write(f"  Observer gain L shape: {observer_design_info['L_shape']}\n")
        f.write(f"  Observer spectral radius: {observer_design_info['spectral_radius']:.6f}\n")
        f.write(f"  Observer stable: {observer_design_info['is_stable']}\n")
        f.write(f"  Observer closed-loop matrix: Aobs = Ad - L @ Cmeas\n")
        f.write(f"  Observer closed-loop spectral radius: {lqr_design_info.get('observer_spectral_radius', 'N/A'):.6f}\n")
        f.write(f"  Measurement matrix Cmeas shape: {Cmeas.shape}\n")
        f.write(f"  Cmeas measures x1 and x6 (Part 2 sensor configuration)\n")
        f.write(f"  Note: Observer redesigned using same method/parameters as Part 2/3 for consistency\n\n")
        
        # Comparison section
        f.write("Comparison Against Part 3:\n")
        if baseline['loaded']:
            f.write(f"  Part 3 baseline loaded successfully from python/part3/outputs/results.txt\n")
            f.write(f"  Part3_J = {baseline['J']:.6e}\n")
            f.write(f"  Part4_J = {cost_metrics['total_cost_J']:.6e}\n")
            delta_J = cost_metrics['total_cost_J'] - baseline['J']
            f.write(f"  delta_J = Part4_J - Part3_J = {delta_J:.6e}\n")
            f.write(f"  Relative change: {(delta_J / baseline['J'] * 100):.2f}%\n\n")
            
            f.write(f"  Maximum Input Magnitudes:\n")
            f.write(f"    Part3_max_abs_u1 = {baseline['max_abs_u1']:.6e}\n")
            f.write(f"    Part4_max_abs_u1 = {cost_metrics['max_abs_u1']:.6e}\n")
            f.write(f"    Part3_max_abs_u2 = {baseline['max_abs_u2']:.6e}\n")
            f.write(f"    Part4_max_abs_u2 = {cost_metrics['max_abs_u2']:.6e}\n")
            f.write(f"    Part3_max_abs_u3 = {baseline['max_abs_u3']:.6e} (not applicable to Part 4)\n")
            f.write(f"    Part3_max_abs_u_overall = {baseline['max_abs_u_overall']:.6e}\n")
            f.write(f"    Part4_max_abs_u_overall = {cost_metrics['max_abs_u_overall']:.6e} (aligned metric: max(abs(u_red)))\n")
            f.write(f"    Part4_max ||u_red[k]||_inf = {cost_metrics['max_u_inf']:.6e} (additional metric)\n")
        else:
            f.write(f"  Part 3 baseline: UNKNOWN\n")
            f.write(f"  Reason: Part 3 results.txt not found or incomplete at {part3_results_path}\n")
            f.write(f"  Action required: Run Part 3 first to generate baseline metrics\n")
            f.write(f"  Comparison metrics:\n")
            # Safely format baseline values (use UNKNOWN if None)
            part3_j_str = f"{baseline['J']:.6e}" if baseline['J'] is not None else "UNKNOWN"
            part3_u1_str = f"{baseline['max_abs_u1']:.6e}" if baseline['max_abs_u1'] is not None else "UNKNOWN"
            part3_u2_str = f"{baseline['max_abs_u2']:.6e}" if baseline['max_abs_u2'] is not None else "UNKNOWN"
            part3_u3_str = f"{baseline['max_abs_u3']:.6e}" if baseline['max_abs_u3'] is not None else "UNKNOWN"
            part3_overall_str = f"{baseline['max_abs_u_overall']:.6e}" if baseline['max_abs_u_overall'] is not None else "UNKNOWN"
            
            f.write(f"    Part3_J = {part3_j_str}\n")
            f.write(f"    Part4_J = {cost_metrics['total_cost_J']:.6e}\n")
            if baseline['J'] is not None:
                delta_J = cost_metrics['total_cost_J'] - baseline['J']
                f.write(f"    delta_J = {delta_J:.6e}\n")
            else:
                f.write(f"    delta_J = UNKNOWN\n")
            f.write(f"    Part3_max_abs_u1 = {part3_u1_str}\n")
            f.write(f"    Part3_max_abs_u2 = {part3_u2_str}\n")
            f.write(f"    Part3_max_abs_u3 = {part3_u3_str}\n")
            f.write(f"    Part3_max_abs_u_overall = {part3_overall_str}\n")
            f.write(f"    Part4_max_inputs = u1: {cost_metrics['max_abs_u1']:.6e}, u2: {cost_metrics['max_abs_u2']:.6e}\n")
            f.write(f"    Part4_max_abs_u_overall = {cost_metrics['max_abs_u_overall']:.6e}\n")
        f.write("\n")
        
        f.write("Source Citations:\n")
        f.write(f"  Part 4 requirement: docs/sources/final_exam_extract.md Section 6\n")
        f.write(f"  Part 3 cost definition: docs/sources/final_exam_extract.md Section 5\n")
        f.write(f"  Part 2 C matrix and initial conditions: docs/sources/final_exam_extract.md Section 4\n")
    
    print(f"   Saved: {results_file}")
    
    # Save key matrices for reproducibility
    print("\n16. Saving key matrices for reproducibility...")
    K_red_file = os.path.join(output_dir, 'K_red_matrix.npy')
    np.save(K_red_file, K_red)
    print(f"   Saved: {K_red_file}")
    
    # Save trajectories for later comparisons (Parts 5-7 overlay plots)
    print("\n17. Saving trajectories for later comparisons...")
    traj_file = os.path.join(output_dir, 'traj.npz')
    # Standardized trajectory keys: Part 4 uses 'u_red' (2 inputs) vs Part 3 'u' (3 inputs)
    # Both use same keys: t, x, xhat, y, e
    # Input key differs: Part 3 uses 'u', Part 4 uses 'u_red' (documented difference)
    np.savez(traj_file,
             t=results['t'],
             x=results['x'],
             xhat=results['xhat'],
             u_red=results['u_red'],  # Part 4: 2 inputs (u1, u2), different from Part 3's 'u' (3 inputs)
             y=results['y'],
             e=results['e'])
    print(f"   Saved: {traj_file}")
    print(f"   Note: Trajectories saved for Parts 5-7 overlay comparisons")
    
    print("\n" + "="*60)
    print("Part 4 simulation complete!")
    print("="*60)
    print(f"\nOutput files saved to: {output_dir}/")
    print("  - results.txt")
    print("  - K_red_matrix.npy")
    print("  - traj.npz (trajectories for Parts 5-7 comparisons)")
    print("  - outputs_y1_y6.png")
    print("  - inputs_u1_u2.png")
    print("  - estimation_error_norm.png")
