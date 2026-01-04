"""
Observer design for Part 2: Discrete-time Luenberger observer.

This module provides functions to:
1. Compute observability rank for (Ad, Cd_new) where Cd_new measures x1 and x6
2. Design observer gain L using pole placement via dual system approach
"""

import numpy as np
from scipy.signal import place_poles
from scipy.linalg import solve_discrete_are
try:
    from scipy.linalg import balance_matrix
    HAS_BALANCE = True
except ImportError:
    # Older scipy versions don't have balance_matrix
    HAS_BALANCE = False
import sys
import os

# Add parent directory to path to import Part 1 utilities
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from part1.observability_rank import analyze_observability


def get_part2_C_matrix():
    """
    Get Part 2 sensor matrix Cd_new (2×12) measuring x1 and x6.
    
    Source: Unverified (from docs/00_anchor.md, to be verified against docs/sources/final_exam.pdf)
    Expected: Measures displacement of mass 1 (x1) and mass 6 (x6)
    
    Returns:
        Cd_new: (2, 12) output matrix
    """
    # Part 2 C matrix: measures x1 (row 1) and x6 (row 2)
    # Row 1: [1 0 0 0 0 0 0 0 0 0 0 0] - measures x1
    # Row 2: [0 0 0 0 0 1 0 0 0 0 0 0] - measures x6
    Cd_new = np.array([
        [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
    ])
    return Cd_new


def check_dual_controllability(Ad, Cd_new, tol=1e-10):
    """
    Check controllability rank of dual system (Ad^T, Cd_new^T).
    
    This is equivalent to observability of (Ad, Cd_new).
    
    Args:
        Ad: (n, n) discrete-time state matrix
        Cd_new: (p, n) output matrix
        tol: Tolerance for rank computation
    
    Returns:
        dict: Controllability analysis results
    """
    n = Ad.shape[0]
    p = Cd_new.shape[0]
    
    # Form dual system
    Ad_dual = Ad.T
    Bd_dual = Cd_new.T  # (n, p)
    
    # Construct controllability matrix for dual system
    # O = [B, A*B, A^2*B, ..., A^(n-1)*B]
    O_dual = np.zeros((n, n * p))
    AB_power = np.eye(n)
    for i in range(n):
        O_dual[:, i*p:(i+1)*p] = AB_power @ Bd_dual
        AB_power = Ad_dual @ AB_power
    
    # Compute rank
    U, sigma, Vh = np.linalg.svd(O_dual, full_matrices=False)
    machine_eps = np.finfo(O_dual.dtype).eps
    max_sigma = np.max(sigma)
    tol_used = max(tol, machine_eps * max_sigma)
    threshold = tol_used * max_sigma
    rank = np.sum(sigma > threshold)
    
    return {
        'rank': rank,
        'n': n,
        'is_controllable': (rank == n),
        'min_singular_value': np.min(sigma[sigma > threshold]) if rank > 0 else 0.0,
        'condition_number': max_sigma / (np.min(sigma[sigma > threshold]) if rank > 0 else 1.0)
    }


def design_observer_pole_placement(Ad, Cd_new, desired_poles=None, 
                                   pole_range=(0.4, 0.8),
                                   method='YT', maxiter=1000, rtol=1e-2,
                                   stability_margin=0.90, use_balancing=True):
    """
    Design observer gain L using pole placement via dual system approach.
    
    Observer design for (Ad, Cd_new) is dual to controller design for (Ad^T, Cd_new^T).
    We solve pole placement for the dual system, then transpose to get L.
    
    Design policy: Uses 12 distinct real poles evenly spaced in [0.4, 0.8].
    This avoids duplicate poles and convergence issues with complex pole angles.
    
    Args:
        Ad: (n, n) discrete-time state matrix
        Cd_new: (p, n) output matrix (2×12 for Part 2)
        desired_poles: Optional array of desired observer pole locations.
            If None, uses policy: 12 distinct real poles in pole_range.
        pole_range: (min, max) tuple for real pole selection (default (0.4, 0.8))
        method: SciPy placement method ('YT' or 'KNV0', default 'YT')
        maxiter: Maximum iterations for solver (default 1000)
        rtol: Relative tolerance for solver (default 1e-2)
        stability_margin: Target spectral radius (default 0.90)
        use_balancing: Apply balancing transformation for better conditioning (default True)
    
    Returns:
        tuple: (L, design_info) where
            - L: (n, p) observer gain matrix (12×2)
            - design_info: dict with design parameters and eigenvalues
    
    Raises:
        RuntimeError: If pole placement fails or observer is unstable
    """
    n = Ad.shape[0]
    p = Cd_new.shape[0]
    
    # Check controllability of dual system (hard gate)
    ctrl_info = check_dual_controllability(Ad, Cd_new)
    if not ctrl_info['is_controllable']:
        raise RuntimeError(
            f"Dual system is not controllable (rank {ctrl_info['rank']}/{n}). "
            f"Pole placement cannot proceed."
        )
    
    # Record conditioning info for logging
    cond_before = ctrl_info['condition_number']
    
    # Design policy: Use 12 distinct real poles (avoids duplicate poles and convergence issues)
    if desired_poles is None:
        # Generate n distinct real poles evenly spaced in pole_range
        min_pole, max_pole = pole_range
        desired_poles = np.linspace(min_pole, max_pole, n)
    
    desired_poles = np.asarray(desired_poles)
    if desired_poles.size != n:
        raise ValueError(f"Number of desired poles ({desired_poles.size}) must equal system dimension ({n})")
    
    # Validate desired poles: ensure all inside unit circle
    if np.any(np.abs(desired_poles) >= 1.0):
        max_desired = np.max(np.abs(desired_poles))
        raise ValueError(f"Desired poles must be inside unit circle. Max magnitude: {max_desired:.6f}")
    
    # Apply balancing transformation for better conditioning (optional)
    balancing_used = False
    T_bal = np.eye(n)
    cond_after = cond_before
    
    if use_balancing and HAS_BALANCE:
        try:
            # Balance Ad for better numerical properties
            Ad_bal, T_bal = balance_matrix(Ad, permute=False, separate=False, overwrite_a=False)
            # Transform C: C_bal = C @ T_bal
            Cd_new_bal = Cd_new @ T_bal
            
            # Check condition number after balancing
            ctrl_info_bal = check_dual_controllability(Ad_bal, Cd_new_bal)
            cond_after = ctrl_info_bal['condition_number']
            
            # Form dual system on balanced matrices
            Ad_dual = Ad_bal.T
            Cd_new_dual = Cd_new_bal.T  # (n, p)
            balancing_used = True
        except Exception:
            # Fallback if balancing fails
            Ad_dual = Ad.T
            Cd_new_dual = Cd_new.T
            T_bal = np.eye(n)
            balancing_used = False
    else:
        # Form dual system: (Ad^T, Cd_new^T)
        Ad_dual = Ad.T
        Cd_new_dual = Cd_new.T  # (n, p)
        balancing_used = False
    
    # Try different methods in order: YT, then KNV0
    methods_to_try = [method] if method in ['YT', 'KNV0'] else ['YT', 'KNV0']
    L = None
    observer_poles = None
    method_used = None
    
    for method_attempt in methods_to_try:
        try:
            # place_poles with relaxed tolerances
            result = place_poles(
                Ad_dual, Cd_new_dual, desired_poles,
                method=method_attempt,
                maxiter=maxiter,
                rtol=rtol
            )
            L_dual = result.gain_matrix  # Shape: (p, n)
            
            # If balancing was used, map back to original coordinates
            if balancing_used:
                # L in balanced coords: L_bal = L_dual^T
                # Transform back: L = T_bal @ L_bal
                L_bal = L_dual.T  # (n, p)
                L_candidate = T_bal @ L_bal
            else:
                # Transpose to get observer gain: L = L_dual^T
                L_candidate = L_dual.T  # (n, p)
            
            # Verify observer poles
            A_L = Ad - L_candidate @ Cd_new
            observer_poles_candidate = np.linalg.eigvals(A_L)
            spectral_radius = np.max(np.abs(observer_poles_candidate))
            
            # Accept if stable
            if spectral_radius < 1.0:
                L = L_candidate
                observer_poles = observer_poles_candidate
                method_used = method_attempt
                break
                
        except Exception as e:
            if method_attempt == methods_to_try[-1]:
                raise RuntimeError(
                    f"Pole placement failed with all methods. "
                    f"Last attempt ({method_attempt}) error: {e}. "
                    f"Consider using dual LQR method instead."
                )
            continue
    
    # Final validation: spectral radius check
    if L is None:
        raise RuntimeError(
            f"Pole placement failed: Could not achieve stable observer with any method. "
            f"Try dual LQR method instead."
        )
    
    spectral_radius = np.max(np.abs(observer_poles))
    if spectral_radius >= 1.0:
        raise RuntimeError(
            f"Observer validation FAILED: spectral radius = {spectral_radius:.6f} >= 1.0. "
            f"Observer is unstable."
        )
    
    # Store design information
    design_info = {
        'method': 'pole_placement_dual',
        'placement_method': method_used,
        'design_policy': f'12 distinct real poles evenly spaced in {pole_range}',
        'desired_poles': desired_poles,
        'achieved_poles': observer_poles,
        'observer_poles': observer_poles,  # Alias for compatibility
        'max_desired_pole': np.max(np.abs(desired_poles)),
        'spectral_radius': spectral_radius,
        'max_achieved_pole': spectral_radius,
        'max_pole_magnitude': spectral_radius,  # Alias for compatibility
        'is_stable': spectral_radius < 1.0,
        'stability_margin': stability_margin,
        'balancing_used': balancing_used,
        'T_bal': T_bal,  # Transformation matrix (identity if not used)
        'cond_number_before': cond_before,
        'cond_number_after': cond_after,
        'ctrl_rank': ctrl_info['rank'],
        'ctrl_min_sv': ctrl_info['min_singular_value'],
        'L': L,
        'L_shape': L.shape
    }
    
    return L, design_info


def design_observer_dual_lqr(Ad, Cd_new, Qe=None, Re=None, alpha_sweep=None):
    """
    Design observer gain L using dual LQR approach (robust fallback).
    
    This method designs state feedback for the dual system (Ad^T, Cd_new^T) using
    discrete-time LQR, then transposes the gain to get the observer gain.
    
    This is more robust than pole placement for high-dimensional systems.
    
    Args:
        Ad: (n, n) discrete-time state matrix
        Cd_new: (p, n) output matrix
        Qe: (n, n) state weight matrix for dual system. If None, uses identity.
        Re: (p, p) input weight matrix for dual system. If None, uses alpha*I with alpha from sweep.
        alpha_sweep: List of alpha values to try for Re = alpha*I. 
            If None, uses [1e-3, 1e-2, 1e-1, 1, 10]
    
    Returns:
        tuple: (L, design_info) where
            - L: (n, p) observer gain matrix
            - design_info: dict with design parameters and eigenvalues
    
    Raises:
        RuntimeError: If all alpha values fail to produce stable observer
    """
    n = Ad.shape[0]
    p = Cd_new.shape[0]
    
    # Form dual system: (Ad^T, Cd_new^T)
    # For LQR: minimize cost for dual system
    A_dual = Ad.T
    B_dual = Cd_new.T  # (n, p)
    
    # Default Qe = I
    if Qe is None:
        Qe = np.eye(n)
    
    # Default alpha sweep
    if alpha_sweep is None:
        alpha_sweep = [1e-3, 1e-2, 1e-1, 1, 10]
    
    best_L = None
    best_spectral_radius = np.inf
    best_alpha = None
    
    for alpha in alpha_sweep:
        # Re = alpha * I
        Re = alpha * np.eye(p)
        
        try:
            # Solve discrete-time algebraic Riccati equation (DARE)
            # For dual system: A_dual, B_dual, Qe, Re
            P = solve_discrete_are(A_dual, B_dual, Qe, Re)
            
            # Compute LQR gain: K = (R + B^T P B)^(-1) (B^T P A)
            R_BPB = Re + B_dual.T @ P @ B_dual
            K_dual = np.linalg.solve(R_BPB, B_dual.T @ P @ A_dual)  # (p, n)
            
            # Observer gain: L = K_dual^T
            L_candidate = K_dual.T  # (n, p)
            
            # Verify observer poles
            A_L = Ad - L_candidate @ Cd_new
            observer_poles = np.linalg.eigvals(A_L)
            spectral_radius = np.max(np.abs(observer_poles))
            
            # Track best result
            if spectral_radius < best_spectral_radius:
                best_spectral_radius = spectral_radius
                best_L = L_candidate
                best_alpha = alpha
            
            # Accept if stable and reasonable performance
            if spectral_radius < 1.0:
                L = L_candidate
                observer_poles = observer_poles
                break
                
        except Exception as e:
            continue
    
    # Final validation
    if best_L is None:
        raise RuntimeError("Dual LQR failed: Could not solve DARE for any alpha value.")
    
    if best_spectral_radius >= 1.0:
        raise RuntimeError(
            f"Dual LQR failed: Best spectral radius = {best_spectral_radius:.6f} >= 1.0. "
            f"Observer is unstable."
        )
    
    # Use best result
    L = best_L
    A_L = Ad - L @ Cd_new
    observer_poles = np.linalg.eigvals(A_L)
    spectral_radius = np.max(np.abs(observer_poles))
    
    design_info = {
        'method': 'dual_lqr',
        'design_policy': 'Dual LQR with Re = alpha*I, Qe = I',
        'alpha_used': best_alpha,
        'alpha_sweep': alpha_sweep,
        'achieved_poles': observer_poles,
        'observer_poles': observer_poles,  # Alias for compatibility
        'spectral_radius': spectral_radius,
        'max_achieved_pole': spectral_radius,
        'max_pole_magnitude': spectral_radius,  # Alias for compatibility
        'is_stable': spectral_radius < 1.0,
        'L': L,
        'L_shape': L.shape
    }
    
    return L, design_info


def design_observer(Ad, Cd_new, method='pole_placement', fallback_to_lqr=True, **kwargs):
    """
    Main function to design observer gain L.
    
    If pole placement fails, automatically falls back to dual LQR if fallback_to_lqr=True.
    
    Args:
        Ad: (n, n) discrete-time state matrix
        Cd_new: (p, n) output matrix
        method: Design method ('pole_placement' or 'dual_lqr')
        fallback_to_lqr: If True, automatically fall back to dual LQR if pole placement fails
        **kwargs: Additional arguments for design method
            - For pole_placement: desired_poles, pole_range, method, maxiter, rtol, stability_margin, use_balancing
            - For dual_lqr: Qe, Re, alpha_sweep
    
    Returns:
        tuple: (L, design_info)
    """
    if method == 'pole_placement':
        try:
            return design_observer_pole_placement(Ad, Cd_new, **kwargs)
        except RuntimeError as e:
            if fallback_to_lqr:
                # Automatic fallback to dual LQR
                return design_observer_dual_lqr(Ad, Cd_new, **kwargs)
            else:
                raise
    elif method == 'dual_lqr':
        return design_observer_dual_lqr(Ad, Cd_new, **kwargs)
    else:
        raise ValueError(f"Unknown design method: {method}")


def compute_observability_rank(Ad, Cd_new, tol=None):
    """
    Compute observability rank for (Ad, Cd_new).
    
    Wrapper around Part 1's analyze_observability function.
    
    Args:
        Ad: (n, n) discrete-time state matrix
        Cd_new: (p, n) output matrix
        tol: Optional tolerance for rank computation
    
    Returns:
        dict: Results from observability analysis
    """
    return analyze_observability(Ad, Cd_new, tol=tol)


if __name__ == '__main__':
    """
    Main section: Demonstrate observability check and observer design.
    """
    import sys
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'utils'))
    from build_model import build_continuous_model, discretize_zoh
    
    # Load model
    print("Loading model from Part 0 utilities...")
    A, B, C = build_continuous_model()
    Ad, Bd, Cd, Dd = discretize_zoh(A, B, C, Ts=0.01)
    
    # Get Part 2 C matrix
    print("\nPart 2: Using augmented sensor matrix Cd_new (measuring x1 and x6)")
    Cd_new = get_part2_C_matrix()
    print(f"Cd_new shape: {Cd_new.shape}")
    print(f"Cd_new = \n{Cd_new}")
    
    # Check observability
    print("\n" + "="*60)
    print("Step 1: Observability Analysis")
    print("="*60)
    obsv_results = compute_observability_rank(Ad, Cd_new)
    
    print(f"\nObservability Results:")
    print(f"  System dimension (n): {obsv_results['n']}")
    print(f"  Number of outputs (p): {obsv_results['p']}")
    print(f"  Observability matrix shape: {obsv_results['O'].shape}")
    print(f"  Rank: {obsv_results['rank']} / {obsv_results['n']}")
    print(f"  Tolerance used: {obsv_results['tol_used']:.2e}")
    print(f"  Is observable: {obsv_results['is_observable']}")
    print(f"  Observable subspace dimension: {obsv_results['dim_observable']}")
    print(f"  Unobservable subspace dimension: {obsv_results['dim_unobservable']}")
    
    if not obsv_results['is_observable']:
        print("\nWARNING: System is not fully observable!")
        print("Observer design may not be able to estimate all states.")
    else:
        print("\nSystem is fully observable - observer design can proceed.")
    
    # Design observer
    print("\n" + "="*60)
    print("Step 2: Observer Design (Pole Placement)")
    print("="*60)
    
    try:
        L, design_info = design_observer(Ad, Cd_new, method='pole_placement', pole_range=(0.4, 0.8))
        
        print(f"\nObserver Gain L:")
        print(f"  Shape: {design_info['L_shape']}")
        print(f"  Design method: {design_info['method']}")
        if design_info['method'] == 'pole_placement_dual':
            print(f"  Placement method: {design_info['placement_method']}")
            print(f"  Design policy: {design_info['design_policy']}")
            print(f"  Balancing used: {design_info['balancing_used']}")
            if design_info['balancing_used']:
                print(f"  Condition number (before): {design_info['cond_number_before']:.2e}")
                print(f"  Condition number (after): {design_info['cond_number_after']:.2e}")
            print(f"  Dual controllability rank: {design_info['ctrl_rank']}/{Ad.shape[0]}")
            print(f"  Dual controllability min SV: {design_info['ctrl_min_sv']:.2e}")
        elif design_info['method'] == 'dual_lqr':
            print(f"  Alpha used: {design_info['alpha_used']}")
            print(f"  Alpha sweep: {design_info['alpha_sweep']}")
        
        print(f"\nPole Consistency Check:")
        if 'desired_poles' in design_info:
            print(f"  Requested poles (max magnitude): {design_info['max_desired_pole']:.6f}")
        achieved_poles = design_info['achieved_poles']
        print(f"  Achieved poles eig(Ad - L@Cd_new) (max magnitude): {design_info['max_achieved_pole']:.6f}")
        print(f"  Spectral radius: {design_info['spectral_radius']:.6f}")
        print(f"  Observer is stable: {design_info['is_stable']}")
        
        print(f"\nObserver poles (eigenvalues of Ad - L @ Cd_new):")
        for i, pole in enumerate(achieved_poles):
            print(f"  λ_{i+1} = {pole:.6f} (magnitude: {np.abs(pole):.6f})")
        
        # Save results
        output_dir = os.path.join(os.path.dirname(__file__), 'outputs')
        os.makedirs(output_dir, exist_ok=True)
        
        results_file = os.path.join(output_dir, 'observer_design_results.txt')
        with open(results_file, 'w') as f:
            f.write("Part 2: Observer Design Results\n")
            f.write("="*60 + "\n\n")
            f.write(f"C Matrix (Cd_new):\n")
            f.write(f"Shape: {Cd_new.shape}\n")
            for i, row in enumerate(Cd_new):
                f.write(f"Row {i+1}: {row}\n")
            f.write("\n")
            f.write("Observability Analysis:\n")
            f.write(f"  Rank: {obsv_results['rank']} / {obsv_results['n']}\n")
            f.write(f"  Is observable: {obsv_results['is_observable']}\n")
            f.write(f"  Tolerance used: {obsv_results['tol_used']:.2e}\n")
            f.write("\n")
            f.write("Observer Design:\n")
            f.write(f"  Method: {design_info['method']}\n")
            if design_info['method'] == 'pole_placement_dual':
                f.write(f"  Placement method: {design_info['placement_method']}\n")
                f.write(f"  Design policy: {design_info['design_policy']}\n")
                f.write(f"  Balancing used: {design_info['balancing_used']}\n")
                if design_info['balancing_used']:
                    f.write(f"  Condition number (before balancing): {design_info['cond_number_before']:.2e}\n")
                    f.write(f"  Condition number (after balancing): {design_info['cond_number_after']:.2e}\n")
                f.write(f"  Dual controllability rank: {design_info['ctrl_rank']}/{Ad.shape[0]}\n")
                f.write(f"  Dual controllability min singular value: {design_info['ctrl_min_sv']:.2e}\n")
                f.write(f"  Requested poles (desired): max magnitude = {design_info['max_desired_pole']:.6f}\n")
            elif design_info['method'] == 'dual_lqr':
                f.write(f"  Alpha used: {design_info['alpha_used']}\n")
                f.write(f"  Alpha sweep: {design_info['alpha_sweep']}\n")
            
            f.write(f"  Observer gain L shape: {design_info['L_shape']}\n")
            f.write(f"\nPole Consistency Check:\n")
            f.write(f"  Spectral radius (max(abs(eig(Ad - L@Cd_new)))): {design_info['spectral_radius']:.6f}\n")
            f.write(f"  Is stable (spectral_radius < 1.0): {design_info['is_stable']}\n")
            f.write("\nAchieved Observer Poles (eig(Ad - L @ Cd_new)):\n")
            for i, pole in enumerate(design_info['achieved_poles']):
                f.write(f"  λ_{i+1} = {pole:.6f} (|λ| = {np.abs(pole):.6f})\n")
            if 'desired_poles' in design_info:
                f.write("\nRequested Poles (desired):\n")
                for i, pole in enumerate(design_info['desired_poles']):
                    f.write(f"  λ_{i+1}_desired = {pole:.6f} (|λ| = {np.abs(pole):.6f})\n")
            f.write("\nObserver Gain L (12×2):\n")
            for i in range(L.shape[0]):
                f.write(f"  Row {i+1}: [{L[i, 0]:.6e}, {L[i, 1]:.6e}]\n")
        
        print(f"\nResults saved to: {results_file}")
        
    except Exception as e:
        print(f"\nERROR: Observer design failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

