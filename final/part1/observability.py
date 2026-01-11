import numpy as np
import os
import sys

# Add workspace root to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

from final.utils.model import build_continuous_model, discretize_zoh


def build_observability_matrix(Ad, Cd):
    """Construct observability matrix O = [C; CA; CA^2; ...; CA^(n-1)]."""
    n = Ad.shape[0]
    p = Cd.shape[0]
    O = np.zeros((n * p, n))
    
    O[0:p, :] = Cd
    Ad_power = np.eye(n)
    for i in range(1, n):
        Ad_power = Ad_power @ Ad
        O[i*p:(i+1)*p, :] = Cd @ Ad_power
    
    return O


def compute_rank(O, tol=None):
    """Compute numerical rank using SVD."""
    _, sigma, _ = np.linalg.svd(O, full_matrices=False)
    
    if tol is None:
        machine_eps = np.finfo(O.dtype).eps
        tol = max(1e-10, machine_eps * np.max(sigma))
    
    threshold = tol * np.max(sigma) if np.max(sigma) > 0 else 0
    rank = np.sum(sigma > threshold)
    return rank, tol


def kalman_decomposition(Ad, Cd, tol=None):
    """Perform Kalman decomposition to separate observable/unobservable subspaces."""
    # Compute observability matrix and rank
    O = build_observability_matrix(Ad, Cd)
    rank, tol_used = compute_rank(O, tol)
    n = Ad.shape[0]
    
    # Construct observable basis using QR on O^T
    Q, _ = np.linalg.qr(O.T, mode='reduced')
    To = Q[:, :rank]
    
    # Construct unobservable basis from null space of O
    if rank < n:
        _, _, Vh = np.linalg.svd(O, full_matrices=True)
        Tuo = Vh[rank:, :].T
        
        # Orthogonalize against observable basis
        for i in range(n - rank):
            for j in range(rank):
                Tuo[:, i] = Tuo[:, i] - np.dot(Tuo[:, i], To[:, j]) * To[:, j]
            norm = np.linalg.norm(Tuo[:, i])
            if norm > 1e-12:
                Tuo[:, i] = Tuo[:, i] / norm
        T = np.hstack([To, Tuo])
    else:
        T = To
        Tuo = np.array([]).reshape(n, 0)
    
    # Compute transformed system
    Tinv = np.linalg.inv(T)
    Abar = Tinv @ Ad @ T
    Cbar = Cd @ T
    
    # Extract blocks
    r = rank
    Aoo = Abar[0:r, 0:r]
    if rank < n:
        Auu = Abar[r:n, r:n]
        Cbar_o = Cbar[:, 0:r]
    else:
        Auu = np.array([]).reshape(0, 0)
        Cbar_o = Cbar
    
    # Compute eigenvalues
    eig_obs = np.linalg.eigvals(Aoo)
    eig_unobs = np.linalg.eigvals(Auu) if Auu.size > 0 else np.array([])
    
    return {
        'rank': rank,
        'tol_used': tol_used,
        'n': n,
        'O': O,
        'Aoo': Aoo,
        'Auu': Auu,
        'eig_obs': eig_obs,
        'eig_unobs': eig_unobs,
        'cond_T': np.linalg.cond(T)
    }


def format_eigenvalues(eigvals, Ts=0.01):
    """Format eigenvalues with magnitude, angle, and frequency information."""
    if len(eigvals) == 0:
        return "None"
    
    # Sort by angle (frequency) for better interpretation
    sorted_eig = sorted(eigvals, key=lambda x: np.angle(x))
    result = f"Total: {len(sorted_eig)} eigenvalues\n"
    result += f"  Format: z = magnitude * exp(j*angle) → ω = angle/Ts (rad/s)\n"
    result += f"  Note: All magnitudes ≈ 1.0 (undamped system on unit circle)\n"
    result += "\n"
    
    for i, eig in enumerate(sorted_eig):
        magnitude = np.abs(eig)
        angle = np.angle(eig)
        freq = angle / Ts if Ts > 0 else 0
        if np.iscomplexobj(eig):
            result += f"  {i+1}. z = {eig.real:.6f}{eig.imag:+.6f}j  "
            result += f"|z|={magnitude:.6f}, ∠z={angle:.6f} rad  "
            result += f"→ ω = {freq:.4f} rad/s ({freq/(2*np.pi):.4f} Hz)\n"
        else:
            result += f"  {i+1}. z = {eig.real:.6f}, |z|={magnitude:.6f}\n"
    
    return result


def main():
    """Perform observability analysis and Kalman decomposition."""
    Ts = 0.01
    
    # Build and discretize system
    A, B, C = build_continuous_model()
    Ad, Bd, Cd, Dd = discretize_zoh(A, B, C, Ts)
    
    # Perform observability analysis
    decomp = kalman_decomposition(Ad, Cd)
    
    # Save results
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.join(script_dir, 'outputs')
    os.makedirs(output_dir, exist_ok=True)
    
    output_path = os.path.join(output_dir, 'observability_results.txt')
    with open(output_path, 'w') as f:
        f.write("Part 1: Observability Analysis\n")
        f.write("=" * 60 + "\n\n")
        f.write("OBSERVABILITY RANK ANALYSIS\n")
        f.write("-" * 60 + "\n")
        f.write(f"System dimension (n): {decomp['n']}\n")
        f.write(f"Rank of observability matrix: {decomp['rank']}\n")
        f.write(f"SVD tolerance used: {decomp['tol_used']:.6e}\n")
        f.write(f"Observable subspace dimension: {decomp['rank']}\n")
        f.write(f"Unobservable subspace dimension: {decomp['n'] - decomp['rank']}\n")
        f.write(f"System is observable: {decomp['rank'] == decomp['n']}\n\n")
        
        f.write("KALMAN DECOMPOSITION RESULTS\n")
        f.write("-" * 60 + "\n")
        f.write(f"Condition number of T: {decomp['cond_T']:.6e}\n")
        f.write(f"Observable block Aoo shape: {decomp['Aoo'].shape}\n")
        if decomp['Auu'].size > 0:
            f.write(f"Unobservable block Auu shape: {decomp['Auu'].shape}\n")
        else:
            f.write("Unobservable block: None (system is fully observable)\n")
        f.write("\n")
        
        f.write("EIGENVALUES\n")
        f.write("-" * 60 + "\n")
        f.write("NOTE: All eigenvalues have magnitude ≈ 1.0 because the system is UNDAMPED.\n")
        f.write("      They represent different frequencies (see ω values below).\n")
        f.write("      The similarity in appearance is due to:\n")
        f.write("      1. Undamped system → |z| = 1.0 (unit circle)\n")
        f.write("      2. Small Ts = 0.01s → small angles ω*Ts\n")
        f.write("      3. Limited precision display → look similar but are DIFFERENT frequencies\n\n")
        f.write("Observable block (Aoo) eigenvalues:\n")
        f.write(format_eigenvalues(decomp['eig_obs'], Ts=Ts))
        f.write("\n")
        
        if decomp['eig_unobs'].size > 0:
            f.write("Unobservable block (Auu) eigenvalues:\n")
            f.write(format_eigenvalues(decomp['eig_unobs'], Ts=Ts))
    
    # Print summary
    print(f"Part 1: Observability Analysis complete")
    print(f"  System dimension: {decomp['n']}")
    print(f"  Observability rank: {decomp['rank']} / {decomp['n']}")
    print(f"  Observable eigenvalues: {len(decomp['eig_obs'])}")
    print(f"  Unobservable eigenvalues: {len(decomp['eig_unobs']) if decomp['eig_unobs'].size > 0 else 0}")
    print(f"  Results saved to {output_path}")


if __name__ == '__main__':
    main()
