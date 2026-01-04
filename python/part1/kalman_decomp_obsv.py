"""
Observability-based Kalman decomposition.

This module provides functions to perform an observability-based Kalman decomposition
that separates a system into observable and unobservable subspaces using a similarity transform.
"""

import numpy as np


def construct_observable_basis(O, rank):
    """
    Construct basis for the observable subspace from the observability matrix.
    
    Uses QR decomposition on O^T to extract a basis for the column space of O^T,
    which corresponds to the observable subspace.
    
    Args:
        O: (m, n) observability matrix
        rank: Rank of O (number of linearly independent columns)
    
    Returns:
        To: (n, rank) matrix whose columns form a basis for the observable subspace
    """
    n = O.shape[1]
    
    # Use QR decomposition on O^T to get basis for column space
    # Q from QR(O^T) gives orthonormal basis for observable subspace
    Q, R = np.linalg.qr(O.T, mode='reduced')
    
    # Extract first 'rank' columns as basis for observable subspace
    # (QR gives us an orthonormal basis, first 'rank' columns span the observable space)
    To = Q[:, :rank]
    
    return To


def construct_unobservable_basis(O, To, rank):
    """
    Construct basis for the unobservable subspace.
    
    The unobservable subspace is the null space of O (or equivalently, the orthogonal
    complement of the column space of O^T).
    
    Args:
        O: (m, n) observability matrix
        To: (n, rank) basis for observable subspace
        rank: Rank of O (dimension of observable subspace)
    
    Returns:
        Tuo: (n, n-rank) matrix whose columns form a basis for the unobservable subspace
    """
    n = O.shape[1]
    n_unobs = n - rank
    
    if n_unobs == 0:
        # System is fully observable, no unobservable subspace
        return np.array([]).reshape(n, 0)
    
    # Compute null space of O using SVD
    # Null space of O is the right null space, i.e., vectors x such that O @ x = 0
    U, sigma, Vh = np.linalg.svd(O, full_matrices=True)
    
    # Vh is (n, n) where rows are right singular vectors (Vh = V^T)
    # The null space is spanned by the last (n - rank) right singular vectors
    # These correspond to the last (n - rank) rows of Vh
    # Extract as columns: Vh[rank:, :].T gives (n, n-rank)
    Tuo = Vh[rank:, :].T  # Transpose to get (n, n-rank) matrix
    
    # Ensure orthogonality with observable basis (Gram-Schmidt style)
    # Project out observable components from unobservable basis
    for i in range(n_unobs):
        # Orthogonalize Tuo[:, i] against To
        for j in range(rank):
            Tuo[:, i] = Tuo[:, i] - np.dot(Tuo[:, i], To[:, j]) * To[:, j]
        # Normalize
        norm = np.linalg.norm(Tuo[:, i])
        if norm > 1e-12:
            Tuo[:, i] = Tuo[:, i] / norm
    
    return Tuo


def kalman_decomposition_observable(Ad, Cd, tol=None):
    """
    Perform observability-based Kalman decomposition.
    
    Constructs a similarity transform T = [To, Tuo] that partitions the system
    into observable and unobservable components. The transformed system has:
        Abar = inv(T) * Ad * T
        Cbar = Cd * T
    
    Structure of transformed matrices:
        Abar = [Aoo  Aou]
               [0    Auu]
        Cbar = [Cbar_o  0]
    
    where Aoo is the observable block, Auu is the unobservable block.
    
    Args:
        Ad: (n, n) discrete-time state matrix
        Cd: (p, n) discrete-time output matrix
        tol: Optional tolerance for rank computation. If None, uses automatic policy.
    
    Returns:
        dict: Dictionary containing:
            - 'T': (n, n) transformation matrix [To, Tuo]
            - 'Tinv': (n, n) inverse of T
            - 'Abar': (n, n) transformed state matrix
            - 'Cbar': (p, n) transformed output matrix
            - 'Aoo': (r, r) observable block
            - 'Auu': (n-r, n-r) unobservable block
            - 'Aou': (r, n-r) coupling term (may be nonzero)
            - 'Cbar_o': (p, r) observable output block
            - 'eig_obs': Eigenvalues of observable block Aoo
            - 'eig_unobs': Eigenvalues of unobservable block Auu
            - 'cond_T': Condition number of transformation matrix T
            - 'rank': Rank of observability matrix
    """
    from observability_rank import analyze_observability
    
    # First, compute observability analysis to get rank
    obsv_results = analyze_observability(Ad, Cd, tol=tol)
    O = obsv_results['O']
    rank = obsv_results['rank']
    n = obsv_results['n']
    
    # Construct bases
    To = construct_observable_basis(O, rank)
    Tuo = construct_unobservable_basis(O, To, rank)
    
    # Form transformation matrix
    if Tuo.size > 0:
        T = np.hstack([To, Tuo])
    else:
        # System is fully observable
        T = To
    
    # Verify T is square and invertible
    cond_T = np.linalg.cond(T)
    
    # Compute inverse
    Tinv = np.linalg.inv(T)
    
    # Compute transformed matrices
    Abar = Tinv @ Ad @ T
    Cbar = Cd @ T
    
    # Extract blocks
    r = rank
    Aoo = Abar[0:r, 0:r]
    
    if r < n:
        Auu = Abar[r:n, r:n]
        Aou = Abar[0:r, r:n]  # Coupling term (may be nonzero)
        Cbar_o = Cbar[:, 0:r]
    else:
        # Fully observable system
        Auu = np.array([]).reshape(0, 0)
        Aou = np.array([]).reshape(r, 0)
        Cbar_o = Cbar
    
    # Compute eigenvalues
    eig_obs = np.linalg.eigvals(Aoo)
    if Auu.size > 0:
        eig_unobs = np.linalg.eigvals(Auu)
    else:
        eig_unobs = np.array([])
    
    # Check orthonormality of T (T^T @ T should be approximately identity)
    T_ortho_error = np.max(np.abs(T.T @ T - np.eye(T.shape[1])))
    T_is_orthonormal = T_ortho_error < 1e-10
    
    return {
        'T': T,
        'Tinv': Tinv,
        'Abar': Abar,
        'Cbar': Cbar,
        'Aoo': Aoo,
        'Auu': Auu,
        'Aou': Aou,
        'Cbar_o': Cbar_o,
        'eig_obs': eig_obs,
        'eig_unobs': eig_unobs,
        'cond_T': cond_T,
        'T_ortho_error': T_ortho_error,
        'T_is_orthonormal': T_is_orthonormal,
        'rank': rank
    }

