"""
Validation script to compare Python discretization with MATLAB reference.

This script loads discrete-time matrices from MATLAB (if exported) and
compares them with Python discretization results using tolerance checks.
"""

import numpy as np
import os
import sys

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from utils.build_model import build_continuous_model, discretize_zoh, load_reference_matrices


def compare_matrices(Ad_py, Bd_py, Cd_py, Dd_py, Ad_matlab, Bd_matlab, Cd_matlab, Dd_matlab,
                     rtol=1e-10, atol=1e-12):
    """
    Compare Python and MATLAB discrete-time matrices.
    
    Includes Dd matrix comparison even though it should be all zeros,
    to catch shape mistakes early.
    
    Tolerance Policy:
        - Primary: rtol=1e-10, atol=1e-12 (strict)
        - Fallback: rtol=1e-9, atol=1e-11 (if strict tolerance fails on different machines)
    
    Args:
        Ad_py, Bd_py, Cd_py, Dd_py: Python discretization results
        Ad_matlab, Bd_matlab, Cd_matlab, Dd_matlab: MATLAB discretization results
        rtol: Relative tolerance (default: 1e-10, fallback: 1e-9)
        atol: Absolute tolerance (default: 1e-12, fallback: 1e-11)
    
    Returns:
        dict: Comparison results including max_abs_diff for all matrices
    """
    results = {}
    
    # Compare Ad
    Ad_close = np.allclose(Ad_py, Ad_matlab, rtol=rtol, atol=atol)
    Ad_max_abs_diff = np.max(np.abs(Ad_py - Ad_matlab))
    Ad_max_rel_diff = np.max(np.abs((Ad_py - Ad_matlab) / (Ad_matlab + 1e-15)))
    
    results['Ad'] = {
        'close': Ad_close,
        'max_abs_diff': Ad_max_abs_diff,
        'max_rel_diff': Ad_max_rel_diff
    }
    
    # Compare Bd
    Bd_close = np.allclose(Bd_py, Bd_matlab, rtol=rtol, atol=atol)
    Bd_max_abs_diff = np.max(np.abs(Bd_py - Bd_matlab))
    Bd_max_rel_diff = np.max(np.abs((Bd_py - Bd_matlab) / (Bd_matlab + 1e-15)))
    
    results['Bd'] = {
        'close': Bd_close,
        'max_abs_diff': Bd_max_abs_diff,
        'max_rel_diff': Bd_max_rel_diff
    }
    
    # Compare Cd
    Cd_close = np.allclose(Cd_py, Cd_matlab, rtol=rtol, atol=atol)
    Cd_max_abs_diff = np.max(np.abs(Cd_py - Cd_matlab))
    Cd_max_rel_diff = np.max(np.abs((Cd_py - Cd_matlab) / (Cd_matlab + 1e-15)))
    
    results['Cd'] = {
        'close': Cd_close,
        'max_abs_diff': Cd_max_abs_diff,
        'max_rel_diff': Cd_max_rel_diff
    }
    
    # Compare Dd (should be all zeros, but check to catch shape mistakes)
    Dd_close = np.allclose(Dd_py, Dd_matlab, rtol=rtol, atol=atol)
    Dd_max_abs_diff = np.max(np.abs(Dd_py - Dd_matlab))
    Dd_max_rel_diff = np.max(np.abs((Dd_py - Dd_matlab) / (Dd_matlab + 1e-15)))
    
    results['Dd'] = {
        'close': Dd_close,
        'max_abs_diff': Dd_max_abs_diff,
        'max_rel_diff': Dd_max_rel_diff
    }
    
    results['all_close'] = Ad_close and Bd_close and Cd_close and Dd_close
    
    return results


def main():
    """Main validation routine."""
    print("=" * 60)
    print("MATLAB-Python Discretization Comparison")
    print("=" * 60)
    
    # Get Python discretization
    print("\n1. Computing Python discretization...")
    A, B, C = build_continuous_model()
    Ad_py, Bd_py, Cd_py, Dd_py = discretize_zoh(A, B, C, Ts=0.01)
    
    # Try to load MATLAB matrices
    mat_file = os.path.join(os.path.dirname(__file__), '..', '..', 
                           'matlab', 'discrete_matrices.mat')
    
    if not os.path.exists(mat_file):
        print(f"\n   MATLAB matrices not found at: {mat_file}")
        print("   Run matlab/export_matrices.m first to enable comparison.")
        print("   Skipping MATLAB comparison.")
        return
    
    print(f"\n2. Loading MATLAB matrices from: {mat_file}")
    try:
        Ad_matlab, Bd_matlab, Cd_matlab, Dd_matlab = load_reference_matrices(mat_file)
        print("   Successfully loaded MATLAB matrices (Ad, Bd, Cd, Dd)")
    except Exception as e:
        print(f"   Error loading MATLAB matrices: {e}")
        return
    
    # Compare matrices with strict tolerance
    print("\n3. Comparing matrices (rtol=1e-10, atol=1e-12)...")
    results = compare_matrices(Ad_py, Bd_py, Cd_py, Dd_py,
                               Ad_matlab, Bd_matlab, Cd_matlab, Dd_matlab)
    
    # Print results
    for matrix_name in ['Ad', 'Bd', 'Cd', 'Dd']:
        r = results[matrix_name]
        print(f"\n   {matrix_name}:")
        print(f"     Matches (within tolerance): {r['close']}")
        print(f"     Max absolute difference: {r['max_abs_diff']:.2e}")
        print(f"     Max relative difference: {r['max_rel_diff']:.2e}")
    
    print(f"\n   All matrices match: {results['all_close']}")
    
    if results['all_close']:
        print("\n   SUCCESS: Python discretization matches MATLAB within tolerance!")
    else:
        print("\n   WARNING: Some matrices do not match within strict tolerance.")
        print("   Attempting fallback tolerance (rtol=1e-9, atol=1e-11)...")
        # Try fallback tolerance
        results_fallback = compare_matrices(Ad_py, Bd_py, Cd_py, Dd_py,
                                           Ad_matlab, Bd_matlab, Cd_matlab, Dd_matlab,
                                           rtol=1e-9, atol=1e-11)
        if results_fallback['all_close']:
            print("   Matches with fallback tolerance.")
        else:
            print("   Still does not match with fallback tolerance.")
            print("   This may indicate a discretization algorithm difference.")
            print("   Review max_abs_diff values above to assess numerical differences.")


if __name__ == '__main__':
    main()

