# Part 2: Discretization of Error Model

**Status: COMPLETED**

## Scope

Discretize the continuous-time error model `(Ac, Bc)` to obtain `(Ad, Bd)` using zero-order hold (ZOH). This discrete model will be used by both regulators in subsequent parts.

## Python Implementation

```python
import numpy as np
from scipy.linalg import expm

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

# Usage
Ts = 0.02  # 50 Hz sampling
Ad, Bd = c2d_zoh_exact(Ac, Bc, Ts)

# Verify dimensions
assert Ad.shape == (5, 5), "Ad must be 5x5"
assert Bd.shape == (5, 2), "Bd must be 5x2"

# Compute eigenvalues for reference
eig_Ad = np.linalg.eigvals(Ad)
print("Eigenvalues of Ad:", eig_Ad)
```

## Acceptance Checks

- [x] `Ad` and `Bd` are computed correctly
- [x] `Ad` is 5x5 and `Bd` is 5x2
- [x] Discretization uses ZOH method
- [x] Matrix dimensions are verified

## Notes

- Both Part 3 (LQR) and Part 4 (Pole Placement) will use the same `Ad`, `Bd` matrices
- The discretization is exact (not an approximation)
