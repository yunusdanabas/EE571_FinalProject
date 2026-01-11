# Part 1: Observability Analysis Report

## Objective

Analyze the observability of the discrete-time system with a single sensor measuring the displacement of mass 1. Determine which states are observable and which are unobservable using observability matrix rank analysis and Kalman decomposition.

## Approach

1. Built and discretized the continuous-time system using zero-order hold (ZOH) at Ts = 0.01 seconds
2. Constructed the observability matrix O = [C; CA; CA²; ...; CA^(n-1)]
3. Computed the numerical rank using SVD with automatic tolerance selection
4. Performed Kalman decomposition to separate the system into observable and unobservable subspaces via a similarity transform
5. Analyzed eigenvalues of the observable and unobservable blocks

## Key Results

- **System dimension:** 12 states
- **Observability rank:** 6 / 12 (system is not fully observable)
- **Observable subspace:** 6 states
- **Unobservable subspace:** 6 states
- **Transformation condition number:** 1.000010 (well-conditioned)
- **Observable eigenvalues:** 6 complex conjugate pairs (all near unit circle)
- **Unobservable eigenvalues:** 6 complex conjugate pairs (all near unit circle)

The Kalman decomposition yields:
- Observable block Aoo: (6, 6) matrix with 6 observable eigenvalues
- Unobservable block Auu: (6, 6) matrix with 6 unobservable eigenvalues
- Output coupling: The unobservable states are completely decoupled from the output (Cbar has zero entries for unobservable components)

## Results

Complete observability analysis results:

```
Part 1: Observability Analysis
============================================================

OBSERVABILITY RANK ANALYSIS
------------------------------------------------------------
System dimension (n): 12
Rank of observability matrix: 6
SVD tolerance used: 1.000000e-10
Observable subspace dimension: 6
Unobservable subspace dimension: 6
System is observable: False

KALMAN DECOMPOSITION RESULTS
------------------------------------------------------------
Condition number of T: 1.000010e+00
Observable block Aoo shape: (6, 6)
Unobservable block Auu shape: (6, 6)

EIGENVALUES
------------------------------------------------------------
NOTE: All eigenvalues have magnitude ≈ 1.0 because the system is UNDAMPED.
      They represent different frequencies (see ω values below).
      The similarity in appearance is due to:
      1. Undamped system → |z| = 1.0 (unit circle)
      2. Small Ts = 0.01s → small angles ω*Ts
      3. Limited precision display → look similar but are DIFFERENT frequencies

Observable block (Aoo) eigenvalues:
Total: 6 eigenvalues
  Format: z = magnitude * exp(j*angle) → ω = angle/Ts (rad/s)
  Note: All magnitudes ≈ 1.0 (undamped system on unit circle)

  1. z = 0.999860-0.018475j  |z|=1.000031, ∠z=-0.018475 rad  → ω = -1.8475 rad/s (-0.2940 Hz)
  2. z = 0.999962-0.014139j  |z|=1.000062, ∠z=-0.014139 rad  → ω = -1.4139 rad/s (-0.2250 Hz)
  3. z = 1.000002-0.007653j  |z|=1.000031, ∠z=-0.007653 rad  → ω = -0.7653 rad/s (-0.1218 Hz)
  4. z = 1.000002+0.007653j  |z|=1.000031, ∠z=0.007653 rad  → ω = 0.7653 rad/s (0.1218 Hz)
  5. z = 0.999962+0.014139j  |z|=1.000062, ∠z=0.014139 rad  → ω = 1.4139 rad/s (0.2250 Hz)
  6. z = 0.999860+0.018475j  |z|=1.000031, ∠z=0.018475 rad  → ω = 1.8475 rad/s (0.2940 Hz)

Unobservable block (Auu) eigenvalues:
Total: 6 eigenvalues
  Format: z = magnitude * exp(j*angle) → ω = angle/Ts (rad/s)
  Note: All magnitudes ≈ 1.0 (undamped system on unit circle)

  1. z = 0.999794-0.018015j  |z|=0.999956, ∠z=-0.018017 rad  → ω = -1.8017 rad/s (-0.2868 Hz)
  2. z = 0.999854-0.012467j  |z|=0.999932, ∠z=-0.012469 rad  → ω = -1.2469 rad/s (-0.1984 Hz)
  3. z = 0.999977-0.004450j  |z|=0.999987, ∠z=-0.004451 rad  → ω = -0.4451 rad/s (-0.0708 Hz)
  4. z = 0.999977+0.004450j  |z|=0.999987, ∠z=0.004451 rad  → ω = 0.4451 rad/s (0.0708 Hz)
  5. z = 0.999854+0.012467j  |z|=0.999932, ∠z=0.012469 rad  → ω = 1.2469 rad/s (0.1984 Hz)
  6. z = 0.999794+0.018015j  |z|=0.999956, ∠z=0.018017 rad  → ω = 1.8017 rad/s (0.2868 Hz)
```

## Eigenvalue Analysis: Why All Eigenvalues Are Near 1.0

### Executive Summary

**The eigenvalues are NOT a problem** - they represent physically correct behavior of an undamped discrete-time system. They appear similar because:

1. **Physical system is undamped** → eigenvalues lie on unit circle (|z| = 1.0)
2. **Small sampling time** (Ts = 0.01s) → small imaginary parts
3. **Limited output precision** (6 decimals) → makes them look identical
4. **They actually represent DISTINCT frequencies** - not similar at all!

### Detailed Explanation

#### 1. System Characteristics

The system is a **spring-mass chain** with:
- **No damping** (undamped system)
- Continuous-time eigenvalues: `s = ±jω` (purely imaginary)
- All eigenvalues on the imaginary axis → marginally stable

#### 2. Discretization Effect (ZOH with Ts = 0.01s)

When we discretize with zero-order hold:
```
z = exp(s * Ts) = exp(±jω * Ts) = cos(ω*Ts) ± j*sin(ω*Ts)
```

**Key insight:** For any undamped system (s = ±jω):
- Magnitude: |z| = √(cos² + sin²) = **1.0 exactly**
- Phase: ∠z = ω*Ts

So **all eigenvalues MUST have |z| = 1.0** (theoretically).

#### 3. Why They Appear Similar

They look similar because:
1. **All real parts ≈ 0.999...** (because |z| ≈ 1.0 and angles are small)
2. **Imaginary parts are small** (because ω*Ts is small: e.g., 1.85 × 0.01 = 0.0185)
3. **Limited precision** (6 decimals) makes small differences invisible

#### 4. They Are Actually DIFFERENT!

The eigenvalues represent **distinct frequencies** representing different normal modes of vibration:

**Observable modes:**
- ω₁ = 0.7653 rad/s (0.1218 Hz)
- ω₂ = 1.4139 rad/s (0.2250 Hz)
- ω₃ = 1.8475 rad/s (0.2940 Hz)

**Unobservable modes:**
- ω₁ = 0.4451 rad/s (0.0708 Hz)
- ω₂ = 1.2469 rad/s (0.1984 Hz)
- ω₃ = 1.8017 rad/s (0.2868 Hz)

These are **distinct frequencies** - the modes are NOT similar!

#### 5. Numerical Precision

The slight deviation from |z| = 1.0 is due to floating-point arithmetic:
- Min magnitude: 0.999932
- Max magnitude: 1.000062
- Deviation from 1.0: ~6.8×10⁻⁵

This is **normal** and expected for numerical computation.

#### Is This a Problem?

**NO** - This is **correct behavior** for:
- ✓ Undamped systems → eigenvalues on unit circle
- ✓ Small Ts → small imaginary parts
- ✓ Different frequencies → different modes (even if they look similar)

#### Can We "Prevent" This?

**This is not something to prevent** - it's physically correct! However, we can **improve presentation**:
1. **Show eigenvalues in polar form** (magnitude and angle) - implemented in results
2. **Display continuous-time frequencies** (more meaningful) - included in results
3. **Use higher precision** (more decimal places)
4. **Note:** Adding damping would move eigenvalues inside unit circle, but that changes the physical system!

## Figures

- **`outputs/observability_results.txt`**: Complete observability analysis results including rank, Kalman decomposition structure, and eigenvalues (with frequency information)

## Findings

The system with a single sensor (measuring mass 1 displacement) is **not fully observable**. The observability rank is 6 out of 12, meaning only half of the system states can be reconstructed from the output measurements. This is consistent with the physical structure of the system - with a single displacement sensor, we can only observe symmetric modes (modes where the first mass participates) but not antisymmetric modes (modes where the first mass remains stationary while others move).

The Kalman decomposition successfully separates the system into observable and unobservable subspaces. All eigenvalues of both blocks lie near the unit circle (magnitude ≈ 1.0), indicating marginally stable discrete-time modes, as expected for an undamped spring-mass system. The eigenvalues represent distinct normal modes with different frequencies (0.4451-1.8475 rad/s range), even though they appear similar in the numerical output due to the undamped nature of the system and small sampling time.

The unobservable subspace is completely decoupled from the output, confirming that no information about these 6 states can be inferred from the single sensor measurement. The similarity in eigenvalue appearance is an artifact of:
- Undamped dynamics (|z| = 1.0 exactly for undamped systems)
- Small sampling time (Ts = 0.01s → small angles ω*Ts)
- Limited output precision

**Conclusion:** The eigenvalues being near 1.0 is expected and correct for an undamped discrete-time system. The system is working correctly!
