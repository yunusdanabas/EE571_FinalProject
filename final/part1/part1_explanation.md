# Part 1: Observability Analysis - Detailed Explanation

## Overview

Part 1 analyzes the **observability** of the 6-mass spring system with a single sensor measuring the displacement of mass 1 (x1). Observability determines which states can be estimated from the available measurements.

## What is Observability?

**Observability** is a fundamental property of control systems that answers the question: "Can we determine the internal states of the system by observing its outputs?"

- **Observable states**: Can be determined from output measurements
- **Unobservable states**: Cannot be determined from output measurements (even with perfect measurements)

### Why Does Observability Matter?

If a state is unobservable, we cannot:
- Estimate it from measurements
- Use it for feedback control (if we can't measure or estimate it)
- Monitor it for safety or diagnostics

Observability is a **necessary condition** for state estimation and observer design.

## Code Structure

### Main Components

1. **`build_observability_matrix()`**
   - Constructs the observability matrix: O = [C; CA; CA²; ...; CA^(n-1)]
   - This matrix captures all information available from output measurements

2. **`compute_rank()`**
   - Computes the numerical rank of the observability matrix using SVD
   - Rank = number of observable states

3. **`kalman_decomposition()`**
   - Performs Kalman decomposition to separate observable and unobservable subspaces
   - Transforms the system into a canonical form showing which states are observable

## Step-by-Step Execution

### Step 1: Build Observability Matrix

```python
O = [C; CA; CA²; ...; CA^(n-1)]
```

**What is the observability matrix?**
- Each row block (C, CA, CA², ...) represents information available at different time steps
- C: Direct output (what we measure now)
- CA: Output after one time step (what we can infer)
- CA²: Output after two time steps (more information)
- ...

**Why n-1 powers?**
- For an n-dimensional system, we need at most n-1 time steps to determine all observable states
- This is the Cayley-Hamilton theorem: A^n can be expressed as a linear combination of lower powers

### Step 2: Compute Rank

```python
rank, tol = compute_rank(O)
```

**Numerical rank computation:**
- Uses Singular Value Decomposition (SVD): O = U Σ V^T
- Rank = number of singular values above a threshold
- Threshold accounts for numerical precision (machine epsilon)

**Result**: Rank = 6 out of 12
- This means only 6 states are observable
- The other 6 states are unobservable

### Step 3: Kalman Decomposition

```python
result = kalman_decomposition(Ad, Cd)
```

**What does Kalman decomposition do?**
- Finds a similarity transformation T that separates the system into:
  - **Observable block (Aoo)**: Observable states
  - **Unobservable block (Auu)**: Unobservable states
- The transformed system has the form:
  ```
  [Aoo  Aou]   [observable states]
  [0    Auu]   [unobservable states]
  ```

**Key insight**: The unobservable block is completely decoupled from the output (Cbar has zeros for unobservable components).

### Step 4: Analyze Eigenvalues

The decomposition reveals:
- **6 observable eigenvalues**: Frequencies where mass 1 participates
- **6 unobservable eigenvalues**: Frequencies where mass 1 remains stationary (antisymmetric modes)

## Understanding the Results

### Why Only 6 States Are Observable?

With a single sensor measuring x1:
- **Symmetric modes**: Observable (mass 1 moves, so we can see it)
- **Antisymmetric modes**: Unobservable (mass 1 stays still while others move)

**Physical interpretation:**
- If mass 1 doesn't move, we can't observe what's happening to other masses
- The system has 6 normal modes of vibration
- Half are symmetric (observable), half are antisymmetric (unobservable)

### Eigenvalue Analysis

**All eigenvalues have magnitude ≈ 1.0:**
- This is **correct** for an undamped discrete-time system
- Undamped systems have eigenvalues on the unit circle (|z| = 1.0)
- They represent different frequencies (not the same!)

**Why they look similar:**
1. **Undamped system**: All eigenvalues must have |z| = 1.0
2. **Small sampling time**: Ts = 0.01s makes angles small (ω*Ts is small)
3. **Limited precision**: Display precision makes them look identical

**They are actually different:**
- Observable modes: ω = 0.7653, 1.4139, 1.8475 rad/s
- Unobservable modes: ω = 0.4451, 1.2469, 1.8017 rad/s
- These are **distinct frequencies** representing different vibration modes

### Condition Number

The transformation matrix T has condition number ≈ 1.0:
- This means the decomposition is **well-conditioned**
- The observable and unobservable subspaces are clearly separated
- No numerical issues with the decomposition

## Key Insights

### 1. Observability is a System Property

Observability depends on:
- **Which outputs we measure** (C matrix)
- **System dynamics** (A matrix)
- **Not on the controller** (observability is independent of control)

### 2. Single Sensor is Insufficient

With only one sensor (measuring x1):
- We can observe 6 states (symmetric modes)
- We cannot observe 6 states (antisymmetric modes)
- **Solution**: Add more sensors (Part 2 uses 2 sensors: x1 and x6)

### 3. Kalman Decomposition is Powerful

The decomposition:
- Shows exactly which states are observable/unobservable
- Provides a canonical form for analysis
- Separates the system into observable and unobservable parts

## What Happens Next?

Part 1 reveals that a single sensor is insufficient. In Part 2:
- We add a second sensor (measuring x6)
- This makes the system **fully observable** (all 12 states can be estimated)
- We design an observer to estimate all states from these 2 measurements

## Summary

Part 1 successfully:
1. ✓ Constructed the observability matrix
2. ✓ Computed the rank (6 out of 12 states observable)
3. ✓ Performed Kalman decomposition
4. ✓ Identified observable and unobservable subspaces
5. ✓ Analyzed eigenvalues and frequencies

**Key finding**: With a single sensor (x1), only 6 out of 12 states are observable. This motivates adding more sensors in Part 2 to achieve full observability.
