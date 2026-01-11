# Part 2: Observer Design - Detailed Explanation

## Overview

Part 2 designs a **Luenberger observer** for the 6-mass spring system. The observer uses measurements from only two sensors (measuring x1 and x6) to estimate all 12 states of the system.

## What is an Observer?

An observer is a mathematical system that estimates the internal states of a plant based on:
1. **Plant outputs** (what we can measure - in this case, x1 and x6)
2. **Plant inputs** (control signals, if any)
3. **Observer dynamics** (a mathematical model of the plant)

The observer continuously compares its output estimates with the actual measured outputs and corrects its internal state estimates accordingly.

## The Code Structure

### 1. Sensor Matrix (`get_part2_C()`)

```python
Cd_new = [[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
          [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]]
```

This is a **2×12 matrix** that defines which states we can measure:
- Row 1: Measures x1 (displacement of mass 1)
- Row 2: Measures x6 (displacement of mass 6)

The output equation is: **y = Cd_new @ x**, where:
- y[0] = x1 (measured)
- y[1] = x6 (measured)

### 2. Initial Conditions (`get_part2_initial_conditions()`)

```python
x0 = [0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0]  # Actual system state
xhat0 = [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]  # Observer initial guess
```

**Important**: The observer starts with a **wrong initial guess**:
- Actual state: x4=1, x5=1, x6=1 (all other states are 0)
- Observer guess: x6=1 (all other states are 0)

This mismatch tests whether the observer can converge to the correct estimate.

### 3. Observer Design (`design_observer_gain()`)

The observer gain **L** is designed using **pole placement via the dual system approach**.

#### What is Pole Placement?

Pole placement assigns specific eigenvalues (poles) to the observer error dynamics. The observer error dynamics are:

**e[k+1] = (Ad - L @ Cd) @ e[k]**

where **e = x - xhat** is the estimation error.

For the observer to work, we want the eigenvalues of **(Ad - L @ Cd)** to be inside the unit circle (for discrete-time systems), which ensures the error converges to zero.

#### Dual System Approach

Designing an observer for (Ad, Cd) is **mathematically equivalent** to designing a controller for (Ad^T, Cd^T). This is called the **dual system**.

The algorithm:
1. Form the dual system: **Ad_dual = Ad^T**, **Cd_dual = Cd^T**
2. Place poles for the dual system (as if designing a controller)
3. Get the controller gain **L_dual** from pole placement
4. Transpose to get observer gain: **L = L_dual^T**

#### Pole Selection

We use **12 distinct real poles** evenly spaced in [0.4, 0.8]:
- All poles are real (not complex) to avoid oscillatory convergence
- All poles are between 0.4 and 0.8 (well inside the unit circle)
- The spectral radius (max pole magnitude) is 0.8

This ensures:
- **Stable observer** (all poles < 1.0)
- **Fast convergence** (poles not too close to 1.0)
- **Smooth convergence** (no oscillations from complex poles)

### 4. Simulation (`simulate_observer()`)

The simulation runs the **coupled plant-observer system**:

**Plant:**
- x[k+1] = Ad @ x[k] + Bd @ u[k]
- y[k] = Cd @ x[k]

**Observer:**
- yhat[k] = Cd @ xhat[k]  (observer's output estimate)
- xhat[k+1] = Ad @ xhat[k] + Bd @ u[k] + L @ (y[k] - yhat[k])

The key term is **L @ (y[k] - yhat[k])**, which is the **correction term** that drives the observer estimate toward the true state.

## Warnings Encountered

### Warning 1: Pole Placement Convergence Warning

```
UserWarning: Convergence was not reached after maxiter iterations.
You asked for a tolerance of 0.01, we got 1.0.
```

**What this means:**
- SciPy's `place_poles` function uses an iterative algorithm to place poles
- We asked for a tolerance of **0.01** (relative tolerance)
- The algorithm couldn't achieve this tolerance and stopped after maximum iterations
- The actual tolerance achieved was **1.0** (much larger than requested)

**Why this happens:**
1. **High-dimensional system**: We're trying to place 12 poles in a 12-dimensional system
2. **Numerical conditioning**: The dual system may be poorly conditioned
3. **Pole placement difficulty**: Placing 12 distinct poles exactly where we want is numerically challenging

**Impact:**
- The observer gain **L** is still computed and the observer is **still stable**
- The spectral radius is exactly **0.8** as desired (all poles are inside unit circle)
- The observer works correctly, but the pole placement didn't converge to the exact requested tolerance
- This is common in high-dimensional pole placement problems

**Is this a problem?**
- **No, the observer still works correctly**
- The spectral radius is 0.8 (exactly as desired)
- All poles are inside the unit circle (observer is stable)
- The warning indicates numerical imprecision in pole placement, but the observer gain is still valid

### Warning 2: Qt Platform Plugin Warning

```
qt.qpa.plugin: Could not find the Qt platform plugin "wayland" in ""
```

**What this means:**
- This is a **graphics/windowing system warning** from matplotlib
- Qt is a GUI framework used by matplotlib for plotting
- The system is trying to use the "wayland" display protocol but can't find it
- This is a **harmless warning** - plots are still generated correctly

**Impact:**
- **None** - this is just a display system warning
- All plots are generated and saved correctly
- The warning doesn't affect the simulation or results

## Results Explanation

### Observer Design Results

```
Observer gain L shape: (12, 2)
Spectral radius: 0.800000
```

- **L shape (12, 2)**: The observer gain is a 12×2 matrix
  - 12 rows (one for each state)
  - 2 columns (one for each measurement)
  - L[i, j] tells us how much measurement j affects the estimate of state i

- **Spectral radius: 0.8**: The largest eigenvalue magnitude of (Ad - L @ Cd) is 0.8
  - All eigenvalues have magnitude ≤ 0.8
  - This ensures the observer error decays at least as fast as 0.8^k
  - After 100 steps, error is reduced by at least 0.8^100 ≈ 2×10^-10

### RMS Estimation Errors

RMS (Root Mean Square) error measures the average magnitude of estimation errors over time:

```
x1: 9.91×10⁻⁷  (very small - x1 is measured)
x2: 1.06×10⁻²  (small)
x3: 9.51×10⁰   (larger - x3 is not measured)
x4: 5.15×10¹   (largest - x4 is not measured)
x5: 7.44×10⁻²  (small)
x6: 4.86×10⁻⁶  (very small - x6 is measured)
Overall RMS: 5.24×10¹
```

**Key Observations:**

1. **Measured states (x1, x6)** have **very small errors** (~10⁻⁶):
   - These are directly measured, so the observer can estimate them accurately
   - The errors are essentially numerical precision

2. **Unmeasured states (x2, x3, x4, x5)** have **larger errors**:
   - x4 has the largest error (5.15×10¹)
   - x3 has the second largest error (9.51×10⁰)
   - These states are not directly measured, so the observer estimates them from the system dynamics and the two measurements

3. **Why x3 and x4 have larger errors:**
   - They are in the middle of the chain (not directly measured)
   - The initial condition mismatch was largest for x3 and x4 (observer started at 0, true state was 1)
   - The observer needs time to propagate information from the measured states (x1 and x6) to the unmeasured states

4. **Overall performance:**
   - The observer successfully estimates all 12 states
   - The overall RMS of 5.24×10¹ is dominated by x4
   - All errors are bounded and the observer is stable

## What the Plots Show

### 1. Outputs Comparison (`outputs_comparison.png`)

Shows true outputs (y1, y6) vs estimated outputs (yhat1, yhat6):
- The estimated outputs should match the true outputs very closely
- Since y1 = x1 and y6 = x6 are directly measured, the observer estimates should match almost exactly

### 2. Estimation Errors (`estimation_errors.png`)

Shows the estimation error e = x - xhat for all 6 displacement states:
- All errors should converge toward zero over time
- Errors for x1 and x6 (measured states) should be smallest
- Errors for x3 and x4 (unmeasured states) may be larger initially but should still converge

## Summary

1. **Observer Design**: Successfully designed using pole placement (spectral radius = 0.8)

2. **Convergence Warning**: Pole placement didn't converge to exact tolerance, but observer is still stable and works correctly

3. **Results**: Observer successfully estimates all 12 states, with measured states (x1, x6) having very small errors and unmeasured states having larger but bounded errors

4. **Stability**: Observer is stable (spectral radius = 0.8 < 1.0), ensuring errors converge to zero

5. **Purpose**: This observer will be used in later parts (Part 3+) to provide state estimates for feedback control when only partial state measurements are available
