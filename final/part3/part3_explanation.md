# Part 3: LQR Controller Design with Observer - Detailed Explanation

## Overview

Part 3 implements an **LQR (Linear Quadratic Regulator) controller** that uses **estimated states** from the Part 2 observer to stabilize the 6-mass spring system and regulate the outputs y1 and y6.

## What is LQR Control?

**LQR (Linear Quadratic Regulator)** is an optimal control method that minimizes a quadratic cost function. The cost function balances:
- **Control effort**: Penalizing large inputs (u^T u)
- **Output regulation**: Penalizing deviations of outputs from zero (y1² + y6²)

The controller finds the optimal feedback gain K such that `u[k] = -K @ x[k]` minimizes the total cost over time.

## Code Structure

### Main Components

1. **`design_lqr()`**
   - Solves the Discrete-time Algebraic Riccati Equation (DARE)
   - Computes optimal LQR gain: K = (R + Bd^T P Bd)^(-1) (Bd^T P Ad)
   - Verifies closed-loop stability

2. **`compute_cost()`**
   - Computes the cost function: J = Σ(u[k]^T u[k] + y1[k]² + y6[k]²)
   - Evaluates controller performance

3. **`simulate_lqr_with_observer()`** (from `final/utils/simulation.py`)
   - Simulates the closed-loop system with observer-based control

## Step-by-Step Execution

### Step 1: Load Part 2 Dependencies

```python
Cy = get_part2_C()  # Measurement matrix (2×12, measures x1 and x6)
x0, xhat0 = get_part2_initial_conditions()
L, obs_info = design_observer_gain(Ad, Cy)  # Observer gain from Part 2
```

- **Cy**: Measurement matrix (2 sensors: x1 and x6)
- **L**: Observer gain from Part 2 (12×2 matrix)
- **x0, xhat0**: Initial conditions (true state and observer estimate)

### Step 2: Define Cost Matrices

```python
Q = Cy.T @ Cy  # State weight: penalizes x1 and x6
R = np.eye(3)  # Input weight: identity matrix (equal weighting)
```

**Cost function structure:**
- **Q = Cy^T @ Cy**: (12×12) matrix that extracts x1 and x6
  - Q[i,j] = 1 if i=j and (i=0 or i=5), else 0
  - This penalizes x1² and x6² in the cost
- **R = I₃**: (3×3) identity matrix
  - Penalizes u1², u2², u3² equally
  - Equal weighting on all three inputs

**Cost function:**
```
J = Σ_{k=0}^{N-1} (u[k]^T u[k] + y1[k]² + y6[k]²)
```

This balances:
- **Control effort**: u[k]^T u[k] (don't use too much control)
- **Output regulation**: y1[k]² + y6[k]² (keep outputs near zero)

### Step 3: Design LQR Controller

```python
K, P, rho = design_lqr(Ad, Bd, Q, R)
```

**DARE (Discrete-time Algebraic Riccati Equation):**
```
P = solve_discrete_are(Ad, Bd, Q, R)
```

This solves:
```
P = Q + Ad^T P Ad - Ad^T P Bd (R + Bd^T P Bd)^(-1) Bd^T P Ad
```

**LQR gain computation:**
```
K = (R + Bd^T P Bd)^(-1) (Bd^T P Ad)
```

**What does K do?**
- K is a (3×12) matrix
- Maps 12 states to 3 control inputs
- Optimal in the sense that it minimizes the cost function J

**Closed-loop stability:**
```
Acl = Ad - Bd @ K
spectral_radius = max(|eigenvalues(Acl)|)
```

- Spectral radius < 1.0 → system is stable
- Result: 0.999463 (stable, but close to unity → slow convergence)

### Step 4: Simulate Closed-Loop System

```python
results = simulate_lqr_with_observer(Ad, Bd, Cy, K, L, x0, xhat0, N, Ts)
```

**Key difference from full-state feedback:**
- **Full-state feedback**: u[k] = -K @ x[k] (uses true states)
- **Observer-based control**: u[k] = -K @ xhat[k] (uses estimated states)

**Closed-loop dynamics:**
1. **Plant**: x[k+1] = Ad @ x[k] + Bd @ u[k]
2. **Observer**: xhat[k+1] = Ad @ xhat[k] + Bd @ u[k] + L @ (y[k] - yhat[k])
3. **Controller**: u[k] = -K @ xhat[k]

**Why use estimated states?**
- In practice, we don't have access to all states
- We only measure y1 and y6 (2 outputs)
- The observer estimates all 12 states from these 2 measurements
- The controller uses these estimates for feedback

### Step 5: Compute Cost

```python
J = compute_cost(results['x'], results['u'], Cy, N)
```

The cost J = 9.057×10⁷ reflects:
- Control effort over 10 seconds
- Output regulation performance
- Trade-off between control and regulation

## Understanding the Results

### Separation Principle

The **separation principle** states that:
1. **Observer design** (Part 2): Design L to estimate states accurately
2. **Controller design** (Part 3): Design K to minimize cost
3. **Combined system**: Works optimally even though designed separately

**Why this works:**
- Observer error (x - xhat) converges to zero (Part 2)
- Once error is small, using xhat is almost as good as using x
- The controller and observer can be designed independently

### Closed-Loop Performance

**Spectral radius: 0.999463**
- Stable (< 1.0), but very close to unity
- Indicates slow convergence (poles near unit circle)
- System will converge, but slowly

**Total cost: J = 9.057×10⁷**
- Accumulated over 10 seconds (1000 steps)
- Balances control effort and output regulation
- Large value due to initial conditions and slow convergence

**Output regulation:**
- Both y1 and y6 are regulated toward zero
- Controller successfully reduces oscillations
- System converges over the simulation horizon

### Comparison: Full-State vs Observer-Based

| Feature | Full-State Feedback | Observer-Based (Part 3) |
|---------|---------------------|------------------------|
| **Control law** | u = -K @ x | u = -K @ xhat |
| **State access** | All states available | Only 2 outputs measured |
| **Performance** | Optimal (if states perfect) | Near-optimal (if observer good) |
| **Practicality** | Often impractical | Practical (few sensors) |

## Key Concepts

### 1. Optimal Control

LQR is **optimal** in the sense that:
- It minimizes the quadratic cost function J
- No other linear controller can achieve lower cost
- The solution is unique (if the system is controllable)

### 2. Cost Function Design

The cost function J = Σ(u^T u + y1² + y6²) balances:
- **u^T u**: Control effort (energy)
- **y1² + y6²**: Output regulation (keep outputs near zero)

**Tuning:**
- Increase Q → more emphasis on regulation (may need more control)
- Increase R → more emphasis on saving energy (may have worse regulation)
- Current design: Q = Cy^T @ Cy, R = I₃ (balanced)

### 3. Observer-Based Control

Using estimated states (xhat) instead of true states (x):
- **Advantage**: Only need 2 sensors (x1, x6) instead of 12
- **Disadvantage**: Observer error affects performance
- **Solution**: Design good observer (Part 2) → error is small

## Warnings Encountered

### Warning: Pole Placement Convergence

The observer design (Part 2) generates a warning about pole placement convergence. This doesn't affect Part 3 because:
- The observer gain L is already computed (from Part 2)
- The LQR controller design is independent
- Both observer and controller are stable

### Warning: Qt Platform Plugin

Harmless graphics warning from matplotlib. Doesn't affect simulation or results.

## What Happens Next?

Part 3 demonstrates LQR control with estimated states. In subsequent parts:
- **Part 4**: Reduces inputs (removes u3) to analyze actuator limitations
- **Part 5**: Designs Kalman filter for noisy systems
- **Part 6**: Combines LQR + Kalman filter (LQG controller)

## Summary

Part 3 successfully:
1. ✓ Designed LQR controller using DARE
2. ✓ Computed optimal gain K (3×12)
3. ✓ Verified closed-loop stability (spectral radius = 0.999463)
4. ✓ Simulated observer-based control
5. ✓ Regulated outputs y1 and y6 toward zero
6. ✓ Demonstrated separation principle

**Key achievement**: Successfully stabilized the 6-mass spring system using only 2 sensors (x1, x6) by combining observer (Part 2) and controller (Part 3) designs.
