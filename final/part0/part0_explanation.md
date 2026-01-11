# Part 0: Baseline Verification - Detailed Explanation

## Overview

Part 0 establishes the foundation for the entire project by verifying the system model and discretization. This part performs a baseline open-loop simulation to confirm that the continuous-time model is correctly discretized and behaves as expected.

## What is Baseline Verification?

Baseline verification is the first step in any control system design project. Before designing controllers or observers, we need to:
1. **Verify the model**: Ensure the mathematical model (A, B, C matrices) is correct
2. **Verify discretization**: Confirm that the continuous-time to discrete-time conversion works properly
3. **Understand open-loop behavior**: See how the system behaves without any control input

This provides a reference point for all subsequent parts.

## Code Structure

### Main Components

1. **`build_continuous_model()`** (from `final/utils/model.py`)
   - Constructs the continuous-time state-space matrices (A, B, C)
   - A: (12×12) state matrix capturing spring-mass dynamics
   - B: (12×3) input matrix for 3 control inputs
   - C: (1×12) output matrix (measures only x1, displacement of mass 1)

2. **`discretize_zoh()`** (from `final/utils/model.py`)
   - Converts continuous-time model to discrete-time using Zero-Order Hold (ZOH)
   - Sampling time: Ts = 0.01 seconds
   - Produces discrete-time matrices: Ad, Bd, Cd, Dd

3. **`simulate_discrete()`** (from `final/utils/simulation.py`)
   - Simulates the discrete-time system for N = 1000 steps (10 seconds)
   - Zero input: u = 0 (open-loop, no control)

## Step-by-Step Execution

### Step 1: Model Construction

```python
A, B, C = build_continuous_model()
```

- **A matrix (12×12)**: Describes how the 12 states (6 positions + 6 velocities) evolve over time
- **B matrix (12×3)**: Maps 3 control inputs (u1, u2, u3) to state changes
- **C matrix (1×12)**: Extracts a single output (y = x1) from the state vector

### Step 2: Discretization

```python
Ad, Bd, Cd, Dd = discretize_zoh(A, B, C, Ts=0.01)
```

**Why discretize?**
- Controllers are implemented digitally (on computers/microcontrollers)
- Digital systems operate at discrete time steps
- We need a discrete-time model to design discrete-time controllers

**Zero-Order Hold (ZOH) method:**
- Assumes control inputs are held constant between sampling instants
- This is the most common discretization method for digital control
- Formula: `Ad = exp(A * Ts)`, `Bd = ∫[0 to Ts] exp(A * τ) dτ * B`

**Sampling time Ts = 0.01 seconds:**
- 100 samples per second (100 Hz sampling rate)
- Fast enough to capture system dynamics (spring-mass frequencies are ~0.1-0.3 Hz)
- Standard choice for control systems

### Step 3: Initial Condition

```python
x0 = [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
```

This means:
- **Positions**: x1=0, x2=0, x3=0, x4=0, x5=0, **x6=1** (mass 6 displaced by 1 unit)
- **Velocities**: All velocities are zero (v1-v6 = 0)

The system starts with mass 6 displaced, and all other masses at rest.

### Step 4: Open-Loop Simulation

```python
u = np.zeros((3, N))  # Zero input
x, y, t = simulate_discrete(Ad, Bd, Cd, x0, u, N, Ts=Ts)
```

**What happens:**
- System evolves according to: `x[k+1] = Ad @ x[k] + Bd @ u[k]`
- With zero input: `x[k+1] = Ad @ x[k]` (free response)
- The initial displacement of mass 6 propagates through the spring chain
- All masses oscillate due to spring coupling

### Step 5: Visualization

Two plots are generated:

1. **`output_plot.png`**: System output y = Cx = x1
   - Shows the displacement of mass 1 over time
   - Demonstrates oscillatory behavior (undamped system)

2. **`displacements_plot.png`**: All 6 mass displacements (x1 through x6)
   - Shows how the initial condition (x6=1) propagates through the chain
   - All masses oscillate with different phases and amplitudes
   - Demonstrates the coupled nature of the spring-mass system

## Understanding the Results

### System Behavior

The system shows **oscillatory behavior** because:
1. **No damping**: The system is undamped (no friction or energy dissipation)
2. **Spring coupling**: Masses are connected by springs, creating oscillatory motion
3. **Initial energy**: The initial displacement (x6=1) provides potential energy
4. **Energy conservation**: In an undamped system, energy oscillates between potential and kinetic

### Why This Matters

1. **Verification**: Confirms the model and discretization are correct
2. **Baseline**: Provides a reference for comparing controlled behavior
3. **Understanding**: Shows the natural (open-loop) dynamics of the system
4. **Foundation**: All subsequent parts build upon this verified model

### Key Observations

- **Oscillatory response**: The system oscillates indefinitely (no damping)
- **Coupled motion**: All masses move together, showing the spring coupling
- **Energy propagation**: Initial displacement at mass 6 affects all masses
- **Marginal stability**: System is marginally stable (eigenvalues on unit circle)

## What Happens Next?

Part 0 establishes the baseline. In subsequent parts:
- **Part 1**: Analyzes which states can be observed (observability)
- **Part 2**: Designs an observer to estimate all states from partial measurements
- **Part 3+**: Designs controllers to regulate the system and reduce oscillations

## Summary

Part 0 successfully:
1. ✓ Built the continuous-time model (A, B, C matrices)
2. ✓ Discretized using ZOH at Ts = 0.01s
3. ✓ Simulated open-loop system with zero input
4. ✓ Generated plots showing oscillatory behavior
5. ✓ Verified the system model is correct

This baseline verification confirms that the mathematical model and discretization are working correctly, providing a solid foundation for all subsequent control design work.
