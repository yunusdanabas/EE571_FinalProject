# EE571 Bonus Question: Vehicle Trajectory Tracking Project

## What This Project Is About

This project is about making a car follow a curved path automatically. Think of it like cruise control, but instead of just maintaining speed, the controller also steers the car to follow a specific route. The challenge is that the car's dynamics are nonlinear (real-world physics), but we design the controller using a simplified linear model.

The goal was to compare two different control design methods:
1. **LQR (Linear Quadratic Regulator)** - optimizes a cost function
2. **Pole Placement** - directly assigns where the system's poles should be

Both controllers were tested with three different initial error conditions (1x, 2x, 3x the default offset) to see how robust they are.

---

## What Was Provided To You

You were given a MATLAB file `vehicle_dlqr.m` that had most of the infrastructure already built:

### The Vehicle Model
The template included a **bicycle model** - a simplified representation of a car that captures the essential dynamics:
- **State variables**: Position (X, Y), heading angle (ψ), longitudinal velocity (vx), lateral velocity (vy), and yaw rate (r)
- **Inputs**: Steering angle (δ) and longitudinal acceleration (ax)
- **Physics**: Tire forces based on slip angles, with linear tire model (force proportional to slip angle)

The vehicle parameters were already set:
- Mass: 1500 kg
- Yaw inertia: 2500 kg·m²
- Front/rear axle distances: 1.2 m / 1.6 m
- Cornering stiffnesses: 80,000 N/rad each

### The Reference Trajectory
The code generated a reference path that the car should follow:
- Curvature: `κ(t) = 0.01·sin(0.35t) + 0.005·sin(0.10t)` - a wavy path
- Speed: `v(t) = 15 + 1.0·sin(0.15t)` m/s - oscillating around 15 m/s
- The reference position (Xref, Yref) and heading (ψref) were computed by integrating these

### Helper Functions
- `bicycle_dynamics()` - computes the nonlinear vehicle dynamics
- `integrate_reference()` - generates the reference trajectory
- `lateral_heading_error()` - computes cross-track error (ey) and heading error (epsi)
- `rk4_step()` - Runge-Kutta 4th order integration
- `c2d_zoh_exact()` - discretizes continuous-time models using zero-order hold

### What Was Missing
The template had placeholder comments where you needed to fill in:
1. **Discretization** - convert the continuous error model to discrete-time
2. **Controller gains** - compute K_LQR and K_PP
3. **Control law** - calculate the regulation inputs in the simulation loop
4. **Experiment framework** - run all 6 cases (2 controllers × 3 scales) and generate comparison plots

---

## The Control Theory Behind It

### The Error Model

Instead of controlling the full vehicle state directly, we work with **error states** - how far off we are from the reference:

```
x_e = [vy, r, ey, epsi, ev]^T
```

Where:
- `vy` = lateral velocity (sideways motion)
- `r` = yaw rate (rotation speed)
- `ey` = cross-track error (how far left/right from the path)
- `epsi` = heading error (angle difference from reference heading)
- `ev` = speed error (how much faster/slower than reference)

The continuous-time error model is:
```
ẋ_e = A_c·x_e + B_c·u_reg
```

This model is **linearized** around a nominal operating point (vx ≈ 15 m/s, vy ≈ 0, etc.). The matrices A_c and B_c were already built in the template.

### Discretization

Since the controller runs at discrete time steps (every Ts = 0.02 seconds), we need to convert the continuous model to discrete:

```
x_e[k+1] = A_d·x_e[k] + B_d·u_reg[k]
```

The template provided `c2d_zoh_exact()` which uses the matrix exponential:
```
exp([A_c  B_c; 0  0] · Ts) = [A_d  B_d; 0  I]
```

This assumes the control input is held constant between samples (zero-order hold).

### Controller Design

#### LQR Controller

LQR minimizes a quadratic cost function:
```
J = Σ (x_e^T·Q·x_e + u_reg^T·R·u_reg)
```

You chose:
- Q = diag([5, 5, 50, 50, 30]) - weights on error states (higher weight on ey and epsi)
- R = diag([2, 1]) - weights on control effort (steering costs more than acceleration)

The solution gives a feedback gain K_LQR such that:
```
u_reg = -K_LQR · x_e
```

In your code, you solved the discrete algebraic Riccati equation using `scipy.linalg.solve_discrete_are()`.

#### Pole Placement Controller

Pole placement directly assigns where the closed-loop poles should be. The closed-loop system is:
```
x_e[k+1] = (A_d - B_d·K_PP)·x_e[k]
```

You chose real poles: `[0.85, 0.80, 0.75, 0.70, 0.65]` - all inside the unit circle (stable for discrete-time).

The gain K_PP was computed using `scipy.signal.place_poles()`.

### Feedforward + Feedback Structure

The total control input combines:
1. **Feedforward** - handles the reference trajectory (steering for curvature, acceleration for speed changes)
2. **Feedback** - corrects errors (the regulator part)

```
steering = steering_ff + steering_reg
throttle = throttle_ff + throttle_reg
```

Where:
- `steering_ff = (lf + lr)·κ_ref` - geometric steering for the curvature
- `throttle_ff = a_ref` - acceleration to match reference speed
- `steering_reg = -K[0,:]·x_e` - error correction
- `throttle_reg = -K[1,:]·x_e` - error correction

---

## Your Python Implementation

You translated the MATLAB code to Python and added the missing pieces. Here's what each file does:

### `vehicle_tracking.py` - The Core

This is the main simulation file. Key additions you made:

**Discretization (lines 114-128):**
```python
def c2d_zoh_exact(Ac, Bc, Ts):
    # Exact ZOH discretization using matrix exponential
    M = np.block([[Ac, Bc], [np.zeros((m, n)), np.zeros((m, m))]])
    Md = expm(M * Ts)
    Ad = Md[:n, :n]
    Bd = Md[:n, n:n+m]
    return Ad, Bd
```

**LQR Design (lines 133-141):**
```python
Q = np.diag([5.0, 5.0, 50.0, 50.0, 30.0])
R = np.diag([2.0, 1.0])
P = solve_discrete_are(Ad, Bd, Q, R)
K_LQR = np.linalg.inv(R + Bd.T @ P @ Bd) @ (Bd.T @ P @ Ad)
```

**Pole Placement Design (lines 143-146):**
```python
desired_poles = np.array([0.85, 0.80, 0.75, 0.70, 0.65])
result = place_poles(Ad, Bd, desired_poles)
K_PP = result.gain_matrix
```

**Simulation Loop (lines 246-298):**
The loop computes errors, builds the error state vector, calculates regulation inputs, combines with feedforward, applies saturations, and propagates the nonlinear plant using RK4.

### `main.py` - Experiment Runner

This orchestrates all 6 experiments:
- Loops through both controllers (LQR, PP)
- Loops through all three scales (1x, 2x, 3x)
- Generates comparison plots for each scale
- Computes and saves performance metrics

### `utils/plotting.py` - Visualization

Three comparison plot functions:
- `plot_trajectory_comparison()` - X-Y path overlay
- `plot_errors_comparison()` - ey, epsi, ev over time
- `plot_inputs_comparison()` - steering and acceleration commands

### `utils/metrics.py` - Performance Analysis

Computes:
- RMS errors (root-mean-square of ey, epsi, ev)
- Maximum absolute errors
- Control effort (RMS steering/acceleration)
- Saturation counts (how often actuators hit limits)

---

## Your Results

### LQR Controller: Success Story

The LQR controller works **really well**. Looking at your metrics:

**Scale 1x (default initial errors):**
- RMS cross-track error: 0.20 m
- RMS heading error: 3.18°
- RMS speed error: 0.72 m/s
- Steering saturation: only 10 out of 1251 samples (0.8%)
- Acceleration saturation: 73 out of 1251 samples (5.8%)

The trajectory plot shows the LQR path almost perfectly overlays the reference. The errors quickly converge to near zero and stay there.

**Scale 2x and 3x:**
Even with larger initial errors, LQR handles it gracefully:
- Scale 2x: RMS ey = 0.75 m (still very good)
- Scale 3x: RMS ey = 2.37 m (acceptable, though errors are larger)

The controller uses more control effort as errors increase, but it never loses stability.

### Pole Placement Controller: What Went Wrong

The pole placement controller **completely fails**. Your metrics show:

**All scales (1x, 2x, 3x):**
- RMS cross-track error: 80-112 m (hundreds of times worse than LQR!)
- RMS heading error: 93-115° (vehicle spinning out of control)
- **100% saturation** - both steering and acceleration are maxed out the entire time
- The trajectory plot shows the vehicle spiraling away from the reference immediately

### Why Did Pole Placement Fail?

There are a few likely reasons:

1. **Pole selection**: The poles `[0.85, 0.80, 0.75, 0.70, 0.65]` are all real and relatively close together. This might not provide enough damping or the right dynamics for this particular system. The closed-loop response could be too slow or have poor disturbance rejection.

2. **Control effort**: The pole placement gain K_PP likely requires very large control inputs to achieve those pole locations. When combined with the initial errors, the actuators immediately saturate, and once saturated, the controller can't do its job.

3. **Robustness**: LQR naturally provides some robustness margins (gain/phase margins), while pole placement doesn't guarantee these. The pole placement design might be fragile to the nonlinearities in the actual plant.

4. **Controllability**: While the system appears controllable (since `place_poles()` succeeded), the required gains might be very high, leading to sensitivity issues.

The fact that it fails immediately and saturates 100% suggests the gain matrix K_PP has very large entries, causing the control inputs to hit the limits right away.

---

## Key Takeaways

1. **LQR is robust**: Even though it's designed on a linearized model, it works well on the nonlinear plant. The quadratic cost naturally balances tracking performance with control effort.

2. **Pole placement is tricky**: Just placing poles inside the unit circle doesn't guarantee good performance. You need to consider:
   - The required control effort
   - Robustness margins
   - The actual closed-loop dynamics (not just stability)

3. **Feedforward helps**: The combination of feedforward (handles reference) + feedback (corrects errors) is a powerful structure. The feedforward does most of the work, and the regulator just fixes small deviations.

4. **Saturation matters**: Real actuators have limits. When controllers saturate, they lose effectiveness. LQR's cost function naturally penalizes large inputs, helping avoid saturation.

5. **Initial conditions matter**: Testing with different error scales (1x, 2x, 3x) shows how controllers handle larger disturbances. LQR degrades gracefully; pole placement fails catastrophically.

---

## What You Learned

This project demonstrates several important control concepts:

- **Linearization**: Designing controllers on simplified models that work on complex nonlinear systems
- **Discretization**: Converting continuous-time designs to work with digital controllers
- **Error-based control**: Regulating deviations from a reference rather than absolute states
- **Feedforward + Feedback**: Combining open-loop and closed-loop control
- **Controller comparison**: LQR vs pole placement - different design philosophies with very different results
- **Robustness**: A controller that works in simulation might fail in practice if it's not robust to uncertainties and nonlinearities

The dramatic difference between LQR and pole placement shows why LQR is widely used in practice - it's not just about stability, it's about performance, robustness, and handling constraints.
