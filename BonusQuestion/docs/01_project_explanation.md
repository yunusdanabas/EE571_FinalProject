# EE571 Final Exam Bonus: Vehicle Reference Tracking Project

## Purpose and Goal

Design and compare two state-feedback regulators for vehicle trajectory tracking:
- Use a **linearized error-state model** for controller design
- Simulate the **nonlinear bicycle-model plant** for closed-loop evaluation
- Track a **time-parameterized reference path** using feedback control

The objective is to track a reference trajectory using controllers designed on a linearized tracking-error model, while the closed-loop system is evaluated on the nonlinear bicycle-model plant.

---

## What is Provided

### Files
- **`vehicle_dlqr.m`**: Reference MATLAB code (for understanding structure)
- **`code/vehicle_tracking.py`**: Python implementation (main working file)
- **`doc/VehcileTracking_Document.md`**: Complete project specification

### Code Components to Implement (Python)
- Vehicle parameters and reference generation
- Continuous-time linear error model matrices `Ac` (5x5) and `Bc` (5x2)
- ZOH discretization using `scipy.linalg.expm`
- Nonlinear plant simulation with RK4 integration
- Tracking error computation
- Feedforward control terms
- Input saturation limits
- Plotting with matplotlib

### Key Python Libraries
- `numpy`: Matrix operations, linear algebra
- `scipy`: `scipy.linalg.expm` for discretization, `scipy.linalg.solve_discrete_are` for LQR
- `scipy.signal`: `place_poles` for pole placement
- `matplotlib`: Plotting

---

## What Must Be Implemented

### 1. Discretization
- Discretize the continuous error model `(Ac, Bc)` using zero-order hold (ZOH) at sampling time `Ts = 0.02 s`
- Use exact ZOH via augmented matrix exponential: `scipy.linalg.expm`
- Result: Discrete-time model `x_{e,k+1} = Ad @ x_{e,k} + Bd @ u_{reg,k}`

### 2. Regulator 1: Discrete-Time Infinite-Horizon LQR
- Choose weighting matrices `Q` (5x5, positive semi-definite) and `R` (2x2, positive definite)
- Solve discrete algebraic Riccati equation using `scipy.linalg.solve_discrete_are`
- Compute feedback gain: `K_LQR = inv(R + Bd.T @ P @ Bd) @ Bd.T @ P @ Ad`
- Regulation law: `u_reg = -K_LQR @ x_e`
- Combine with feedforward: `u = u_ff + u_reg`

### 3. Regulator 2: Discrete-Time Pole Placement
- Select **real poles only** (no complex pole locations)
- Poles must be inside the unit circle for stability (discrete-time)
- Use `scipy.signal.place_poles(Ad, Bd, desired_poles)`
- Regulation law: `u_reg = -K_PP @ x_e`
- Combine with feedforward: `u = u_ff + u_reg`

### 4. Error State Vector Construction
In the simulation loop, build the 5-state error vector:
```python
x_e = np.array([vy, r, ey, epsi, ev])
```
where:
- `vy`: lateral velocity (from plant state)
- `r`: yaw rate (from plant state)
- `ey`: cross-track error
- `epsi`: heading error (wrapped to [-pi, pi])
- `ev`: speed error `vx - v_ref`

---

## Required Experiment Matrix

Run **6 total cases**:
- **2 regulators**: LQR and Pole Placement
- **3 initial error scales**: baseline (1x), 2x, and 3x

| Regulator | Scale | Description |
|-----------|-------|-------------|
| LQR | 1x | Default initial errors |
| LQR | 2x | Default initial errors x 2 |
| LQR | 3x | Default initial errors x 3 |
| Pole Placement | 1x | Default initial errors |
| Pole Placement | 2x | Default initial errors x 2 |
| Pole Placement | 3x | Default initial errors x 3 |

### Initial Condition Scaling
The baseline initial condition offsets from reference:
- Position: `X(0) = X_ref(0) - 2.0 m`, `Y(0) = Y_ref(0) + 1.0 m`
- Heading: `psi(0) = psi_ref(0) + 8 deg`
- Speed: `v_x(0) = Vx0 - 5 m/s`
- Lateral velocity and yaw rate: `v_y(0) = 0`, `r(0) = 0`

For scale `s in {1, 2, 3}`, scale only the **offsets**:
- `X(0) = X_ref(0) - 2.0*s`
- `Y(0) = Y_ref(0) + 1.0*s`
- `psi(0) = psi_ref(0) + np.deg2rad(8*s)`
- `v_x(0) = Vx0 - 5*s`
- `v_y(0) = 0`, `r(0) = 0`

---

## Required Deliverables

### 1. PDF Report
Must include:
- Controller derivations (LQR and pole placement methods)
- Performance comparisons with plots
- Discussion of regulator robustness as initial error increases

### 2. Runnable Code Folder
- Python code that runs without errors
- Reproduces all required plots
- Clear instructions for execution (requirements.txt)

### 3. Comparison Plots
- **Trajectory plots**: Reference vs. vehicle path for both regulators
- **Error plots**: Overlay `e_y`, `e_psi`, `e_v` for both regulators
- **Input plots**: Steering angle and acceleration with saturation limits visible
- Plots should show performance of both regulators in the same figure for comparison

---

## Constraints and Parameters

### Simulation Parameters
- Sampling time: `Ts = 0.02 s` (50 Hz)
- Simulation duration: `Tend = 25 s`
- Nominal linearization speed: `Vx0 = 15 m/s`
- Internal integration step: `dt_int = Ts/10 = 0.002 s` (for RK4)

### Vehicle Parameters
- Mass: `m = 1500 kg`
- Yaw inertia: `Iz = 2500 kg*m^2`
- Front axle distance: `lf = 1.2 m`
- Rear axle distance: `lr = 1.6 m`
- Front cornering stiffness: `Cf = 80000 N/rad`
- Rear cornering stiffness: `Cr = 80000 N/rad`

### Input Saturation Limits
- Steering angle: `delta in [-25 deg, +25 deg]`
- Longitudinal acceleration: `ax in [-6, +3] m/s^2`

### Control Structure
The control input is:
```
u = u_ff + u_reg
```

Where:
- **Feedforward** (provided):
  - `delta_ff = (lf + lr) * kappa_ref` (geometric steering)
  - `ax_ff = a_ref` (reference acceleration)
- **Regulation** (to be implemented):
  - `u_reg = -K @ x_e` (where K is either K_LQR or K_PP)

---

## Python Implementation Notes

### Key Functions to Implement

```python
def c2d_zoh_exact(Ac, Bc, Ts):
    """Exact ZOH discretization using matrix exponential."""
    n = Ac.shape[0]
    m = Bc.shape[1]
    M = np.block([[Ac, Bc], [np.zeros((m, n)), np.zeros((m, m))]])
    Md = scipy.linalg.expm(M * Ts)
    Ad = Md[:n, :n]
    Bd = Md[:n, n:n+m]
    return Ad, Bd

def dlqr(Ad, Bd, Q, R):
    """Discrete LQR using scipy."""
    P = scipy.linalg.solve_discrete_are(Ad, Bd, Q, R)
    K = np.linalg.inv(R + Bd.T @ P @ Bd) @ (Bd.T @ P @ Ad)
    return K

def wrap_to_pi(angle):
    """Wrap angle to [-pi, pi]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi
```

### LQR Design
```python
from scipy.linalg import solve_discrete_are

P = solve_discrete_are(Ad, Bd, Q, R)
K_LQR = np.linalg.inv(R + Bd.T @ P @ Bd) @ (Bd.T @ P @ Ad)
```

### Pole Placement
```python
from scipy.signal import place_poles

result = place_poles(Ad, Bd, desired_poles)
K_PP = result.gain_matrix
```

---

## Error State Model Summary

The 5-state error vector for control design:
```
x_e = [v_y, r, e_y, e_psi, e_v]
```

The continuous-time model is:
```
x_e_dot = Ac @ x_e + Bc @ u_reg
```

After ZOH discretization:
```
x_e[k+1] = Ad @ x_e[k] + Bd @ u_reg[k]
```

---

## Reference Trajectory

The reference is time-parameterized:
- Curvature: `kappa_ref(t) = 0.01*sin(0.35*t) + 0.005*sin(0.10*t) [1/m]`
- Speed: `v_ref(t) = Vx0 + 1.0*sin(0.15*t) [m/s]`
- Acceleration: `a_ref(t) = (v_ref(t+Ts) - v_ref(t))/Ts`
- Yaw rate: `psi_dot_ref = v_ref * kappa_ref`
- Position: Integrated from velocity components

---

## Next Steps

After baseline setup, the project proceeds through Parts 0-6:
1. **Part 0**: Baseline setup and requirements freeze (completed)
2. **Part 1**: Model and signals review (completed)
3. **Part 2**: Discretization of error model (completed)
4. **Part 3**: LQR regulator implementation
5. **Part 4**: Pole placement regulator implementation
6. **Part 5**: Experiments and comparisons (6 runs)
7. **Part 6**: Report packaging and submission readiness

See `docs/00_anchor.md` for the detailed step-by-step plan.
