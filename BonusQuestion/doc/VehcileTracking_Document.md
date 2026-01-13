# EE571 Final Exam Bonus Problem
## Steering a Vehicle Along a Reference Track. Discrete LQR and pole placement on a linearized error model

This document consolidates (1) the bonus-problem handout pages (screenshots), (2) the provided MATLAB starter code `vehicle_dlqr.m`, and (3) additional explanatory materials. It explains what is given, what you are expected to implement, and how the math maps to the code.

---

## 1. Goal and objective

### Goal
Design and compare **two state-feedback regulators** for a vehicle trajectory-tracking task, using a **linearized error-state model** while simulating the **nonlinear bicycle-model plant**.

### Objective
Track a **time-parameterized reference path** using a feedback controller designed on a **linearized tracking-error model**, while the closed-loop system is evaluated on the **nonlinear bicycle-model plant**.

### What you must implement
You must apply two regulators to the discrete-time error model:

1. **Regulator 1. Infinite-horizon discrete-time LQR (DLQR)**  
   Design a discrete state-feedback law on the error model and derive the final regulation input.

2. **Regulator 2. Discrete-time pole placement**  
   Design a discrete state-feedback gain by assigning **real** closed-loop poles (no complex pole locations).

### Comparison requirement
Run both regulators under three different initial-error magnitudes:

- default initial errors (as given in the MATLAB file),
- default initial errors scaled by **2**,
- default initial errors scaled by **3**.

So you will produce **6 runs total** (2 regulators × 3 initial-error scales).

### Deliverables
- A short **PDF report**: controller derivations plus performance comparisons with plots.
- A runnable **code folder**: your modified MATLAB code should run without issues and reproduce the plots.

---

## 2. Nonlinear plant model used for simulation (bicycle dynamics)

The nonlinear plant state is:

\[
x = \begin{bmatrix} X & Y & \psi & v_x & v_y & r \end{bmatrix}^T
\]

- \(X, Y\). global position  
- \(\psi\). yaw angle (heading)  
- \(v_x, v_y\). body-frame longitudinal and lateral velocities  
- \(r\). yaw rate  

The plant input is:

\[
u = \begin{bmatrix} \delta \\ a_x \end{bmatrix}
\]

- \(\delta\). steering angle input  
- \(a_x\). longitudinal acceleration command (named `throttle` in the MATLAB file)

### Kinematics
The handout and code use:

\[
\dot X = v_x\cos\psi - v_y\sin\psi,\qquad
\dot Y = v_x\sin\psi + v_y\cos\psi,\qquad
\dot\psi = r
\]

### Tire slip and lateral forces (small-angle approximation, \(v_x > 0\))
Slip angles:

\[
\alpha_f \approx \delta - \frac{v_y + l_f r}{v_x},\qquad
\alpha_r \approx -\frac{v_y - l_r r}{v_x}
\]

Linear tire model:

\[
F_{yf} = C_f \alpha_f,\qquad
F_{yr} = C_r \alpha_r
\]

### Dynamics
\[
\dot v_x = a_x + r v_y,\qquad
\dot v_y = \frac{F_{yf}+F_{yr}}{m} - r v_x,\qquad
\dot r = \frac{l_f F_{yf}-l_r F_{yr}}{I_z}
\]

All of the above is implemented in the MATLAB helper function:

- `bicycle_dynamics(x, u, params)`

This completes the nonlinear plant \(\dot x = f(x,u)\). In the code, the plant is integrated with RK4 using a smaller internal step `dt_int = Ts/10`, and the input is held constant over each sampling interval (ZOH).

### Parameters used in `vehicle_dlqr.m`
Near the top of the script:

- \(m=1500\,\mathrm{kg}\)
- \(I_z=2500\,\mathrm{kg\,m^2}\)
- \(l_f=1.2\,\mathrm{m}\), \(l_r=1.6\,\mathrm{m}\)
- \(C_f=80000\,\mathrm{N/rad}\), \(C_r=80000\,\mathrm{N/rad}\)

where \(C_f, C_r\) are cornering stiffnesses, and \(l_f, l_r\) are distances from CG to front and rear axles.

---

## 3. Reference trajectory and tracking-error definitions

### Reference signals
The reference is time-parameterized:

\[
\big(X_{ref}(t), Y_{ref}(t), \psi_{ref}(t), v_{ref}(t), \kappa_{ref}(t)\big)
\]

Reference yaw-rate relation:

\[
\dot\psi_{ref}(t) = v_{ref}(t)\,\kappa_{ref}(t)
\]

### How the MATLAB file generates the reference
In `vehicle_dlqr.m`:

- Sample time: `Ts = 0.02 s` (50 Hz)
- Duration: `Tend = 25 s`
- Nominal speed used for linearization: `Vx0 = 15 m/s`

Curvature and speed profiles:

\[
\kappa_{ref}(t)=0.01\sin(0.35t) + 0.005\sin(0.10t)\quad [1/\mathrm{m}]
\]
\[
v_{ref}(t)=V_{x0} + 1.0\sin(0.15t)\quad [\mathrm{m/s}]
\]

Reference acceleration is approximated by finite difference:

\[
a_{ref}(t) \approx \frac{v_{ref}(t+T_s)-v_{ref}(t)}{T_s}
\]

The function `integrate_reference(t, v_ref, kappa_ref)` computes:

\[
\dot\psi_{ref}=v_{ref}\kappa_{ref},\quad
\dot X_{ref}=v_{ref}\cos\psi_{ref},\quad
\dot Y_{ref}=v_{ref}\sin\psi_{ref}
\]

using simple forward-Euler integration.

### Tracking errors used for regulation
Define position error:

\[
e_p = \begin{bmatrix} X-X_{ref} \\ Y-Y_{ref} \end{bmatrix}
\]

Define the reference normal vector:

\[
\hat n = \begin{bmatrix} -\sin\psi_{ref} \\ \cos\psi_{ref} \end{bmatrix}
\]

Cross-track error:

\[
e_y = \hat n^T e_p
\]

Heading error (wrapped to \([-\pi, \pi]\)):

\[
e_\psi = \mathrm{wrapToPi}(\psi-\psi_{ref})
\]

Speed error (the code uses longitudinal speed):

\[
e_v = v_x - v_{ref}
\]

In code, `lateral_heading_error(...)` returns \(e_y\) and \(e_\psi\).

---

## 4. Linearized tracking-error model used for controller design

The handout emphasizes that the plant is nonlinear, so control is designed on a **linear error model** obtained by linearizing around a nominal operating point.

### Nominal operating point
The linear model assumes:

\[
v_x \approx V_{x0} > 0,\quad v_y \approx 0,\quad r \approx 0,\quad \delta \approx 0
\]

In the MATLAB file: `Vx0 = 15`.

### Lateral-yaw linear approximation
The standard linear model uses:

\[
\begin{bmatrix}\dot v_y \\ \dot r\end{bmatrix}
=
A_{lat}
\begin{bmatrix}v_y\\r\end{bmatrix}
+
B_\delta\,\delta
\]

with:

\[
A_{lat}=
\begin{bmatrix}
-\frac{C_f+C_r}{mV_{x0}} &
-\Big(V_{x0}+\frac{l_fC_f-l_rC_r}{mV_{x0}}\Big)\\
-\frac{l_fC_f-l_rC_r}{I_zV_{x0}} &
-\frac{l_f^2C_f+l_r^2C_r}{I_zV_{x0}}
\end{bmatrix},
\quad
B_\delta=
\begin{bmatrix}
\frac{C_f}{m}\\
\frac{l_fC_f}{I_z}
\end{bmatrix}
\]

This matches how the script defines `A11, A12, A21, A22` and the steering-input terms `Bvydelta, Brdelta`.

### Error-state vector for control design
The handout and code define a 5-state error vector:

\[
x_e = \begin{bmatrix} v_y \\ r \\ e_y \\ e_\psi \\ e_v \end{bmatrix}
\]

The regulation input is:

\[
u_{reg} = \begin{bmatrix} \delta_{reg} \\ a_{x,reg} \end{bmatrix}
\]

### Error propagation equations used in the script
The original handout indicates the error derivatives include reference terms:

\[
\dot e_y \approx v_y + V_{x0}e_\psi,\quad
\dot e_\psi \approx r - V_{x0}\kappa_{ref}(t),\quad
\dot e_v \approx a_x - a_{ref}(t).
\]

However, the MATLAB script handles the reference curvature and acceleration terms via **feedforward**, so the **regulation model** used to build `(Ac, Bc)` uses the simpler forms:

\[
\dot e_y \approx v_y + V_{x0}e_\psi
\]
\[
\dot e_\psi \approx r
\]
\[
\dot e_v \approx a_x
\]

This separation allows the feedforward to handle the reference tracking while the regulator focuses on error correction.

### Continuous-time state space form in `vehicle_dlqr.m`
The script constructs:

\[
\dot x_e = A_c x_e + B_c u_{reg}
\]

with \(A_c \in \mathbb{R}^{5\times 5}\), \(B_c \in \mathbb{R}^{5\times 2}\). The matrix structure in the code is:

- Rows 1 to 2: lateral-yaw dynamics for \([v_y, r]\) driven by \(\delta\)
- Row 3: \(\dot e_y = v_y + V_{x0} e_\psi\)
- Row 4: \(\dot e_\psi = r\)
- Row 5: \(\dot e_v = a_x\)

---

## 5. Control structure in the assignment. Feedforward plus feedback regulation

The required structure is:

\[
u = u_{ff}(t) + u_{reg}(x_e)
\]

### Feedforward terms provided in the MATLAB script
Inside the simulation loop, the script defines starter feedforward:

- Steering feedforward:
  \[
  \delta_{ff} \approx (l_f+l_r)\kappa_{ref}(t)
  \]
  This is the geometric relation \(\delta \approx L\kappa\) for small angles, where \(L = l_f + l_r\) is the wheelbase.

- Longitudinal acceleration feedforward:
  \[
  a_{x,ff} = a_{ref}(t) = \dot v_{ref}(t)
  \]

In the code these are:

- `steering_feed_forward`
- `throttle_feed_forward`

The feedforward terms make the vehicle follow the reference in an ideal, error-free case, while the regulator stabilizes and drives the error states toward zero.

### What you must compute
You must compute `steering_reg` and `ax_reg` using one of the two regulators, then combine:

- `steering_input = steering_feed_forward + steering_reg`
- `throttle = throttle_feed_forward + ax_reg`

The script already applies saturations after this combination:

- steering limited to ±25 degrees
- acceleration limited to [-6, 3] m/s²

---

## 6. Discretization and simulation timing

### Discretize the continuous error model (required)
You must discretize \((A_c, B_c)\) using **zero-order hold** at `Ts`:

\[
x_{e,k+1} = A_d x_{e,k} + B_d u_{reg,k}
\]

The script includes an exact ZOH helper:

- `c2d_zoh_exact(A, B, Ts)`

It uses the augmented matrix exponential identity:

\[
\exp\left(\begin{bmatrix} A & B \\ 0 & 0 \end{bmatrix} Ts\right)
=
\begin{bmatrix} A_d & B_d \\ 0 & I \end{bmatrix}
\]

You can use this function directly and avoid relying on MATLAB's `c2d` function.

### Simulation of the nonlinear plant
The plant is integrated with RK4 using an internal step:

- `dt_int = Ts/10`

Inputs are held constant within each `Ts` interval, which matches the ZOH assumption used for discretizing the controller model.

---

## 7. Regulator 1. Discrete-time infinite-horizon LQR (DLQR)

After discretization, DLQR solves the discrete algebraic Riccati equation for an infinite-horizon quadratic cost:

\[
J = \sum_{k=0}^{\infty} \left(x_{e,k}^T Q x_{e,k} + u_{reg,k}^T R u_{reg,k}\right)
\]

and yields:

\[
u_{reg,k} = -K_{LQR} x_{e,k}
\]

Implementation steps:

1. Compute `[Ad, Bd] = c2d_zoh_exact(Ac, Bc, Ts)`.
2. Choose \(Q \succeq 0\) (5×5) and \(R \succ 0\) (2×2), symmetric and positive semi-definite / definite respectively.
3. In MATLAB, compute `[K_LQR, ~, ~] = dlqr(Ad, Bd, Q, R)`.
4. In the simulation loop form:
   \[
   x_e = [v_y,\ r,\ e_y,\ e_\psi,\ e_v]^T
   \]
   then compute `u_reg = -K_LQR * x_e`.

---

## 8. Regulator 2. Discrete-time pole placement (real poles only)

Pole placement chooses a gain \(K_{PP}\) so that the closed-loop discrete matrix:

\[
A_{cl} = A_d - B_d K_{PP}
\]

has desired eigenvalues (poles).

Constraints from the problem statement:

- Choose **real** poles only (no complex pole locations).
- For discrete time, stable poles must lie inside the unit circle.

Practical notes:

- If \((A_d, B_d)\) is not fully controllable, you can only assign poles of the controllable modes. The uncontrollable modes remain fixed.
- Because the input \(u\) is 2D, you will select 5 poles total only if the discrete system is controllable. Otherwise, you select poles for the controllable subspace.
- In MATLAB you typically use: `K_PP = place(Ad, Bd, desired_poles)` (or `acker` for SISO systems).

---

## 9. Initial-condition scaling experiments (default, ×2, ×3)

The MATLAB file's default initial condition (referenced as "line 87" in the problem statement) offsets the plant from the reference:

\[
[X(0),Y(0),\psi(0),v_x(0)] = [X_{ref}(0)-2.0\,\mathrm{m},\;Y_{ref}(0)+1.0\,\mathrm{m},\;\psi_{ref}(0)+8^\circ,\;V_{x0}-5\,\mathrm{m/s}]
\]

and:

\[
v_y(0)=0,\quad r(0)=0
\]

To form the ×2 and ×3 cases, scale these offsets from the reference by factors 2 and 3, while keeping the reference itself unchanged.

Implementation pattern:

1. Build the baseline offset \(\Delta x_0\) relative to \((X_{ref}(0), Y_{ref}(0), \psi_{ref}(0), V_{x0})\).
2. For scale \(s \in \{1,2,3\}\), set \(x_0 = x_{ref,0} + s\Delta x_0\).
3. Run the same simulation for each `s` and for each regulator.

---

## 10. What `vehicle_dlqr.m` already provides, and what is missing

### Already implemented
- Vehicle parameters and reference generation.
- Construction of the continuous error model `(Ac, Bc)`.
- Nonlinear plant simulation with RK4 and ZOH input.
- Computation of tracking errors `ey`, `epsi`, `ev`.
- Plot templates for trajectory, errors, and inputs.

### Missing, and required for the assignment
- Discretization: `(Ad, Bd)`.
- Regulator gains: `K_LQR` and `K_PP`.
- Regulation input calculation inside the simulation loop:
  - compute `x_e`
  - compute `u_reg`
  - add feedforward
  - apply saturation
- A loop or wrapper to run:
  - both regulators
  - all three initial-error scales
  - then produce overlay plots for comparison.

---

## 11. Recommended plots and simple performance metrics

### Plots (minimum)
The script currently plots a single run. For the report, you typically want overlays:

- **Trajectory plot** showing:
  - reference trajectory,
  - LQR result,
  - pole-placement result,
  possibly for each initial-error scaling (x1, x2, x3).

- **Error plots** (\(e_y, e_\psi, e_v\)) comparing:
  - regulator type (LQR vs pole placement),
  - initial error level (x1 vs x2 vs x3).

- **Input plots** (\(\delta, a_x\)) to show effort and saturation behavior, with saturation limits visible.

This is consistent with the deliverable request to "show the performance of two different regulators in the same figure."

### Useful scalar metrics (optional but helpful in the report)
For each run:
- RMS cross-track error: \(\sqrt{\frac{1}{T}\int_0^T e_y(t)^2 dt}\) (discrete sum in code)
- max absolute errors for \(e_y\), \(e_\psi\), \(e_v\)
- percent of time saturated for \(\delta\) and \(a_x\)

These help explain why one regulator performs better or worse, especially for ×2 and ×3 initial errors.

---

## 12. Mapping between math and MATLAB variables

| Concept | Math | MATLAB variable |
|---|---:|---|
| Plant state | \([X,Y,\psi,v_x,v_y,r]\) | `x(k,:)` |
| Inputs | \([\delta,a_x]\) | `steering_input`, `throttle` |
| Reference curvature | \(\kappa_{ref}\) | `kappa_ref(k)` |
| Reference speed | \(v_{ref}\) | `v_ref(k)` |
| Reference accel | \(a_{ref}\) | `a_ref(k)` |
| Cross-track error | \(e_y\) | `ey` |
| Heading error | \(e_\psi\) | `epsi` |
| Speed error | \(e_v\) | `ev` |
| Error state | \([v_y,r,e_y,e_\psi,e_v]\) | you create `xe` |
| Continuous model | \(\dot x_e=A_c x_e + B_c u_{reg}\) | `Ac`, `Bc` |
| Discrete model | \(x_{e,k+1}=A_d x_{e,k}+B_d u_{reg,k}\) | `Ad`, `Bd` |
| LQR feedback | \(u_{reg}=-Kx_e\) | `K_LQR` |
| Pole placement feedback | \(u_{reg}=-Kx_e\) | `K_PP` |

---

## 13. Additional notes

### Integration method
The nonlinear plant is simulated with RK4 using an internal integration step `dt_int = Ts/10`. The controller updates every `Ts` and holds inputs constant within each sample interval, which matches the ZOH assumption used for discretizing the controller model.

### Saturations
The script applies saturations after combining feedforward and regulation inputs:
- Steering: \(\delta \in [-25^\circ, +25^\circ]\)
- Acceleration: \(a_x \in [-6, +3]\,\mathrm{m/s^2}\)

These limits are applied each control update.

### Reference integration
The function `integrate_reference(t, v_ref, kappa_ref)` computes the reference pose using simple forward-Euler integration:
- \(\psi_{ref}\) via \(\psi_{ref,k+1} = \psi_{ref,k} + Ts\,v_{ref,k}\kappa_{ref,k}\)
- \(X_{ref}\) and \(Y_{ref}\) via Euler integration using \(v_{ref}\cos\psi_{ref}\) and \(v_{ref}\sin\psi_{ref}\)

---

## Note on file availability
If you want this document to incorporate additional materials that are not currently accessible (for example, a PDF version of the handout or the MATLAB script `vehicle_dlqr.m`), reupload them and the details can be integrated.
