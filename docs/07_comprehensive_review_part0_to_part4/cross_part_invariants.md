# Cross-Part Invariants: Parts 0-7

**DO NOT CHANGE** these values when implementing Part 8. These are frozen parameters used consistently across Parts 0-7.

## Simulation Parameters

- **Sampling time:** `Ts = 0.01` seconds
- **Simulation horizon:** `N = 1000` steps (10.0 seconds total)
- **Time vector:** `t = np.arange(N+1) * Ts` (length N+1, matches state/output arrays: t[0] to t[N])

## Array Indexing Convention

- **State trajectory x:** `(n, N+1)` stores `x[0]` through `x[N]` (N+1 samples)
- **Input trajectory u:** `(m, N)` stores `u[0]` through `u[N-1]` (N samples)
- **Output trajectory y:** `(p, N+1)` stores `y[0]` through `y[N]` (N+1 samples)

**Cost indexing convention:**
- Cost accumulation: `J = sum from k=0 to N-1 of stage_cost[k]`
- Cost range: `k = 0 to 999` (inclusive, all N input samples)
- Standard convention: `u[k]` pairs with transition from `x[k]` to `x[k+1]`

## Measurement Matrix (Part 2 Configuration)

**C_part2 (2×12):** Measures x1 and x6
```
C_part2 = [[1 0 0 0 0 0 0 0 0 0 0 0]
           [0 0 0 0 0 1 0 0 0 0 0 0]]
```

**Source:** Final exam Question 2, verified in `docs/sources/final_exam_extract.md` Section 4

**Used in:** Parts 2, 3, 4 (and should be used in Parts 5-7)

## Initial Conditions

**Actual system initial state x0:**
```
x0 = [0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0]^T
```

**Observer initial state xhat0:**
```
xhat0 = [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]^T
```

**Source:** Final exam Question 2, verified in `docs/sources/final_exam_extract.md` Section 4

**Used in:** Parts 2, 3, 4 (and should be used in Parts 5-7)

## Cost Function Definition

**Part 3 and Part 4 cost:**
```
J = Σ_{k=0}^{N-1} (u[k]^T u[k] + y1[k]^2 + y6[k]^2)
```

Where:
- `u[k]` is the input vector at time k
- `y1[k]` is the displacement of mass 1 (state x1) at time k
- `y6[k]` is the displacement of mass 6 (state x6) at time k

**Cost output selector Cy:**
```
Cy = C_part2 = [[1 0 0 0 0 0 0 0 0 0 0 0]
                 [0 0 0 0 0 1 0 0 0 0 0 0]]
```

**Cost computation:**
- Uses plant output `y[k] = Cy @ x[k]` (NOT estimated output)
- Cost consistency: `stage_cost[k]` uses `u[k]` and `y[k]` from same time index k

**Source:** Final exam Question 3, verified in `docs/sources/final_exam_extract.md` Section 5

**Used in:** Parts 3, 4 (and should be used in Parts 5-7 if same cost function applies)

## Observer Design (Part 2)

**Observer gain L:** Shape (12, 2)

**Observer spectral radius:** 0.800000 (< 1.0, stable)

**Observer design method:** Pole placement (dual system approach)

**Observer poles:** 12 distinct real poles evenly spaced in [0.4, 0.8]

**Observer closed-loop matrix:** `Aobs = Ad - L @ Cmeas`

**Used in:** Parts 2, 3, 4 (and should be reused in Parts 5-7 for consistency)

## Part 3 Baseline Metrics

**Total cost J:** 3.915420e+07

**Max input magnitudes:**
- Max |u1|: 1.228057e+03
- Max |u2|: 2.700929e+01
- Max |u3|: 2.403429e+03
- Max |u| overall: 2.403429e+03

**LQR gain K:** Shape (3, 12), saved in `python/part3/outputs/K_matrix.npy`

**Observer gain L:** Shape (12, 2), saved in `python/part3/outputs/L_matrix.npy`

**Used for:** Part 4 comparison, Parts 5-7 baseline reference

## Part 4 Baseline Metrics

**Total cost J_red:** 5.838118e+07

**Max input magnitudes:**
- Max |u1|: 3.310572e+03
- Max |u2|: 4.921896e+01
- Max |u_red| overall: 3.310572e+03

**LQR gain K_red:** Shape (2, 12), saved in `python/part4/outputs/K_red_matrix.npy`

**Used for:** Parts 5-7 baseline reference (if reduced input configuration is relevant)

## Trajectory File Convention (traj.npz)

**Standardized keys for Parts 3 and 4:**
- `t`: Time vector, shape `(N+1,)`, `t[k] = k * Ts`
- `x`: True state trajectory, shape `(n, N+1)`, stores `x[0]` through `x[N]`
- `xhat`: Estimated state trajectory, shape `(n, N+1)`, stores `xhat[0]` through `xhat[N]`
- `y`: Output trajectory, shape `(p, N+1)`, stores `y[0]` through `y[N]`
- `e`: Estimation error, shape `(n, N+1)`, stores `e[0]` through `e[N]` where `e[k] = x[k] - xhat[k]`

**Input key difference (by design):**
- **Part 3:** Uses key `u`, shape `(3, N)`, stores `u[0]` through `u[N-1]` (3 inputs: u1, u2, u3)
- **Part 4:** Uses key `u_red`, shape `(2, N)`, stores `u_red[0]` through `u_red[N-1]` (2 inputs: u1, u2)

**Usage in Parts 5-7:**
- Load Part 3 trajectories: `traj = np.load('python/part3/outputs/traj.npz')`, access `traj['u']` for inputs
- Load Part 4 trajectories: `traj = np.load('python/part4/outputs/traj.npz')`, access `traj['u_red']` for inputs
- All other keys (`t`, `x`, `xhat`, `y`, `e`) are consistent across Parts 3 and 4

## Part 5-6 Stochastic Model Parameters (Frozen)

**Noise covariances (frozen for Parts 5-7):**
- **Actuator noise:** `Qw = 0.05 * I_3` (w ~ N(0, 0.05 I_3))
- **Sensor noise:** `Rv = 0.1 * I_2` (v ~ N(0, 0.1 I_2))
- **Process noise:** `Qx = Bd @ Qw @ Bd.T` (12×12, derived from Qw)
- **Random seed:** `seed = 42` (for reproducibility in Python simulations)

**Stochastic model (modeling choice):**
```
x_{k+1} = Ad @ x_k + Bd @ u_k + Bd @ w_k
y_k     = Cmeas @ x_k + v_k
```

**Critical modeling choice:** Process noise `w_k` enters the system via `Bd` (same channel as control input `u_k`). This is verified from exam statement: "noise enters through `B_d`" per `docs/sources/final_exam_extract.md` Section 7.

**Source:** Final exam Question 5, verified in `docs/sources/final_exam_extract.md` Section 7

**Used in:** Parts 5, 6, 7 (and must be used consistently)

**Output naming convention (Part 5+):**
- `y_true[k] = Cmeas @ x[k]` (true output, no noise)
- `y_meas[k] = Cmeas @ x[k] + v[k]` (measured output, with noise)
- `yhat[k] = Cmeas @ xhat[k]` (estimated output from Kalman filter)

**Output usage conventions:**
- **For cost computation**: Use `y_true` (does not penalize uncontrollable measurement noise)
  - Part 6 cost: `J_true = Σ(u^T u + y_true1² + y_true6²)`
- **For plots**: 
  - "True vs estimated" plots: Show `y_true` and `yhat`
  - "Measurement vs estimate" plots: Show `y_meas` and `yhat`
  - Label clearly: `y_meas` is noisy measurement, not "true"
- **For metrics**:
  - Tracking RMS: RMS of `(y_true - yhat)` (how well filter tracks true output)
  - Innovation RMS: RMS of `(y_meas - yhat)` (measurement residual)

**Source:** Final exam Question 5, verified in `docs/sources/final_exam_extract.md` Section 7

**Used in:** Parts 5, 6, 7 (and should be used consistently)

## Part 5 Baseline Metrics

**Kalman filter design:**
- Kalman gain Lk: Shape (12, 2), saved in `python/part5/outputs/Lk_matrix.npy`
- Estimator spectral radius: 0.999547 (< 1.0, stable)
- Innovation covariance S: (2, 2), condition number ≈ 1.0

**Estimation performance (open-loop, u=0):**
- Overall RMS estimation error (full window): 9.607036e-01
- Overall RMS estimation error (steady-state, last 20%): 5.312798e-01
- RMS error e_x1 (steady-state): 1.075163e-01
- RMS error e_x6 (steady-state): 5.154161e-02

**Used for:** Part 6 comparison baseline (open-loop estimation performance)

## Part 6 LQG Baseline Metrics

**Controller validation:**
- K matrix fingerprint: `||K||_F`, `max(|K|)`, shape (3, 12)
- Controller uses estimated state: `u[k] = -K @ xhat[k]` (verified)
- Early time control magnitudes: `max(|u[:,0:20]|)`, `max(||u[k]||_inf, k=0..20)`

**Cost computation (Part 6):**
- `J_true = Σ(u^T u + y_true1² + y_true6²)` using `y_true = Cmeas @ x` (official metric)
- `J_meas = Σ(u^T u + y_meas1² + y_meas6²)` using `y_meas = Cmeas @ x + v` (for comparison)
- Justification: `J_true` does not penalize uncontrollable measurement noise

**Used for:** Part 7 baseline reference and validation

## Part 7 Sensor Augmentation Metrics

**Source:** Final exam Question 7, verified in `docs/sources/final_exam_extract.md` Section 9

### Part 7 Case 1: 4 Sensors (x1, x2, x5, x6)

**Measurement matrix C_case1 (4×12):**
```
C_case1 = [[1 0 0 0 0 0 0 0 0 0 0 0]
           [0 1 0 0 0 0 0 0 0 0 0 0]
           [0 0 0 0 1 0 0 0 0 0 0 0]
           [0 0 0 0 0 1 0 0 0 0 0 0]]
```

**Noise covariances:**
- Qw = 0.05 * I_3 (unchanged from Part 6)
- Rv = 0.1 * I_4 (4 sensors)
- seed = 42

**Kalman filter design:**
- Lk_case1 shape: (12, 4), saved in `python/part7/outputs/Lk_case1_matrix.npy`
- Estimator spectral radius: 0.998968 (< 1.0, stable)

**LQG metrics:**
- J_true: 4.017590e+02
- max|u| overall: 4.086037e-01
- RMS estimation error (full): 8.655016e-01
- RMS estimation error (SS, last 20%): 4.643125e-01

### Part 7 Case 2: 6 Sensors (x1, x2, x3, x4, x5, x6)

**Measurement matrix C_case2 (6×12):**
```
C_case2 = [[1 0 0 0 0 0 0 0 0 0 0 0]
           [0 1 0 0 0 0 0 0 0 0 0 0]
           [0 0 1 0 0 0 0 0 0 0 0 0]
           [0 0 0 1 0 0 0 0 0 0 0 0]
           [0 0 0 0 1 0 0 0 0 0 0 0]
           [0 0 0 0 0 1 0 0 0 0 0 0]]
```

**Noise covariances:**
- Qw = 0.05 * I_3 (unchanged from Part 6)
- Rv = 0.1 * I_6 (6 sensors)
- seed = 42

**Kalman filter design:**
- Lk_case2 shape: (12, 6), saved in `python/part7/outputs/Lk_case2_matrix.npy`
- Estimator spectral radius: 0.998415 (< 1.0, stable)

**LQG metrics:**
- J_true: 3.709941e+02
- max|u| overall: 4.086037e-01
- RMS estimation error (full): 7.132968e-01
- RMS estimation error (SS, last 20%): 2.702939e-01

### Part 7 Comparison Summary

| Metric                      | Part 6 (2) | Case 1 (4) | Case 2 (6) |
|-----------------------------|----------:|----------:|----------:|
| Estimator spectral radius   | 0.999547  | 0.998968  | 0.998415  |
| J_true                      | 4.261e+02 | 4.018e+02 | 3.710e+02 |
| RMS error (full)            | 9.573e-01 | 8.655e-01 | 7.133e-01 |
| RMS error (SS)              | 5.593e-01 | 4.643e-01 | 2.703e-01 |

**Conclusion:** More sensors help with BOTH estimation AND regulation.

**Key invariants verified:**
- K matrix unchanged from Part 3 (LQR depends on cost, not sensors)
- Qw = 0.05 * I_3 unchanged (process noise enters via Bd)
- seed = 42 unchanged (reproducibility)
- Cost function unchanged: J = sum(u^T u + y1^2 + y6^2)

## Notes

- All values above are from Parts 0-7 execution results
- These invariants ensure consistency when implementing Part 8
- If any value needs to change for Part 8, document the deviation and justification
- Check this document before starting Part 8 to avoid accidental drift
- Array indexing convention is frozen globally (see also `docs/00_anchor.md` "Array Indexing Convention" section)
- Noise settings and RNG seed are frozen to ensure reproducible comparisons across Parts 5-7
- Output naming conventions (y_true, y_meas, yhat) are frozen for Parts 5-7