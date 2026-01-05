# Cross-Part Invariants: Parts 2-4

**DO NOT CHANGE** these values when implementing Parts 5-7. These are frozen parameters used consistently across Parts 2-4.

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

## Notes

- All values above are from Parts 2-4 execution results
- These invariants ensure consistency when implementing Parts 5-7
- If any value needs to change for Parts 5-7, document the deviation and justification
- Check this document before starting Parts 5-7 to avoid accidental drift
- Array indexing convention is frozen globally (see also `docs/00_anchor.md` "Array Indexing Convention" section)