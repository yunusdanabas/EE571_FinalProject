---
name: Part 3 LQR Controller Design
overview: Design and implement a discrete-time infinite-horizon LQR controller for the 6-mass spring system. The controller uses estimated states from the Part 2 observer (u[k] = -K xhat[k]) and minimizes the cost J = sum_k (u^T u + y1^2 + y6^2). The implementation includes LQR design via DARE solver, closed-loop simulation with observer in the loop, cost computation, and required plots and metrics.
todos:
  - id: create_docs
    content: Create docs/05_part3_lqr/plan.md and closeout.md with all required sections, gates, and templates
    status: completed
  - id: create_code_structure
    content: Create python/part3/ directory with run_lqr_with_observer.py and outputs/ subdirectory
    status: completed
  - id: implement_lqr_design
    content: "Implement LQR design: cost matrix construction (Q, R), DARE solver, K computation, stabilizability check, spectral radius validation"
    status: completed
  - id: implement_simulation
    content: "Implement closed-loop simulation with observer in loop: plant, observer, and controller dynamics, with explicit xhat usage validation"
    status: completed
  - id: implement_cost_metrics
    content: "Implement cost computation: stage cost per time step, total cost J, max input magnitudes, all logged to results.txt"
    status: completed
  - id: implement_plotting
    content: "Generate required plots: outputs_y1_y6.png, inputs_u1_u2_u3.png, optional estimation_error_norm.png"
    status: completed
  - id: validate_gates
    content: "Implement all validation gates: cost matrices, LQR design, simulation dimensions, controller xhat usage, cost finiteness"
    status: completed
---

# Part 3: Discrete-Time LQR Controller Design with Observer

## Objective

Design a discrete-time infinite-horizon LQR controller for the discrete model with Ts = 0.01, 3 inputs, using estimated states from the Part 2 observer. The control law is u[k] = -K xhat[k] (uses xhat, not x). The cost to minimize is:

J = sum_{k=0}^{N-1} (u[k]^T u[k] + y1[k]^2 + y6[k]^2)

where y1 and y6 are displacements x1 and x6 (the measured outputs from Part 2).

## Exam Mapping

Source: `docs/sources/final_exam_extract.md` Section 5 (Part 3 Requirement)

- Control uses estimated states: u[k] = -K xhat[k]
- Cost definition: J = sum (u^T u + y1^2 + y6^2)
- Part 2 C matrix and initial conditions are reused (verified from exam extract)

## Implementation Structure

### A. Documentation (`docs/05_part3_lqr/`)

1. **plan.md** - Implementation plan with:

   - Objective and exam mapping
   - Exact commands to run
   - Pass/fail gates with explicit criteria
   - Statement that control uses xhat (not x) with gate validation
   - Simulation horizon N and cost accumulation convention

2. **closeout.md** - Results template with:

   - Environment and commit hash section
   - Checklists for each gate
   - Tables for K matrix, spectral radii, total cost J, max input magnitudes
   - Artifact list with exact paths

### B. Code Implementation (`python/part3/`)

1. **run_lqr_with_observer.py** - Main entrypoint:

   - Loads continuous model and discretizes via `python/utils/build_model.py` (ZOH, Ts=0.01)
   - Imports Part 2 C matrix from `python/part2/observer_design.py::get_part2_C_matrix()`
   - Imports Part 2 initial conditions from `python/part2/run_observer_sim.py::get_part2_initial_conditions()`
   - Imports Part 2 observer gain L from `python/part2/observer_design.py::design_observer()`
   - Designs LQR controller (DARE solver)
   - Runs closed-loop simulation with observer in the loop
   - Computes cost metrics
   - Generates plots and saves results

2. **outputs/** directory for generated artifacts

### C. LQR Design Steps

1. **Cost Mapping to Q and R**:

   - Define Cy = Part 2 C matrix (2×12, outputs [x1, x6])
   - Define Q = Cy^T @ Cy (12×12, symmetric)
   - Define R = I3 (3×3 identity for u^T u)
   - Log Q and R shapes and nonzero structure

2. **LQR Design on Discrete-Time System**:

   - Check stabilizability of (Ad, Bd) using rank-based check
   - Solve discrete-time DARE: P = solve_discrete_are(Ad, Bd, Q, R)
   - Compute K from P: K = (R + Bd^T @ P @ Bd)^(-1) @ (Bd^T @ P @ Ad)
   - Compute closed-loop matrix: Acl = Ad - Bd @ K
   - Verify spectral_radius(Acl) < 1.0 (hard gate)

3. **Closed-Loop Simulation with Observer**:

   - Plant: x[k+1] = Ad @ x[k] + Bd @ u[k], y[k] = Cmeas @ x[k]
   - Observer: xhat[k+1] = Ad @ xhat[k] + Bd @ u[k] + L @ (y[k] - Cmeas @ xhat[k])
   - Controller: u[k] = -K @ xhat[k] (CRITICAL: uses xhat, not x)
   - Simulation horizon: N = 1000 steps (10 seconds at Ts=0.01)
   - Use float64, no noise

4. **Cost and Metrics Computation**:

   - Stage cost: stage_cost[k] = u[k]^T @ u[k] + y_cost[0]^2 + y_cost[1]^2

where y_cost = Cy @ x[k] (or equivalently y_cost = y[k] since Cy = Cmeas)

   - Total cost: J = sum_{k=0}^{N-1} stage_cost[k]
   - Max input magnitudes: max_abs_u1, max_abs_u2, max_abs_u3, max_abs_u_overall

### D. Validation Gates

1. **Cost Matrix Gates**:

   - Q is 12×12, symmetric
   - R is 3×3, symmetric, positive definite

2. **LQR Design Gates**:

   - (Ad, Bd) stabilizability check passes (rank check)
   - DARE solver succeeds without warnings or NaNs
   - spectral_radius(Acl) < 1.0 (hard fail if not)

3. **Simulation Gates**:

   - Dimensions consistent: x (12,), xhat (12,), u (3,), y (2,)
   - Controller uses xhat, not x (explicit check/logged assertion)
   - Simulation runs for N steps without divergence
   - Log max absolute state magnitude and max absolute input magnitude

4. **Cost Gates**:

   - J is finite and non-negative
   - Metrics logged and written to outputs/results.txt

### E. Expected Artifacts

**python/part3/outputs/results.txt**:

- N, Ts, initial conditions used
- Cy and R definitions summary
- K matrix (shape and values)
- spectral_radius(Ad - Bd @ K)
- Total cost J and max input magnitudes
- Statement that control uses xhat

**Plots**:

- outputs_y1_y6.png: x1 and x6 over time (or y[0], y[1])
- inputs_u1_u2_u3.png: All three inputs over time
- estimation_error_norm.png (optional): ||x - xhat|| over time

## Commands to Run

```bash
python python/part3/run_lqr_with_observer.py
```

## Key Implementation Details

1. **Reuse Part 2 Components**:

   - Import `get_part2_C_matrix()` from `python/part2/observer_design.py`
   - Import `get_part2_initial_conditions()` from `python/part2/run_observer_sim.py`
   - Import `design_observer()` from `python/part2/observer_design.py` to get L
   - Do NOT redesign observer in Part 3

2. **Cost Computation**:

   - Cy = Part 2 C matrix (already outputs [x1, x6])
   - Q = Cy^T @ Cy
   - R = np.eye(3)
   - Stage cost uses actual plant outputs y[k] = Cmeas @ x[k] for cost accumulation

3. **Controller Validation**:

   - Add explicit check that u[k] depends on xhat[k], not x[k]
   - Log a statement confirming control uses xhat

4. **Simulation Convention**:

   - N = 1000 steps (10 seconds)
   - Cost accumulated from k=0 to k=N-1
   - Document convention in results.txt