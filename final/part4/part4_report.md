# Part 4: LQR Controller Design with Reduced Input Report

## Objective

Redesign the LQR controller with only two inputs (u1, u2), removing u3, and compare performance with Part 3. The controller uses estimated states from the Part 2 observer and minimizes the same cost function: J = Σ(u_red^T u_red + y1^2 + y6^2).

## Approach

1. **Create reduced input matrix:**
   - Remove third column from B matrix: Bd_red = Bd[:, [0, 1]]
   - Reduced input dimension: u_red in R² (u1 and u2 only, u3 removed)

2. **Redesign LQR controller:**
   - State weight: Q = Cy^T @ Cy (same as Part 3)
   - Input weight: R_red = I₂ (2×2 identity matrix, reduced from Part 3's I₃)
   - Solve discrete-time algebraic Riccati equation (DARE) with (Ad, Bd_red, Q, R_red)
   - Compute reduced LQR gain: K_red = (R_red + Bd_red^T P Bd_red)^(-1) (Bd_red^T P Ad)

3. **Simulate closed-loop system:**
   - Control law: u_red[k] = -K_red @ xhat[k] (uses estimated states)
   - Observer: Same Part 2 observer as Part 3
   - Simulation horizon: N = 1000 steps (10 seconds at Ts = 0.01s)

4. **Compare with Part 3:**
   - Compute cost metrics and compare total cost J_red vs J
   - Analyze performance degradation from removing one input

## Key Results

- **LQR gain K_red shape:** (2, 12)
- **Closed-loop spectral radius:** 0.999518 (system is stable, similar to Part 3)
- **Total cost J_red:** 1.348077×10⁸
- **Maximum input magnitude:** 4.951072×10³
- **Part 3 baseline cost J:** 9.057478×10⁷
- **Cost increase:** 48.84% (performance degradation from removing u3)

### Results Summary

```
Part 4: LQR Controller Design with Reduced Input (u3 Removed) - Results
============================================================

LQR Design:
  K_red shape: (2, 12)
  Closed-loop spectral radius: 0.999518
Cost Metrics:
  Total cost J_red: 1.348077e+08
  Max |u_red|: 4.951072e+03
Comparison with Part 3:
  Part 3 cost J: 9.057478e+07
  Part 4 cost J_red: 1.348077e+08
  Cost increase: 48.84%
Observer:
  Spectral radius: 0.800000
```

## Figures

1. **Outputs y1 and y6** (`outputs_y1_y6.png`): Shows the controlled outputs (displacements of masses 1 and 6). The controller successfully regulates both outputs toward zero, similar to Part 3.

2. **Inputs u1 and u2** (`inputs_u1_u2.png`): Displays the two control inputs applied to the system (u3 removed). Inputs are computed from estimated states via u_red[k] = -K_red @ xhat[k]. The reduced input controller requires higher input magnitudes to achieve similar performance. The full-window plot shows inputs settling to zero after the initial transient. The zoomed version (`inputs_u1_u2_05sec.png`) shows the detailed transient behavior in the first 0.5 seconds.

3. **Estimation Error Norm** (`estimation_error_norm.png`): Shows the evolution of the estimation error norm ||x - xhat|| over time. The observer error converges, enabling accurate state estimation for control, same as Part 3. The full-window plot shows convergence over the full 10-second simulation. The zoomed version (`estimation_error_norm_05sec.png`) shows the detailed convergence behavior in the first 0.5 seconds.

## Explanation of Part 4

Part 4 implements a **reduced input LQR controller** that uses only two inputs (u1 and u2) instead of three, removing u3. This demonstrates the impact of reducing actuator authority on control performance.

### What is Reduced Input Control?

Reduced input control means we have fewer actuators available to control the system. In Part 3, we had three inputs (u1, u2, u3), but in Part 4, we remove u3, leaving only u1 and u2. This reduces our control authority - we have fewer "knobs" to turn to control the system.

### How the Reduced Input Controller Works

1. **Input Matrix Reduction:**
   - Part 3: B matrix has 3 columns (u1, u2, u3) → Bd shape: (12, 3)
   - Part 4: Remove third column → Bd_red = Bd[:, [0, 1]] → shape: (12, 2)
   - This means u3 can no longer affect the system

2. **LQR Controller Redesign:**
   - State weight Q remains the same: Q = Cy^T @ Cy (penalizes x1 and x6)
   - Input weight changes: R_red = I₂ (2×2 identity, down from I₃)
   - Solve DARE with reduced input matrix: (Ad, Bd_red, Q, R_red)
   - Compute reduced gain: K_red (2×12, down from 3×12 in Part 3)

3. **Control Law:**
   - Part 3: u[k] = -K @ xhat[k] (3 inputs)
   - Part 4: u_red[k] = -K_red @ xhat[k] (2 inputs: u1 and u2)
   - Same observer is used (from Part 2)

4. **Closed-Loop System:**
   - Plant: x[k+1] = Ad @ x[k] + Bd_red @ u_red[k]
   - Observer: xhat[k+1] = Ad @ xhat[k] + Bd_red @ u_red[k] + L @ (y[k] - yhat[k])
   - Controller: u_red[k] = -K_red @ xhat[k]

### Why This Matters

When you remove an input (u3), you lose control authority. The remaining inputs (u1, u2) must work harder to compensate. This results in:
- **Higher control effort** (larger input magnitudes)
- **Higher total cost** (less efficient control)
- **Slower or less effective regulation** (depending on the system)

However, the system is still stabilizable with just u1 and u2, demonstrating that these two inputs provide sufficient control authority to stabilize the 6-mass spring system.

## Findings

1. **Controller stability:** The reduced input LQR controller stabilizes the system with a closed-loop spectral radius of 0.999518, indicating stable but slow convergence (very close to unity, similar to Part 3).

2. **Performance degradation:** Removing u3 results in a 48.84% increase in total cost (J_red = 1.35×10⁸ vs Part 3's J = 9.06×10⁷). This demonstrates the impact of reducing actuator authority on control performance.

3. **Input magnitude increase:** The maximum input magnitude increased from 3.60×10³ (Part 3) to 4.95×10³ (Part 4), indicating that the remaining two inputs must work harder to compensate for the loss of u3.

4. **System controllability:** The system remains stabilizable with reduced inputs, confirming that u1 and u2 provide sufficient control authority to stabilize the 6-mass spring system.

5. **Output regulation:** Both outputs (y1 and y6) are still regulated toward zero, demonstrating effective control despite the reduced input dimension.

6. **Trade-off analysis:** The results illustrate the fundamental trade-off between actuator availability and control performance. Removing one input (u3) reduces design flexibility and requires higher control effort from the remaining actuators to achieve similar regulation performance.

The reduced input LQR controller successfully stabilizes the system but at the cost of increased total cost and higher input magnitudes, demonstrating the value of having all three inputs available for control.

## Comparison with Original Implementation

Comparison of simplified implementation (`final/part4/`) with original implementation (`python/part4/`):

| Metric | Original | Simplified | Match |
|--------|----------|------------|-------|
| **LQR Gain K_red** | (2, 12) | (2, 12) | ✓ Exact match |
| **Closed-loop spectral radius** | 0.999518 | 0.999518 | ✓ Exact match |
| **Observer gain L** | From `python/part2` | From `final/part2` | ✗ Different design |
| **Total cost J_red** | 5.844×10⁷ | 1.348×10⁸ | ✗ Different (due to different L) |
| **Max \|u_red\|** | 3.322×10³ | 4.951×10³ | ✗ Different (due to different L) |
| **Cost increase vs Part 3** | 49.16% | 48.84% | ✓ Very similar |

### Key Observations

1. **LQR Controller Design (K_red):** The LQR gain matrix K_red matches **exactly** between implementations (verified numerically with max difference = 0.00e+00), confirming that the controller design logic is correct.

2. **Closed-Loop Stability:** The closed-loop spectral radius matches exactly (0.999518), confirming that both implementations produce the same closed-loop system dynamics.

3. **Observer Design (L):** The observer gain L differs because:
   - Original uses `design_observer()` from `python/part2` (more complex implementation)
   - Simplified uses `design_observer_gain()` from `final/part2` (simplified implementation per anchor guidelines)
   - Both use pole placement with range (0.4, 0.8), but implementation details differ
   - Both produce stable observers with spectral radius ≈ 0.8

4. **Trajectories and Costs:** Due to the different observer gains L, the closed-loop trajectories differ, leading to:
   - Different total cost J_red (original: 5.84×10⁷, simplified: 1.35×10⁸)
   - Different input magnitudes
   - Different output trajectories (but qualitatively similar behavior)

5. **Cost Increase Percentage:** The cost increase relative to Part 3 is very similar (original: 49.16%, simplified: 48.84%), confirming that the **relative performance degradation** from removing u3 is consistent between implementations.

6. **Controller Correctness:** The exact match of the LQR gain K_red and closed-loop spectral radius confirms that the controller design logic is correct. The differences in trajectories and costs are due to the observer design difference, which is expected when using the simplified `final/part2` utilities.

### Original Implementation Results (for reference)

From `python/part4/outputs/results.txt`:
- Total cost J_red = 5.844240×10⁷
- Max \|u_red\| overall = 3.321916×10³
- Max \|u1\| = 3.321916×10³
- Max \|u2\| = 4.631007×10¹
- Part 3 baseline J = 3.918117×10⁷
- Cost increase: 49.16%

### Summary

The simplified implementation correctly designs the reduced input LQR controller (K_red matches exactly), and the relative performance characteristics (cost increase percentage) are consistent with the original implementation. The absolute cost and trajectory differences are due to using the simplified Part 2 observer from `final/part2`, which is consistent with the anchor document's guidelines to use simplified utilities.