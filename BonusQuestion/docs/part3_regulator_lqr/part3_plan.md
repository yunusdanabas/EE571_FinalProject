# Part 3: Regulator 1 Implementation (Discrete LQR)

## Scope

Implement the discrete-time infinite-horizon LQR regulator using Python. Compute the feedback gain `K_LQR` and integrate it into the simulation loop.

## Inputs

- `code/vehicle_tracking.py` (with discretization from Part 2: `Ad`, `Bd` available)
- `docs/part2_discretization/part2_closeout.md` (discretization details)
- Python libraries: numpy, scipy, matplotlib

## Outputs

- Modified `code/vehicle_tracking.py` with LQR controller
- `docs/part3_regulator_lqr/part3_closeout.md` with:
  - `Q` and `R` values and rationale
  - Location where `K_LQR` is computed
  - Baseline plot saved to `results/plots/`
  - How to run

## Steps

1. **Choose Q matrix (5x5, positive semi-definite)**
   - Consider relative importance of error states:
     - `v_y`: Lateral velocity
     - `r`: Yaw rate
     - `e_y`: Cross-track error (important for tracking)
     - `e_psi`: Heading error (important for tracking)
     - `e_v`: Speed error (important for tracking)
   - Typical approach: Diagonal matrix with appropriate weights
   - Document rationale in closeout

2. **Choose R matrix (2x2, positive definite)**
   - Penalize control effort:
     - `R[0,0]`: Steering effort penalty
     - `R[1,1]`: Acceleration effort penalty
   - Document rationale in closeout

3. **Compute LQR gain using scipy**
   ```python
   from scipy.linalg import solve_discrete_are
   import numpy as np
   
   # Solve discrete algebraic Riccati equation
   P = solve_discrete_are(Ad, Bd, Q, R)
   
   # Compute feedback gain
   K_LQR = np.linalg.inv(R + Bd.T @ P @ Bd) @ (Bd.T @ P @ Ad)
   ```
   - Verify `K_LQR` shape is (2, 5)

4. **Integrate into simulation loop**
   - After error computation:
     ```python
     # Build error state vector
     x_e = np.array([vy, r, ey, epsi, ev])
     
     # Compute regulation input
     u_reg = -K_LQR @ x_e
     steering_reg = u_reg[0]
     ax_reg = u_reg[1]
     
     # Combine with feedforward
     steering_input = steering_feed_forward + steering_reg
     throttle = throttle_feed_forward + ax_reg
     ```
   - Apply saturations after combination

5. **Run baseline simulation**
   - Use default initial condition (scale = 1)
   - Run simulation for full duration (25 seconds)
   - Verify no errors occur

6. **Generate and save plots**
   - Trajectory plot
   - Error plots (`e_y`, `e_psi`, `e_v`)
   - Input plots (steering, acceleration)
   - Save to `results/plots/` with naming like `lqr_scale1_trajectory.png`

## Acceptance Checks

- [ ] `Q` is 5x5, positive semi-definite
- [ ] `R` is 2x2, positive definite
- [ ] `K_LQR` shape is (2, 5)
- [ ] Code runs without errors
- [ ] Controller stabilizes error states
- [ ] Feedforward and regulation are properly combined
- [ ] Saturations are applied
- [ ] Plots show reasonable tracking performance
- [ ] Baseline plot is saved

## Risks / Gotchas

- **Q and R tuning**: May need iteration for good performance
- **Sign convention**: LQR gives `u = -K @ x`, which is correct for regulation
- **Error state order**: Must match `[v_y, r, e_y, e_psi, e_v]` exactly
- **scipy version**: Ensure scipy has `solve_discrete_are` function
- **Matrix multiplication**: Use `@` operator for matrix multiplication in numpy

## Handoff to Part 4

Part 4 agent should:
- Use similar integration pattern for pole placement
- Understand the error state vector construction
- Understand feedforward combination pattern
