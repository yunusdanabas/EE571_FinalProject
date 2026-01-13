# Part 4: Regulator 2 Implementation (Pole Placement)

## Scope

Implement the discrete-time pole placement regulator with **real poles only**. Compute feedback gain `K_PP` and integrate it into the simulation loop identically to LQR.

## Inputs

- `code/vehicle_tracking.py` (with LQR from Part 3)
- `docs/part3_regulator_lqr/part3_closeout.md` (LQR integration pattern)
- `Ad`, `Bd` matrices (from Part 2)
- Python libraries: numpy, scipy

## Outputs

- Modified `code/vehicle_tracking.py` with pole placement controller
- `docs/part4_regulator_poleplacement/part4_closeout.md` with:
  - Chosen pole locations (listed)
  - Where `K_PP` is computed
  - Baseline plot saved to `results/plots/`
  - How to run

## Steps

1. **Choose real pole locations**
   - **Constraint: Real poles only** (no complex poles allowed)
   - Poles must be inside unit circle for stability (discrete-time)
   - System has 5 states, so choose 5 poles
   - Typical range: 0.7 to 0.95 (fast but stable)
   - Example: `desired_poles = np.array([0.85, 0.80, 0.75, 0.70, 0.65])`
   - Document chosen poles in closeout

2. **Compute pole placement gain using scipy**
   ```python
   from scipy.signal import place_poles
   import numpy as np
   
   desired_poles = np.array([0.85, 0.80, 0.75, 0.70, 0.65])  # example
   result = place_poles(Ad, Bd, desired_poles)
   K_PP = result.gain_matrix
   ```
   - Verify `K_PP` shape is (2, 5)

3. **Verify closed-loop poles**
   ```python
   # Compute closed-loop matrix
   A_cl = Ad - Bd @ K_PP
   
   # Verify eigenvalues
   eig_cl = np.linalg.eigvals(A_cl)
   print("Closed-loop eigenvalues:", eig_cl)
   
   # Check all real and inside unit circle
   assert np.all(np.isreal(eig_cl)), "Poles must be real"
   assert np.all(np.abs(eig_cl) < 1), "Poles must be inside unit circle"
   ```

4. **Integrate into simulation loop**
   - Use same pattern as LQR:
     ```python
     # Build error state vector
     x_e = np.array([vy, r, ey, epsi, ev])
     
     # Compute regulation input
     u_reg = -K_PP @ x_e
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
   - Save to `results/plots/` with naming like `pp_scale1_trajectory.png`

## Acceptance Checks

- [ ] All chosen poles are real (no complex poles)
- [ ] All chosen poles are inside unit circle (stable)
- [ ] `K_PP` shape is (2, 5)
- [ ] Closed-loop eigenvalues match desired poles (within tolerance)
- [ ] All closed-loop eigenvalues are real
- [ ] Code runs without errors
- [ ] Controller stabilizes error states
- [ ] Plots show reasonable tracking performance
- [ ] Baseline plot is saved

## Risks / Gotchas

- **Real poles only**: Assignment requires real poles, no complex poles
- **scipy.signal.place_poles**: May have issues with certain pole configurations
- **Numerical issues**: Pole placement can be sensitive to pole locations
- **Same integration pattern**: Use identical code structure as LQR for fair comparison
- **Verify eigenvalues**: Always check closed-loop poles match desired poles

## Handoff to Part 5

Part 5 agent should:
- Have both regulators working (LQR and Pole Placement)
- Understand how to switch between regulators
- Be ready to run 6 cases total (2 regulators x 3 scales)
