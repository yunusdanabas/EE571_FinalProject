# Part 5: Experiments and Comparisons (6 Required Runs)

## Scope

Run all 6 required cases (2 regulators x 3 initial error scales) and generate comparison plots and performance metrics using Python.

## Inputs

- `code/vehicle_tracking.py` (with both regulators implemented)
- `docs/part3_regulator_lqr/part3_closeout.md` (LQR details)
- `docs/part4_regulator_poleplacement/part4_closeout.md` (Pole placement details)

## Outputs

- All 6 runs executed successfully
- Comparison plots saved to `results/plots/`
- Metrics summary table in `results/logs/metrics_summary.md`
- `docs/part5_experiments_comparison/part5_closeout.md`

## Steps

1. **Implement run matrix**
   ```python
   configs = [
       {'regulator': 'LQR', 'scale': 1},
       {'regulator': 'LQR', 'scale': 2},
       {'regulator': 'LQR', 'scale': 3},
       {'regulator': 'PP',  'scale': 1},
       {'regulator': 'PP',  'scale': 2},
       {'regulator': 'PP',  'scale': 3},
   ]
   ```

2. **Implement initial condition scaling**
   - Baseline offsets:
     - `dX = -2.0 m`
     - `dY = +1.0 m`
     - `dpsi = +8 deg`
     - `dvx = -5 m/s`
   - For scale `s in {1, 2, 3}`:
     ```python
     X0 = Xref[0] + s * dX
     Y0 = Yref[0] + s * dY
     psi0 = psiref[0] + s * np.deg2rad(8)
     vx0 = Vx0 + s * dvx
     vy0 = 0.0
     r0 = 0.0
     ```

3. **For each of 6 runs:**
   - Set regulator type (LQR or PP)
   - Set initial condition scale (1, 2, or 3)
   - Run simulation for full duration
   - Log trajectory, errors, inputs
   - Compute metrics (see step 4)
   - Generate plots (see step 5)

4. **Compute metrics for each run**
   ```python
   # RMS errors
   rms_ey = np.sqrt(np.mean(ey_log**2))
   rms_epsi = np.sqrt(np.mean(epsi_log**2))
   rms_ev = np.sqrt(np.mean(ev_log**2))
   
   # Max absolute errors
   max_ey = np.max(np.abs(ey_log))
   max_epsi = np.max(np.abs(epsi_log))
   max_ev = np.max(np.abs(ev_log))
   
   # Max inputs
   max_steering = np.max(np.abs(u_log[:, 0]))
   max_accel = np.max(u_log[:, 1])
   min_accel = np.min(u_log[:, 1])
   ```

5. **Generate comparison plots**
   - **Trajectory plots**: Overlay reference, LQR, and PP paths
   - **Error plots**: Overlay `e_y`, `e_psi`, `e_v` for both regulators
   - **Input plots**: Overlay steering and acceleration with saturation limits
   - Use matplotlib with proper legends, labels, and titles

6. **Save outputs**
   - Save plots to `results/plots/` with consistent naming:
     - `trajectory_scale1.png`, `trajectory_scale2.png`, `trajectory_scale3.png`
     - `errors_scale1.png`, `errors_scale2.png`, `errors_scale3.png`
     - `inputs_scale1.png`, `inputs_scale2.png`, `inputs_scale3.png`
   - Save metrics to `results/logs/metrics_summary.md`

7. **Create metrics summary table**
   ```markdown
   | Regulator | Scale | RMS e_y | Max e_y | RMS e_psi | Max e_psi | RMS e_v | Max e_v |
   |-----------|-------|---------|---------|-----------|-----------|---------|---------|
   | LQR       | 1     | ...     | ...     | ...       | ...       | ...     | ...     |
   | LQR       | 2     | ...     | ...     | ...       | ...       | ...     | ...     |
   | ...       | ...   | ...     | ...     | ...       | ...       | ...     | ...     |
   ```

## Acceptance Checks

- [ ] All 6 runs complete without errors
- [ ] Initial condition scaling is correctly implemented
- [ ] All required plots are generated and saved
- [ ] Plots clearly show comparison between regulators
- [ ] Metrics table is complete
- [ ] Plots have proper labels, legends, and titles
- [ ] Saturation limits are visible on input plots

## Risks / Gotchas

- **Fair comparison**: Both regulators must use same initial conditions
- **Same feedforward**: Both regulators use same feedforward terms
- **Same saturations**: Both regulators subject to same limits
- **Plot clarity**: Use distinct line styles/colors for each regulator
- **Metrics consistency**: Compute same metrics for both regulators

## Handoff to Part 6

Part 6 agent should:
- Have all plots and metrics needed for report
- Understand performance comparison results
- Be ready to incorporate findings into report conclusion
