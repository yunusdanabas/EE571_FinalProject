# Part 5 Prompt: Kalman Filter Design

## Copy and paste this prompt to ChatGPT:

---

Now I'm sending you the materials for **Part 5: Kalman Filter Design**.

**Objective:** Design a steady-state Kalman filter for the stochastic 6-mass spring system with process noise (actuator) and measurement noise (sensor). This is a transition from deterministic to stochastic control.

Please write the LaTeX section for Part 5 that includes:

1. **Section Header**: `\section{Part 5: Kalman Filter Design}`

2. **Objective Subsection**: Explain the difference between deterministic observer (Part 2) and stochastic Kalman filter

3. **Noise Model Subsection**:
   - Process noise (actuator): $Q_w = 0.05 \cdot I_3$
   - Measurement noise (sensor): $R_v = 0.1 \cdot I_2$
   - State-space process noise: $Q_x = B_d Q_w B_d^T$
   - Stochastic system: $x[k+1] = A_d x[k] + B_d u[k] + B_d w[k]$
   - Noisy measurements: $y_{meas}[k] = C x[k] + v[k]$

4. **Kalman Filter Design Subsection**:
   - DARE for estimator: $P = \text{solve\_dare}(A_d^T, C^T, Q_x, R_v)$
   - Kalman gain: $L_k = P C^T (C P C^T + R_v)^{-1}$
   - Filter update: $\hat{x}[k+1] = A_d \hat{x}[k] + B_d u[k] + L_k (y_{meas}[k] - C\hat{x}[k])$

5. **Results Subsection**:
   - Kalman gain Lk shape: (12, 2)
   - Estimator spectral radius: 0.999547
   - RMS estimation error (full): 0.9607
   - RMS estimation error (steady-state): 0.5313
   - Include figures: true vs estimated outputs, estimation error norm, per-state RMS bar chart

6. **Comparison with Part 2 Observer** table:
   ```latex
   \begin{table}[H]
       \centering
       \begin{tabular}{lcc}
           \toprule
           Feature & Part 2 Observer & Part 5 Kalman Filter \\
           \midrule
           Design method & Pole placement & DARE (optimal) \\
           Noise handling & None & Process + measurement \\
           Optimality & Suboptimal & Optimal (BLUE) \\
           \bottomrule
       \end{tabular}
       \caption{Comparison of observer and Kalman filter.}
       \label{tab:part5_comparison}
   \end{table}
   ```

7. **Findings Subsection**:
   - Kalman filter is optimal for noisy systems (minimizes error covariance)
   - Steady-state errors are 45% lower than full-window (filter converges)
   - Near-unity spectral radius (0.999547) indicates optimal balance between tracking and noise rejection
   - Successfully estimates all 12 states from 2 noisy measurements

After completing Part 5, STOP and wait for my next message with Part 6 materials.

---

## Attachments to include with this prompt:

Upload these files to ChatGPT:
1. `final/part5/part5_report.md` - Part 5 report
2. `final/part5/part5_explanation.md` - Detailed explanation
3. `final/part5/kalman_filter.py` - Python code
4. `final/part5/outputs/outputs_y_vs_yhat.png` - Outputs comparison plot
5. `final/part5/outputs/estimation_error_norm.png` - Error norm plot
6. `final/part5/outputs/per_state_rms_bar.png` - Per-state RMS bar chart

---

## Next Step:

After ChatGPT responds with Part 5 LaTeX, proceed to `07_part6_prompt.md`.
