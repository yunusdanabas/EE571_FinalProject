# Part 6 Prompt: LQG Controller

## Copy and paste this prompt to ChatGPT:

---

Now I'm sending you the materials for **Part 6: LQG Controller**.

**Objective:** Combine the LQR controller (Part 3) with the Kalman filter (Part 5) to implement an LQG (Linear Quadratic Gaussian) controller for the noisy system.

Please write the LaTeX section for Part 6 that includes:

1. **Section Header**: `\section{Part 6: LQG Controller}`

2. **Objective Subsection**: Explain LQG as the combination of optimal control (LQR) and optimal estimation (Kalman filter)

3. **LQG Structure Subsection**:
   - Controller gain K from Part 3 (unchanged)
   - Kalman filter gain Lk from Part 5 (unchanged)
   - Separation principle: design K and Lk independently, combine for optimal LQG

4. **LQG Dynamics** equations:
   ```latex
   \begin{align}
   x[k+1] &= A_d x[k] + B_d u[k] + B_d w[k] \quad \text{(true system with noise)} \\
   y_{meas}[k] &= C x[k] + v[k] \quad \text{(noisy measurement)} \\
   \hat{x}[k+1] &= A_d \hat{x}[k] + B_d u[k] + L_k (y_{meas}[k] - C\hat{x}[k]) \\
   u[k] &= -K \hat{x}[k]
   \end{align}
   ```

5. **Cost Function Subsection**:
   - Uses $y_{true}$ (not $y_{meas}$) to avoid penalizing uncontrollable measurement noise
   - $J = \sum (u^T u + y_1^2 + y_6^2)$ where $y = Cx$ (true output)

6. **Results Subsection** with comparison table:
   ```latex
   \begin{table}[H]
       \centering
       \begin{tabular}{lcc}
           \toprule
           Metric & Part 3 (No Noise) & Part 6 (LQG with Noise) \\
           \midrule
           Total cost J & $9.057 \times 10^7$ & $4.261 \times 10^2$ \\
           Max $|u|$ & $3.597 \times 10^3$ & $4.086 \times 10^{-1}$ \\
           Estimator & Pole placement (L) & Kalman filter ($L_k$) \\
           \bottomrule
       \end{tabular}
       \caption{Comparison of Part 3 and Part 6.}
       \label{tab:part6_comparison}
   \end{table}
   ```

7. **Include figures**: outputs comparison (Part 3 vs Part 6), inputs, estimation error norm

8. **Findings Subsection**:
   - LQG demonstrates separation principle in practice
   - Much smaller control inputs compared to Part 3 (4 orders of magnitude)
   - Successfully stabilizes system despite noise
   - RMS estimation error: 0.957 (full), 0.559 (steady-state)

After completing Part 6, STOP and wait for my next message with Part 7 materials.

---

## Attachments to include with this prompt:

Upload these files to ChatGPT:
1. `final/part6/part6_report.md` - Part 6 report
2. `final/part6/part6_explanation.md` - Detailed explanation
3. `final/part6/lqg_controller.py` - Python code
4. `final/part6/outputs/outputs_comparison.png` - Outputs comparison plot
5. `final/part6/outputs/inputs_u1_u2_u3.png` - Inputs plot
6. `final/part6/outputs/estimation_error_norm.png` - Error norm plot

---

## Next Step:

After ChatGPT responds with Part 6 LaTeX, proceed to `08_part7_prompt.md`.
