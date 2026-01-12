# Part 4 Prompt: Reduced Input LQR

## Copy and paste this prompt to ChatGPT:

---

Now I'm sending you the materials for **Part 4: LQR Controller with Reduced Input**.

**Objective:** Redesign the LQR controller with only two inputs (u1, u2), removing u3, and compare performance with Part 3.

Please write the LaTeX section for Part 4 that includes:

1. **Section Header**: `\section{Part 4: Reduced Input LQR Controller}`

2. **Objective Subsection**: Explain the purpose of analyzing reduced actuator authority

3. **Input Matrix Reduction Subsection**:
   - Original: $B_d$ has 3 columns (u1, u2, u3)
   - Reduced: $B_{d,red} = B_d[:, [0,1]]$ (only u1, u2)
   - Input weight: $R_{red} = I_2$ (2x2 identity)

4. **LQR Redesign Subsection**:
   - Same state weight Q (penalizes x1 and x6)
   - Solve DARE with reduced input matrix
   - Reduced gain: $K_{red}$ (2x12)

5. **Results Subsection** with comparison table:
   ```latex
   \begin{table}[H]
       \centering
       \begin{tabular}{lcc}
           \toprule
           Metric & Part 3 (3 inputs) & Part 4 (2 inputs) \\
           \midrule
           LQR gain shape & (3, 12) & (2, 12) \\
           Spectral radius & 0.999463 & 0.999518 \\
           Total cost J & $9.057 \times 10^7$ & $1.348 \times 10^8$ \\
           Max $|u|$ & $3.597 \times 10^3$ & $4.951 \times 10^3$ \\
           Cost increase & -- & 48.84\% \\
           \bottomrule
       \end{tabular}
       \caption{Comparison of full input vs reduced input LQR.}
       \label{tab:part4_comparison}
   \end{table}
   ```

6. **Include figures**: outputs y1/y6, inputs u1/u2, estimation error norm

7. **Findings Subsection**:
   - Removing u3 increases cost by 48.84%
   - Maximum input magnitude increases (remaining inputs work harder)
   - System is still stabilizable with 2 inputs
   - Demonstrates trade-off between actuator availability and performance

After completing Part 4, STOP and wait for my next message with Part 5 materials.

---

## Attachments to include with this prompt:

Upload these files to ChatGPT:
1. `final/part4/part4_report.md` - Part 4 report
2. `final/part4/part4_explanation.md` - Detailed explanation
3. `final/part4/lqr_reduced.py` - Python code
4. `final/part4/outputs/outputs_y1_y6.png` - Outputs plot
5. `final/part4/outputs/inputs_u1_u2.png` - Inputs plot
6. `final/part4/outputs/estimation_error_norm.png` - Error norm plot

---

## Next Step:

After ChatGPT responds with Part 4 LaTeX, proceed to `06_part5_prompt.md`.
