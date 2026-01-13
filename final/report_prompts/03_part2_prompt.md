# Part 2 Prompt: Observer Design

## Copy and paste this prompt to ChatGPT:

---

Now I'm sending you the materials for **Part 2: Observer Design**.

**Objective:** Design a Luenberger observer for the 6-mass spring system using an augmented sensor matrix that measures x1 and x6 (2 sensors). Verify that the observer successfully estimates all 12 states.

Please write the LaTeX section for Part 2 that includes:

1. **Section Header**: `\section{Part 2: Observer Design}`

2. **Objective Subsection**: Explain the purpose of state observers

3. **Sensor Configuration Subsection**:
   - New measurement matrix C (2x12) measuring x1 and x6
   - Write the C matrix in LaTeX:
   ```latex
   C = \begin{bmatrix} 1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\ 
                       0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 \end{bmatrix}
   ```

4. **Observer Design Subsection**:
   - Observer dynamics: $\hat{x}[k+1] = A_d \hat{x}[k] + B_d u[k] + L(y[k] - \hat{y}[k])$
   - Error dynamics: $e[k+1] = (A_d - LC_d)e[k]$
   - Pole placement via dual system approach
   - Poles placed in [0.4, 0.8] range

5. **Initial Conditions**:
   - True state: x0 = [0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0]
   - Observer initial: xhat0 = [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]

6. **Results Subsection**:
   - Observer gain L shape: (12, 2)
   - Spectral radius: 0.8
   - Include RMS error table for states x1-x6
   - Include figures: outputs comparison, estimation errors

7. **Warnings Subsection**: Explain the pole placement convergence warning (harmless)

8. **Findings Subsection**:
   - Observer is stable (spectral radius = 0.8)
   - Measured states have very small errors (~10^-6)
   - Unmeasured states have larger but bounded errors
   - Observer successfully estimates all 12 states from 2 measurements

After completing Part 2, STOP and wait for my next message with Part 3 materials.

---

## Attachments to include with this prompt:

Upload these files to ChatGPT:
1. `final/part2/part2_report.md` - Part 2 report
2. `final/part2/part2_explanation.md` - Detailed explanation
3. `final/part2/observer_design.py` - Python code
4. `final/part2/outputs/outputs_comparison.png` - Outputs plot
5. `final/part2/outputs/estimation_errors.png` - Errors plot
6. `final/part2/outputs/estimation_errors_05sec.png` - Zoomed errors plot

---

## Next Step:

After ChatGPT responds with Part 2 LaTeX, proceed to `04_part3_prompt.md`.
