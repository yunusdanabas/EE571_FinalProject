# Part 3 Prompt: LQR Controller Design

## Copy and paste this prompt to ChatGPT:

---

Now I'm sending you the materials for **Part 3: LQR Controller Design with Observer**.

**Objective:** Design a discrete-time LQR controller that uses estimated states from the Part 2 observer and minimizes the cost J = sum(u'u + y1^2 + y6^2).

Please write the LaTeX section for Part 3 that includes:

1. **Section Header**: `\section{Part 3: LQR Controller Design}`

2. **Objective Subsection**: Explain LQR optimal control concept

3. **Theory Subsection**:
   - Cost function: $J = \sum_{k=0}^{N-1} \left( u[k]^T u[k] + y_1[k]^2 + y_6[k]^2 \right)$
   - State weight: $Q = C_y^T C_y$ (penalizes x1 and x6)
   - Input weight: $R = I_3$ (identity matrix)
   - DARE (Discrete-time Algebraic Riccati Equation)
   - LQR gain: $K = (R + B_d^T P B_d)^{-1} B_d^T P A_d$

4. **Separation Principle Subsection**:
   - Explain that observer and controller can be designed independently
   - Control law uses estimated states: $u[k] = -K \hat{x}[k]$

5. **Results Subsection**:
   - LQR gain K shape: (3, 12)
   - Closed-loop spectral radius: 0.999463
   - Total cost J: 9.057 x 10^7
   - Maximum input magnitude: 3.597 x 10^3
   - Include figures: outputs y1/y6, inputs u1/u2/u3, estimation error norm

6. **Closed-Loop Dynamics**:
   ```latex
   \begin{align}
   x[k+1] &= A_d x[k] + B_d u[k] \\
   \hat{x}[k+1] &= A_d \hat{x}[k] + B_d u[k] + L(y[k] - C\hat{x}[k]) \\
   u[k] &= -K \hat{x}[k]
   \end{align}
   ```

7. **Findings Subsection**:
   - Controller stabilizes the system (spectral radius < 1.0)
   - Outputs y1 and y6 are regulated toward zero
   - Spectral radius close to 1.0 indicates slow convergence
   - Separation principle demonstrated successfully

After completing Part 3, STOP and wait for my next message with Part 4 materials.

---

## Attachments to include with this prompt:

Upload these files to ChatGPT:
1. `final/part3/part3_report.md` - Part 3 report
2. `final/part3/part3_explanation.md` - Detailed explanation
3. `final/part3/lqr_controller.py` - Python code
4. `final/part3/outputs/outputs_y1_y6.png` - Outputs plot
5. `final/part3/outputs/inputs_u1_u2_u3.png` - Inputs plot
6. `final/part3/outputs/estimation_error_norm.png` - Error norm plot

---

## Next Step:

After ChatGPT responds with Part 3 LaTeX, proceed to `05_part4_prompt.md`.
