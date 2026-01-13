# Part 0 Prompt: Baseline Verification

## Copy and paste this prompt to ChatGPT:

---

Now I'm sending you the materials for **Part 0: Baseline Verification**.

**Objective:** Verify the system model by discretizing the continuous-time matrices and running a baseline open-loop simulation to establish the baseline behavior.

Please write the LaTeX section for Part 0 that includes:

1. **Section Header**: `\section{Part 0: Baseline Verification}`

2. **Objective Subsection**: Explain what baseline verification accomplishes

3. **System Model Subsection**: Include the state-space equations:
   - Continuous-time: $\dot{x} = Ax + Bu$, $y = Cx$
   - Discrete-time (ZOH): $x[k+1] = A_d x[k] + B_d u[k]$
   - Matrix dimensions: A (12x12), B (12x3), C (1x12)
   - Sampling time Ts = 0.01s

4. **Methodology Subsection**: Explain the discretization process (Zero-Order Hold)

5. **Initial Conditions**: x0 = [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0] (mass 6 displaced by 1 unit)

6. **Results Subsection**: Include the two figures with proper captions:
   - Figure 1: System output y = Cx (displacement of mass 1)
   - Figure 2: All 6 mass displacements (x1 through x6)
   
   Use this LaTeX format for figures:
   ```latex
   \begin{figure}[H]
       \centering
       \includegraphics[width=0.8\textwidth]{figures/part0_output_plot.png}
       \caption{Baseline system output showing displacement of mass 1.}
       \label{fig:part0_output}
   \end{figure}
   ```

7. **Findings Subsection**: Describe:
   - Oscillatory behavior (undamped system)
   - Marginal stability
   - Energy propagation through the spring chain

After completing Part 0, STOP and wait for my next message with Part 1 materials.

---

## Attachments to include with this prompt:

Upload these files to ChatGPT:
1. `final/part0/part0_report.md` - Part 0 report
2. `final/part0/part0_explanation.md` - Detailed explanation
3. `final/part0/baseline_check.py` - Python code
4. `final/part0/outputs/output_plot.png` - Output plot
5. `final/part0/outputs/displacements_plot.png` - Displacements plot

---

## Note for LaTeX compilation:

Tell ChatGPT to use these figure paths (you'll need to copy images to a `figures/` folder):
- `figures/part0_output_plot.png`
- `figures/part0_displacements_plot.png`

---

## Next Step:

After ChatGPT responds with Part 0 LaTeX, proceed to `02_part1_prompt.md`.
