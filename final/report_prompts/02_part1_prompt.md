# Part 1 Prompt: Observability Analysis

## Copy and paste this prompt to ChatGPT:

---

Now I'm sending you the materials for **Part 1: Observability Analysis**.

**Objective:** Analyze the observability of the discrete-time system with a single sensor measuring x1 (displacement of mass 1). Determine which states are observable and which are unobservable.

Please write the LaTeX section for Part 1 that includes:

1. **Section Header**: `\section{Part 1: Observability Analysis}`

2. **Objective Subsection**: Explain observability and why it matters for state estimation

3. **Theory Subsection**: Include mathematical formulation:
   - Observability matrix: $\mathcal{O} = [C; CA; CA^2; \ldots; CA^{n-1}]$
   - Full observability condition: rank($\mathcal{O}$) = n
   - Kalman decomposition concept

4. **Methodology Subsection**: 
   - SVD-based rank computation
   - Kalman decomposition to separate observable/unobservable subspaces

5. **Results Subsection**: Include a table with key results:
   ```latex
   \begin{table}[H]
       \centering
       \begin{tabular}{ll}
           \toprule
           Metric & Value \\
           \midrule
           System dimension (n) & 12 \\
           Observability rank & 6 \\
           Observable states & 6 \\
           Unobservable states & 6 \\
           Transformation condition number & 1.000010 \\
           \bottomrule
       \end{tabular}
       \caption{Observability analysis results.}
       \label{tab:part1_results}
   \end{table}
   ```

6. **Eigenvalue Analysis Subsection**:
   - Explain why all eigenvalues are near |z| = 1.0 (undamped system)
   - List observable mode frequencies: 0.7653, 1.4139, 1.8475 rad/s
   - List unobservable mode frequencies: 0.4451, 1.2469, 1.8017 rad/s

7. **Findings Subsection**:
   - System is NOT fully observable with single sensor
   - Only symmetric modes (where mass 1 participates) are observable
   - Antisymmetric modes are unobservable
   - This motivates adding a second sensor in Part 2

After completing Part 1, STOP and wait for my next message with Part 2 materials.

---

## Attachments to include with this prompt:

Upload these files to ChatGPT:
1. `final/part1/part1_report.md` - Part 1 report
2. `final/part1/part1_explanation.md` - Detailed explanation
3. `final/part1/observability.py` - Python code
4. `final/part1/outputs/observability_results.txt` - Numerical results

---

## Next Step:

After ChatGPT responds with Part 1 LaTeX, proceed to `03_part2_prompt.md`.
