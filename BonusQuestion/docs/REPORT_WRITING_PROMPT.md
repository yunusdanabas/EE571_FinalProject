# Part 6: Report Writing Prompt

This prompt guides you through writing the final PDF report for the Vehicle Tracking Project. The report should consolidate all work from Parts 0-5 into a clear, professional document suitable for submission.

**Before starting**: Verify all prerequisites are complete using `docs/PRE_REPORT_VERIFICATION.md`

---

## Your Task

Write a comprehensive PDF report that:
1. Explains the problem setup and approach
2. Documents both controller design methods (LQR and Pole Placement)
3. Presents results from all 6 experiments with comparison plots
4. Provides quantitative performance analysis using metrics
5. Draws clear conclusions about regulator robustness

**Target Length**: 8-12 pages (excluding title page, appendices if any)

---

## Report Structure

### 1. Title Page
- [ ] Project title: "Vehicle Trajectory Tracking with Discrete LQR and Pole Placement Control"
- [ ] Course: EE571 Final Exam Bonus
- [ ] Your name/student ID
- [ ] Date

### 2. Abstract/Executive Summary (Optional but Recommended)
- [ ] Brief overview (100-150 words)
- [ ] Problem statement
- [ ] Methods used (two controllers)
- [ ] Key findings
- [ ] Main conclusion

### 3. Introduction / Problem Statement

**Content to Include:**
- [ ] Problem description:
  - Vehicle trajectory tracking task
  - Nonlinear bicycle model plant
  - Linearized error model for controller design
  - Time-parameterized reference trajectory
- [ ] Objectives:
  - Design two discrete-time state-feedback regulators
  - Compare performance across different initial error magnitudes
  - Evaluate robustness
- [ ] Report structure overview (brief)

**Length**: ~1 page

**Reference**: `docs/01_project_explanation.md` for problem details

---

### 4. System Model and Error Formulation

**Content to Include:**

#### 4.1 Vehicle Model
- [ ] Nonlinear bicycle model description:
  - Plant state: `x = [X, Y, ψ, v_x, v_y, r]`
  - Inputs: `u = [δ, a_x]` (steering angle, longitudinal acceleration)
  - Key parameters: m, I_z, l_f, l_r, C_f, C_r
  - Reference parameter values
- [ ] Dynamics equations (brief):
  - Kinematics: `Ẋ = v_x cos(ψ) - v_y sin(ψ)`, etc.
  - Tire model: Linear cornering stiffness model
  - Dynamics: `v̇_x`, `v̇_y`, `ṙ` equations

#### 4.2 Reference Trajectory
- [ ] Reference signals:
  - Curvature: `κ_ref(t) = 0.01 sin(0.35t) + 0.005 sin(0.10t)`
  - Speed: `v_ref(t) = V_{x0} + 1.0 sin(0.15t)` with `V_{x0} = 15 m/s`
  - Acceleration: `a_ref(t) ≈ (v_ref(t+T_s) - v_ref(t))/T_s`
- [ ] Reference pose integration (brief)

#### 4.3 Tracking Errors
- [ ] Error state vector definition: `x_e = [v_y, r, e_y, e_ψ, e_v]`
  - `v_y`: Lateral velocity (from plant state)
  - `r`: Yaw rate (from plant state)
  - `e_y`: Cross-track error (normal to reference path)
  - `e_ψ`: Heading error (wrapped to [-π, π])
  - `e_v`: Speed error (`v_x - v_ref`)
- [ ] Error computation methods (brief)
- [ ] Reference frame and error geometry (brief, if helpful)

#### 4.4 Linearized Error Model
- [ ] Continuous-time error model:
  ```
  ẋ_e = A_c x_e + B_c u_reg
  ```
- [ ] Matrix structure (brief):
  - `A_c` (5×5): Error state dynamics
  - `B_c` (5×2): Input matrix
  - Nominal operating point: `v_x ≈ V_{x0}`, `v_y ≈ 0`, `r ≈ 0`, `δ ≈ 0`
- [ ] Mention that reference terms are handled by feedforward

**Length**: ~1.5-2 pages

**Reference**: `docs/part1_model_review/part1_closeout.md` for variable mapping
**Reference**: `doc/VehcileTracking_Document.md` for mathematical details

---

### 5. Controller Design

#### 5.1 Discretization

**Content to Include:**
- [ ] Discretization method: Zero-order hold (ZOH)
- [ ] Sampling time: `T_s = 0.02 s` (50 Hz)
- [ ] Discrete-time model:
  ```
  x_{e,k+1} = A_d x_{e,k} + B_d u_{reg,k}
  ```
- [ ] Discretization method: Exact ZOH using matrix exponential
- [ ] Brief mention of matrix dimensions: `A_d` (5×5), `B_d` (5×2)
- [ ] Note: Same discretization used for both controllers (for fair comparison)

**Length**: ~0.5 page

**Reference**: `docs/part2_discretization/part2_closeout.md`

---

#### 5.2 Regulator 1: Discrete-Time LQR

**Content to Include:**
- [ ] Control objective:
  - Infinite-horizon quadratic cost function
  - Minimize: `J = Σ_{k=0}^∞ (x_e^T Q x_e + u_reg^T R u_reg)`
- [ ] Weighting matrix design:
  - **Q matrix**: List your actual values (diagonal):
    - `Q = diag([q_vy, q_r, q_ey, q_epsi, q_ev])`
    - List your values: `Q = diag([5.0, 5.0, 50.0, 50.0, 30.0])` (or your actual values)
    - **Rationale**: Explain why you chose these values
      - Higher weights on tracking errors (e_y, e_ψ, e_v)
      - Lower weights on internal states (v_y, r)
  - **R matrix**: List your actual values (diagonal):
    - `R = diag([r_delta, r_ax])`
    - List your values: `R = diag([2.0, 1.0])` (or your actual values)
    - **Rationale**: Explain balance between control effort and tracking
- [ ] Gain computation:
  - Solve discrete algebraic Riccati equation (DARE): `P = solve_discrete_are(A_d, B_d, Q, R)`
  - Feedback gain: `K_LQR = (R + B_d^T P B_d)^{-1} B_d^T P A_d`
- [ ] Control law:
  ```
  u_reg = -K_LQR x_e
  u = u_ff + u_reg
  ```
- [ ] Closed-loop stability:
  - Eigenvalues of `A_d - B_d K_LQR`
  - All eigenvalues inside unit circle (verify this)
  - Optional: Show eigenvalue values

**Length**: ~1.5 pages

**Reference**: `docs/part3_regulator_lqr/part3_closeout.md` for actual Q, R values and K_LQR

---

#### 5.3 Regulator 2: Discrete-Time Pole Placement

**Content to Include:**
- [ ] Control objective:
  - Assign desired closed-loop poles
  - Constraint: **Real poles only** (no complex poles allowed)
- [ ] Pole selection:
  - List your actual chosen poles:
    - Example: `poles = [0.85, 0.80, 0.75, 0.70, 0.65]` (use your actual values)
  - **Rationale**: Explain pole selection
    - All poles inside unit circle (stable, discrete-time)
    - All poles are real (requirement)
    - Poles chosen for good tracking performance (fast but stable)
    - Typical range: 0.7 to 0.95
- [ ] Gain computation:
  - Use `scipy.signal.place_poles(A_d, B_d, desired_poles)`
  - Feedback gain: `K_PP = result.gain_matrix`
- [ ] Control law:
  ```
  u_reg = -K_PP x_e
  u = u_ff + u_reg
  ```
- [ ] Closed-loop verification:
  - Verify eigenvalues of `A_d - B_d K_PP` match desired poles
  - All eigenvalues are real (verify this)
  - All eigenvalues inside unit circle (stable)

**Length**: ~1-1.5 pages

**Reference**: `docs/part4_regulator_poleplacement/part4_closeout.md` for actual poles and K_PP

---

#### 5.4 Control Implementation

**Content to Include:**
- [ ] Feedforward terms:
  - Steering feedforward: `δ_ff = (l_f + l_r) κ_ref` (geometric steering)
  - Acceleration feedforward: `a_{x,ff} = a_ref` (reference acceleration)
- [ ] Total control input:
  ```
  u = u_ff + u_reg
  ```
- [ ] Input saturation:
  - Steering: `δ ∈ [-25°, +25°]`
  - Acceleration: `a_x ∈ [-6, +3] m/s²`
- [ ] Simulation setup:
  - Sampling time: `T_s = 0.02 s`
  - Simulation duration: `T = 25 s`
  - Plant integration: RK4 with internal step `dt_int = T_s/10`

**Length**: ~0.5 page

---

### 6. Results and Performance Comparison

**This is the main results section. Include comprehensive analysis.**

#### 6.1 Experimental Setup

**Content to Include:**
- [ ] Experiment matrix: 2 regulators × 3 initial error scales = 6 runs
- [ ] Initial condition scaling:
  - Baseline offsets: `ΔX = -2.0 m`, `ΔY = +1.0 m`, `Δψ = +8°`, `Δv_x = -5 m/s`
  - For scale `s ∈ {1, 2, 3}`: Multiply offsets by `s`
  - Initial conditions: `x(0) = x_ref(0) + s·Δx_0`
- [ ] Comparison methodology:
  - Same discretization (fair comparison)
  - Same feedforward terms
  - Same saturation limits
  - Same simulation parameters

**Length**: ~0.5 page

---

#### 6.2 Performance Metrics

**Content to Include:**
- [ ] Metrics computed:
  - RMS errors: `RMS(e_y)`, `RMS(e_ψ)`, `RMS(e_v)`
  - Max absolute errors: `max|e_y|`, `max|e_ψ|`, `max|e_v|`
  - Control effort: RMS steering angle, RMS acceleration
  - Saturation: Count/percentage of time saturated
- [ ] Create a metrics table (Table 1) with all 6 runs:
  - Columns: Regulator, Scale, RMS e_y, Max |e_y|, RMS e_ψ, Max |e_ψ|, RMS e_v, Max |e_v|
  - Rows: All 6 cases (LQR scale 1-3, PP scale 1-3)
  - **Reference**: Use actual values from `results/logs/metrics_summary.md`
- [ ] Optional: Additional metrics table with control effort

**Length**: ~0.5-1 page

**Reference**: `results/logs/metrics_summary.md` for actual metrics

---

#### 6.3 Trajectory Comparison

**Content to Include:**
- [ ] Include trajectory plots for all 3 scales:
  - Figure 1: Trajectory comparison, scale 1
  - Figure 2: Trajectory comparison, scale 2
  - Figure 3: Trajectory comparison, scale 3
- [ ] Each figure should show:
  - Reference trajectory (black or blue solid line)
  - LQR result (blue line, distinct style)
  - Pole Placement result (red dashed line, distinct style)
  - Clear legend
  - Proper axes labels (X [m], Y [m])
  - Equal aspect ratio
- [ ] Discussion for each scale:
  - Scale 1: Both regulators track well? Similar performance?
  - Scale 2: Performance degrades? Which regulator handles it better?
  - Scale 3: Larger initial errors - significant performance differences?
  - Visual observations: Which path is closer to reference?

**Length**: ~1 page (with figures)

**Reference**: `results/plots/trajectory_scale1.png`, `trajectory_scale2.png`, `trajectory_scale3.png`

---

#### 6.4 Error Time Histories

**Content to Include:**
- [ ] Include error comparison plots for all 3 scales:
  - Figure 4: Errors comparison, scale 1
  - Figure 5: Errors comparison, scale 2
  - Figure 6: Errors comparison, scale 3
- [ ] Each figure should show 3 subplots:
  - Cross-track error `e_y(t)`
  - Heading error `e_ψ(t)`
  - Speed error `e_v(t)`
  - Both regulators overlaid with clear legend
- [ ] Discussion:
  - Convergence behavior: Do errors converge to zero?
  - Convergence speed: Which regulator converges faster?
  - Steady-state error: Any persistent errors?
  - Overshoot/undershoot: Which regulator has better transient response?
  - Scale dependence: How do errors scale with initial error magnitude?

**Length**: ~1-1.5 pages (with figures)

**Reference**: `results/plots/errors_scale1.png`, `errors_scale2.png`, `errors_scale3.png`

---

#### 6.5 Control Inputs

**Content to Include:**
- [ ] Include input comparison plots for all 3 scales:
  - Figure 7: Inputs comparison, scale 1
  - Figure 8: Inputs comparison, scale 2
  - Figure 9: Inputs comparison, scale 3
- [ ] Each figure should show 2 subplots:
  - Steering angle `δ(t)` with saturation limits (±25°)
  - Acceleration `a_x(t)` with saturation limits (-6, +3 m/s²)
  - Both regulators overlaid
- [ ] Discussion:
  - Control effort: Which regulator uses more control effort?
  - Saturation behavior: Which regulator saturates more often?
  - Saturation frequency: How often do inputs hit limits? (use metrics)
  - Control smoothness: Which regulator has smoother inputs?
  - Scale dependence: How does control effort scale?

**Length**: ~0.5-1 page (with figures)

**Reference**: `results/plots/inputs_scale1.png`, `inputs_scale2.png`, `inputs_scale3.png`

---

#### 6.6 Quantitative Performance Analysis

**Content to Include:**
- [ ] Analyze metrics table:
  - Compare RMS errors across regulators (for each scale)
  - Compare max errors across regulators
  - Identify which regulator performs better at each scale
- [ ] Performance trends:
  - How does performance degrade with scale (1→2→3)?
  - Which regulator degrades more gracefully?
  - Which regulator is more robust to larger initial errors?
- [ ] Trade-offs:
  - Control effort vs. tracking accuracy
  - Saturation vs. performance
  - Transient vs. steady-state performance

**Length**: ~1 page

**Reference**: Metrics from `results/logs/metrics_summary.md`

---

### 7. Discussion and Conclusions

**Content to Include:**

#### 7.1 Summary of Findings
- [ ] Key observations from experiments:
  - Scale 1: Both regulators perform well
  - Scale 2: Performance differences emerge
  - Scale 3: Clear performance differences
- [ ] Which regulator performs better at each scale? (Be specific)

#### 7.2 Robustness Analysis
- [ ] **Main Question**: Which regulator is more robust to larger initial errors?
- [ ] Answer based on results:
  - Compare performance degradation (scale 1 → 2 → 3)
  - Compare error magnitudes at each scale
  - Compare control effort and saturation
  - **Conclude**: Which regulator handles larger initial errors better?
- [ ] Reasoning:
  - Why is one regulator more robust?
  - Design differences (LQR vs. pole placement)
  - Q/R tuning vs. pole placement strategy

#### 7.3 Design Insights
- [ ] LQR advantages/disadvantages:
  - Automatic optimality (given Q/R)
  - Tuning via Q/R matrices
- [ ] Pole Placement advantages/disadvantages:
  - Direct pole assignment
  - Real poles constraint
- [ ] Practical considerations:
  - Ease of tuning
  - Performance vs. effort trade-offs

#### 7.4 Limitations and Future Work
- [ ] Brief discussion of limitations:
  - Linearized model assumption
  - Saturation effects
  - Parameter uncertainty (optional)
- [ ] Potential improvements (optional):
  - Different Q/R tuning
  - Different pole locations
  - Adaptive control (optional)

**Length**: ~1-1.5 pages

---

### 8. References (if any)

- [ ] List any references used (if applicable)
- [ ] Project specification document
- [ ] Technical references (optional)

---

## Report Formatting Guidelines

### Figures
- [ ] **All 9 comparison plots must be included**:
  - 3 trajectory plots (scale 1, 2, 3)
  - 3 error plots (scale 1, 2, 3)
  - 3 input plots (scale 1, 2, 3)
- [ ] Figure quality:
  - High resolution (300 DPI minimum for print)
  - Clear labels (axes, legend, title)
  - Readable font sizes
  - Consistent style across figures
- [ ] Figure captions:
  - Descriptive captions
  - Mention what is shown (e.g., "Trajectory comparison for scale 1x initial errors")
  - Reference figure numbers (Figure 1, Figure 2, etc.)
- [ ] Figure placement:
  - Near relevant text discussion
  - Large enough to see details
  - Consistent sizing

### Tables
- [ ] Metrics table:
  - Professional formatting
  - Clear column headers
  - Appropriate number of decimal places
  - Consistent units
- [ ] Table caption:
  - Descriptive (e.g., "Performance metrics for all 6 experimental cases")
  - Reference table number (Table 1, Table 2, etc.)

### Equations
- [ ] Use proper mathematical notation
- [ ] Number important equations (optional)
- [ ] Consistent formatting
- [ ] Clear variable definitions

### Code/Implementation Details
- [ ] Keep code snippets brief (if included)
- [ ] Focus on algorithms, not implementation details
- [ ] Key functions can be mentioned but don't paste full code

---

## File Organization for Report

### Option 1: LaTeX (Recommended for Professional Reports)
- [ ] Create `report/` directory
- [ ] Main file: `report/main.tex`
- [ ] Include plots: `report/figures/` (copy from `results/plots/`)
- [ ] Generate PDF: `pdflatex main.tex`

### Option 2: Markdown with Pandoc
- [ ] Create `report/main.md`
- [ ] Use markdown with figure references
- [ ] Convert to PDF: `pandoc main.md -o report.pdf --pdf-engine=xelatex`
- [ ] Include plot paths

### Option 3: Jupyter Notebook
- [ ] Create `report/report.ipynb`
- [ ] Combine text, plots, and analysis
- [ ] Export to PDF: `jupyter nbconvert --to pdf report.ipynb`

### Option 4: Word/Google Docs
- [ ] Create document
- [ ] Insert plots as images
- [ ] Export to PDF

**Recommended**: LaTeX for best formatting, but use what you're comfortable with.

---

## Final Checklist Before Submission

### Content Completeness
- [ ] Title page included
- [ ] All 6 sections present (Introduction through Conclusions)
- [ ] All 9 comparison plots included
- [ ] Metrics table included
- [ ] All key results discussed
- [ ] Conclusions clearly stated

### Technical Accuracy
- [ ] All equations are correct
- [ ] Parameter values match actual implementation
- [ ] Q/R values match Part 3 closeout
- [ ] Pole values match Part 4 closeout
- [ ] Metrics values match `results/logs/metrics_summary.md`
- [ ] All claims supported by data/plots

### Clarity and Writing
- [ ] Clear, professional writing
- [ ] Proper grammar and spelling
- [ ] Logical flow between sections
- [ ] Figures clearly referenced in text
- [ ] Tables clearly referenced in text
- [ ] Conclusions directly answer the robustness question

### Formatting
- [ ] Consistent formatting throughout
- [ ] All figures are readable
- [ ] Tables are well-formatted
- [ ] Page numbers (if applicable)
- [ ] Professional appearance

### File Verification
- [ ] PDF file generated successfully
- [ ] All figures appear correctly in PDF
- [ ] PDF opens and displays correctly
- [ ] File size reasonable
- [ ] Saved to appropriate location (e.g., `results/report_final.pdf`)

---

## Specific Content Requirements

### Must Answer These Questions:

1. **What is the problem?**
   - Vehicle trajectory tracking with linearized error model control

2. **What are the two controllers?**
   - Discrete LQR (infinite-horizon)
   - Discrete Pole Placement (real poles only)

3. **How were they designed?**
   - LQR: Q/R matrices (list your values), DARE solution
   - PP: Pole locations (list your values), pole placement algorithm

4. **What experiments were run?**
   - 6 runs: 2 regulators × 3 scales

5. **What are the results?**
   - Quantitative: Metrics table
   - Qualitative: 9 comparison plots
   - Performance at each scale

6. **Which regulator is more robust?**
   - **MUST ANSWER THIS CLEARLY**
   - Based on performance at larger scales (2x, 3x)
   - Supported by metrics and plots

---

## Example Report Outline (Quick Reference)

```
1. Title Page
2. Abstract (Optional)
3. Introduction (~1 page)
   - Problem description
   - Objectives
4. System Model and Error Formulation (~1.5-2 pages)
   - Vehicle model
   - Reference trajectory
   - Error definitions
   - Linearized error model
5. Controller Design (~3 pages)
   - Discretization (0.5 page)
   - LQR design (1.5 pages)
   - Pole placement design (1-1.5 pages)
   - Implementation (0.5 page)
6. Results and Performance Comparison (~4-5 pages)
   - Experimental setup (0.5 page)
   - Metrics table (0.5-1 page)
   - Trajectory comparison (1 page)
   - Error comparison (1-1.5 pages)
   - Input comparison (0.5-1 page)
   - Quantitative analysis (1 page)
7. Discussion and Conclusions (~1-1.5 pages)
   - Summary
   - Robustness analysis
   - Design insights
8. References (if any)
```

**Total**: 8-12 pages

---

## Key Files to Reference

1. **Project Overview**: `docs/01_project_explanation.md`
2. **Part 1 Closeout**: `docs/part1_model_review/part1_closeout.md` (variable mapping)
3. **Part 2 Closeout**: `docs/part2_discretization/part2_closeout.md` (discretization)
4. **Part 3 Closeout**: `docs/part3_regulator_lqr/part3_closeout.md` (Q, R, K_LQR values)
5. **Part 4 Closeout**: `docs/part4_regulator_poleplacement/part4_closeout.md` (poles, K_PP values)
6. **Part 5 Closeout**: `docs/part5_experiments_comparison/part5_closeout.md` (experiment results)
7. **Metrics**: `results/logs/metrics_summary.md`
8. **Plots**: `results/plots/` (all 9 comparison plots)
9. **Specification**: `doc/VehcileTracking_Document.md` (mathematical details)

---

## Writing Tips

1. **Be Specific**: Use actual values from your implementation, not placeholders
2. **Be Quantitative**: Reference specific metrics and numbers
3. **Be Visual**: Figures tell the story - discuss what they show
4. **Be Clear**: Answer the robustness question directly
5. **Be Professional**: Use technical language appropriately
6. **Be Complete**: Cover all required sections
7. **Be Concise**: Don't be overly verbose, but don't skip important details

---

## Final Output

**Deliverable**: 
- PDF report file: `results/report_final.pdf` (or similar location)
- Complete, professional report
- All plots included and clearly labeled
- All metrics discussed
- Clear conclusions about regulator robustness

**After completion**: Fill out `docs/part6_report_packaging/part6_closeout.md` with:
- Report location
- Summary of sections
- Any challenges encountered
- Final verification checklist

---

**Good luck with your report writing!**
