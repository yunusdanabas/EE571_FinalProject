# Final Exam Extract - EE571

Extracted requirements from the final exam for Parts 0 to 7, plus the provided `prep_final.m` snippet and known issues.

## 1) Provenance and Traceability Status

**Sources used in this extract**
- Final exam screenshots provided in chat (3 images). Page numbers are **unknown** from screenshots alone.
- `prep_final.m` contents provided in chat (full snippet below). This is treated as authoritative for the continuous-time matrices and baseline discretization workflow, unless the exam explicitly overrides a value.

**Traceability rule**
- If a requirement appears in the exam screenshots, it is **verified from screenshots**.
- If a requirement is only in `prep_final.m`, it is **verified from prep_final.m**.
- If a PDF page number is needed, it is **unknown** until someone opens `docs/sources/final_exam.pdf` and records page numbers.

## 2) System Definition from Exam (Verified from Screenshots)

**System**
- 6 masses connected by linear springs.
- 3 inputs.
  - Two inputs relate to tension on the first and last springs.
  - One input is a force applied on the second body.
- Controller is implemented on the discrete model with **T = 0.01 sec**.

**State order convention used in the project**
- \(x = [x_1, x_2, x_3, x_4, x_5, x_6, \dot x_1, \dot x_2, \dot x_3, \dot x_4, \dot x_5, \dot x_6]^T\).

**Exam statement about prep code**
- The exam states that `prep_final.m` provides one realization and assumes **only a single output is measured**, the displacement of the first body. This aligns with baseline measurement \(y = x_1\).

## 3) Part 0 and Part 1 Requirements (Verified from Screenshots)

### Part 0 Baseline Measurement
Baseline output is the displacement of the first body only.

\[
C_{\text{base}} = \begin{bmatrix} 1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \end{bmatrix}
\]

Traceability: verified from exam screenshots. PDF page number: *[To be recorded from final_exam.pdf]*.

### Part 1 Task
- The discrete system is **not observable** under baseline measurement.
- Obtain the **Kalman decomposition** and identify observable and unobservable modes.

## 4) Part 2 Requirements (Verified from Screenshots)

The exam states that adding another sensor at the 6th body fixes observability. It explicitly provides the measurement matrix and initial conditions.

### Part 2 Measurement Matrix (x1 and x6)
\[
C_{\text{part2}} = \begin{bmatrix}
1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0
\end{bmatrix}
\]

This measures \(x_1\) and \(x_6\) under the project state order.

### Part 2 Initial Conditions
Actual system initial state:
\[
x_0 = \begin{bmatrix} 0 & 0 & 0 & 1 & 1 & 1 & 0 & 0 & 0 & 0 & 0 & 0 \end{bmatrix}^T
\]

Estimated system initial state:
\[
\hat{x}_0 = \begin{bmatrix} 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 \end{bmatrix}^T
\]

### Part 2 Task
- Design an observer for the discrete-time system with \(T=0.01\) sec.
- Simulate and compare estimated states against actual states.
- You may augment the simulation snippet in `prep_final.m`.

Traceability: verified from exam screenshots. PDF page number: *[To be recorded from final_exam.pdf]*. Question: 2.

## 5) Part 3 Requirement (Verified from Screenshots)

Design an LQR for the observable system from Part 2 and simulate. The regulator uses estimated states.

Nominal cost:
\[
J = \sum \left(u^T u + y_1^2 + y_6^2\right), \quad u = [u_1\ u_2\ u_3]^T
\]

Implementation note for this project:
- Under \(C_{\text{part2}}\), the measured outputs are \(y = [x_1, x_6]^T\).
- The cost term \(y_1^2 + y_6^2\) corresponds to penalizing the measured displacement outputs \(x_1\) and \(x_6\).

Traceability: verified from exam screenshots. PDF page number: *[To be recorded from final_exam.pdf]*. Question: 3.

## 6) Part 4 Requirement (Verified from Screenshots)

- Suppose the third input is removed.
- Design a new LQR for the Part 3 setup with the same nominal cost.
- Compare against Part 3 in terms of total cost incurred and maximum input required.

Traceability: verified from exam screenshots. PDF page number: *[To be recorded from final_exam.pdf]*. Question: 4.

## 7) Part 5 Requirement (Verified from Screenshots)

Design a Kalman filter with actuator and sensor uncertainty.

Stochastic discrete model:
\[
x_{k+1} = A_d x_k + B_d u_k + B_d w_k
\]
\[
y_k = C_d x_k + v_k
\]

Noise distributions:
- \(v \sim \mathcal{N}(0, 0.1 I_p)\)
- \(w \sim \mathcal{N}(0, 0.05 I_m)\)
- \(p\) is number of outputs, \(m\) is number of inputs.

Implementation note:
- \(w_k\) has dimension \(m\) and enters through \(B_d\) per the exam statement.

Traceability: verified from exam screenshots. PDF page number: *[To be recorded from final_exam.pdf]*. Question: 5.

## 8) Part 6 Requirement (Verified from Screenshots)

- Use the LQR from Part 3 on the uncertain system from Part 5.
- This combines LQR with the estimated states from the Kalman filter.
- Simulate and compare outputs against those obtained in Part 3.

Traceability: verified from exam screenshots. PDF page number: *[To be recorded from final_exam.pdf]*. Question: 6.

## 9) Part 7 Requirement (Verified from Screenshots)

Closed-loop system from Part 6, add more sensors. Additional sensors have the same uncertainty level as others. Determine whether more sensors help estimation and or regulation.

### Part 7 Case 1 Measurement Matrix
\[
C_{\text{case1}} = \begin{bmatrix}
1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0
\end{bmatrix}
\]
This measures \(x_1, x_2, x_5, x_6\).

### Part 7 Case 2 Measurement Matrix
\[
C_{\text{case2}} = \begin{bmatrix}
1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0
\end{bmatrix}
\]
This measures all displacements \(x_1..x_6\).

Traceability: verified from exam screenshots. PDF page number: *[To be recorded from final_exam.pdf]*. Question: 7.

## 10) `prep_final.m` (Provided Snippet)

The following is included verbatim as provided. It defines the continuous-time matrices and a baseline discretization and simulation workflow.

```matlab
% MATLAB Script for Defining a Discrete Linear System

% Continuous-time system matrices
A = [0  0  0  0  0  0  1  0  0  0  0  0;
     0  0  0  0  0  0  0  1  0  0  0  0;
     0  0  0  0  0  0  0  0  1  0  0  0;
     0  0  0  0  0  0  0  0  0  1  0  0;
     0  0  0  0  0  0  0  0  0  0  1  0;
     0  0  0  0  0  0  0  0  0  0  0  1;
    -2  1  0  0  0  0  0  0  0  0  0  0;
     1 -2  1  0  0  0  0  0  0  0  0  0;
     0  1 -2  1  0  0  0  0  0  0  0  0;
     0  0  1 -2  1  0  0  0  0  0  0  0;
     0  0  0  1 -2  1  0  0  0  0  0  0;
     0  0  0  0  1 -1  0  0  0  0  0  0];

B = [0  0  0;
     0  0  0;
     0  0  0;
     0  0  0;
     0  0  0;
     0  0  0;
     1  0  0;
    -1  0 -1;
     0  0  0;
     0  0  0;
     0  1  0;
     0 -1  0];

% Output matrix (example: observe all displacements)

C = [1 0 0 0 0 0 0 0 0 0 0 0];

% Sampling time
Ts = 0.01; % Choose an appropriate sampling time

% Discretize the system
sys_continuous = ss(A, B, C, zeros(size(C, 1), size(B, 2)));
sys_discrete = c2d(sys_continuous, Ts);

% Extract discrete-time matrices
Ad = sys_discrete.A;
Bd = sys_discrete.B;
Cd = sys_discrete.C;

% Display the discrete system matrices
disp('Discrete-time A matrix:');
disp(Ad);
disp('Discrete-time B matrix:');
disp(Bd);
disp('Discrete-time C matrix:');
disp(Cd);

% Simulate the system with an initial condition
x0 = [0; 0; 0; 0; 0; 1; 0; 0; 0; 0; 0; 0]; % Initial state
N = 1000; % Number of simulation steps
u = zeros(size(B, 2), N); % Zero input for open-loop simulation

% Preallocate state and output arrays
x = zeros(size(Ad, 1), N);
y = zeros(size(Cd, 1), N);
x(:, 1) = x0;

% Simulate the system
for k = 1:N-1
    x(:, k+1) = Ad * x(:, k) + Bd * u(:, k);
    y(:, k) = Cd * x(:, k);
end
y(:, N) = Cd * x(:, N);

% Plot the displacements
time = (0:N-1) * Ts;
figure;
plot(time, y');
xlabel('Time (s)');
ylabel('Displacements');
legend('d1', 'd2', 'd3', 'd4', 'd5', 'd6');
title('Displacements of the Masses');
grid on;
```

## 11) Known Issues and Required Clarifications for Consistency

1. **Comment mismatch in `prep_final.m`**
   - It says "example: observe all displacements", but `C` is 1×12 measuring only `x1`.
   - If "observe all displacements" is intended, `C` should be a 6×12 selector ([I_6, 0]). As written, it is baseline `x1` only.

2. **Legend mismatch in `prep_final.m`**
   - The plot uses `y = Cd x`, where `Cd` is 1×12, so `y` has 1 trace.
   - The legend lists six entries `d1..d6`. That is incorrect for this `y`.
   - Correct options:
     - Keep `C = [1 0 ... 0]`, plot `y`, and use a single legend entry like `x1`.
     - Or define a displacement selector (C_{\text{disp}} = [I_6, 0]), compute (y_{\text{disp}} = C_{\text{disp}} x), then plot six traces with legend `x1..x6`.

3. **Initial condition mismatch between `prep_final.m` and Part 2 exam requirement**
   - `prep_final.m` uses (x_0 = [0,0,0,0,0,1,0,0,0,0,0,0]^T) for its open-loop simulation.
   - Part 2 exam requirement uses:
     - (x_0 = [0,0,0,1,1,1,0,0,0,0,0,0]^T)
     - (\hat{x}_0 = [0,0,0,0,0,1,0,0,0,0,0,0]^T)
   - For Parts 2 and onward, use the exam-specified initial conditions for the observer simulations.

## 12) Page Number Placeholders

**NOTE:** PDF page numbers need to be filled manually by opening `docs/sources/final_exam.pdf` and recording the exact page and question reference. The PDF file exists at `docs/sources/final_exam.pdf` but page numbers are not yet recorded.

**To fill:** Open the PDF, locate each question, and record the page number below.

- Baseline single output statement. Page: ____. Question: ____.
- Part 2 (C_{\text{part2}}), (x_0), (\hat{x}_0). Page: ____. Question: 2.
- Part 3 cost. Page: ____. Question: 3.
- Part 5 noise model. Page: ____. Question: 5.
- Part 7 cases. Page: ____. Question: 7.

