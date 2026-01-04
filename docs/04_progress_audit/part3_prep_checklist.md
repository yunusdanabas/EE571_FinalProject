# Part 3 Preparation Checklist

## Part 3 Requirements Confirmation (Before Starting Implementation)

### 1. Cost Function Mapping Confirmation

**Exam Requirement**: 
\[
J = \sum \left(u^T u + y_1^2 + y_6^2\right), \quad u = [u_1\ u_2\ u_3]^T
\]

**Confirmation**:
- [X] **Cost term mapping verified**: Under Part 2 measurement matrix \(C_{\text{part2}}\), the measured outputs are \(y = [x_1, x_6]^T\)
- [X] **Cost interpretation**: The cost term \(y_1^2 + y_6^2\) corresponds to penalizing the measured displacement outputs \(x_1\) and \(x_6\)
- [X] **Implementation note**: Since \(y_1 = x_1\) and \(y_6 = x_6\) under \(C_{\text{part2}}\), the cost can be constructed as:
  - Option 1: \(Q = C_{\text{part2}}^T W C_{\text{part2}}\) where \(W = \begin{bmatrix} 1 & 0 \\ 0 & 1 \end{bmatrix}\)
  - Option 2: Direct state weighting: \(Q\) matrix with ones at positions (1,1) and (6,6) for displacements
  - Input cost: \(R = I_3\) (3×3 identity for u^T u term)

**Source**: Verified from `docs/sources/final_exam_extract.md` Section 5 (Part 3 Requirement)

---

### 2. Estimated States Usage Confirmation

**Exam Requirement**: "Design an LQR for the observable system from Part 2 and simulate. **The regulator uses estimated states**."

**Confirmation**:
- [X] **LQR uses estimated states**: The exam explicitly states "The regulator uses estimated states"
- [X] **Implementation requirement**: The LQR controller must use \(\hat{x}[k]\) (estimated states from Part 2 observer), NOT \(x[k]\) (true states)
- [X] **Control law**: \(u[k] = -K \hat{x}[k]\) where \(K\) is the LQR gain matrix and \(\hat{x}[k]\) comes from the Part 2 observer

**Source**: Verified from `docs/sources/final_exam_extract.md` Section 5 (Part 3 Requirement)

---

### 3. System Setup Confirmation

**Confirmation**:
- [X] **Use Part 2 observable system**: Same measurement matrix \(C_{\text{part2}}\) (2×12, measures x1 and x6)
- [X] **Use Part 2 initial conditions**: 
  - Actual state: \(x_0 = [0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0]^T\)
  - Observer initial state: \(\hat{x}_0 = [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]^T\)
- [X] **Use Part 2 observer gain**: Observer gain \(L\) from Part 2 must be reused

**Source**: Verified from `docs/sources/final_exam_extract.md` and `docs/00_anchor.md`

---

### 4. Implementation Notes

**Key Implementation Points**:
1. The LQR design uses the discrete-time system \((A_d, B_d)\) from Part 0
2. The cost function maps to LQR weights:
   - State weight \(Q\): Penalizes \(x_1^2 + x_6^2\) (can construct via \(C_{\text{part2}}^T W C_{\text{part2}}\) or direct weighting)
   - Input weight \(R = I_3\): Penalizes \(u^T u\)
3. The controller uses estimated states: \(u[k] = -K \hat{x}[k]\)
4. The simulation must couple:
   - Plant: \(x[k+1] = A_d x[k] + B_d u[k]\)
   - Observer: \(\hat{x}[k+1] = A_d \hat{x}[k] + B_d u[k] + L (y[k] - \hat{y}[k])\)
   - Controller: \(u[k] = -K \hat{x}[k]\)
   - Measurements: \(y[k] = C_{\text{part2}} x[k]\), \(\hat{y}[k] = C_{\text{part2}} \hat{x}[k]\)

**Status**: Ready to proceed with Part 3 implementation

---

## Summary

- [X] Cost function mapping confirmed: \(y_1^2 + y_6^2\) = penalize \(x_1\) and \(x_6\)
- [X] Estimated states usage confirmed: LQR must use \(\hat{x}[k]\), not \(x[k]\)
- [X] System setup confirmed: Use Part 2 observer and initial conditions
- [X] Implementation approach clear: Coupled plant-observer-controller simulation

**Ready to start Part 3**: YES

