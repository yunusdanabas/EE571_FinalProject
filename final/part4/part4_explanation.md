# Part 4: LQR Controller with Reduced Input - Detailed Explanation

## Overview

Part 4 redesigns the LQR controller with only **two inputs** (u1 and u2), removing u3. This demonstrates the impact of reducing actuator authority on control performance.

## What is Reduced Input Control?

**Reduced input control** means we have fewer actuators available to control the system. In Part 3, we had three inputs (u1, u2, u3), but in Part 4, we remove u3, leaving only u1 and u2.

**Why analyze this?**
- In practice, actuators may fail or be unavailable
- Some systems are designed with fewer actuators for cost reasons
- Understanding the impact helps with system design decisions

## Code Structure

### Main Components

1. **Input Matrix Reduction**
   ```python
   Bd_red = Bd[:, [0, 1]]  # Remove third column (u3)
   ```
   - Original: Bd shape (12, 3)
   - Reduced: Bd_red shape (12, 2)

2. **LQR Redesign**
   - Same state weight Q (penalizes x1 and x6)
   - Reduced input weight: R_red = I₂ (2×2, down from I₃)
   - Solve DARE with reduced input matrix

3. **Performance Comparison**
   - Compare cost J_red vs Part 3's J
   - Analyze performance degradation

## Step-by-Step Execution

### Step 1: Create Reduced Input Matrix

```python
Bd_red = Bd[:, [0, 1]]  # Keep only columns 0 and 1 (u1, u2)
```

**What this means:**
- Original B matrix has 3 columns: [B1, B2, B3]
- Reduced B matrix has 2 columns: [B1, B2]
- Input u3 can no longer affect the system

**System dynamics change:**
- Part 3: x[k+1] = Ad @ x[k] + Bd @ u[k] (3 inputs)
- Part 4: x[k+1] = Ad @ x[k] + Bd_red @ u_red[k] (2 inputs)

### Step 2: Redesign LQR Controller

```python
Q = Cy.T @ Cy  # Same as Part 3 (penalizes x1 and x6)
R_red = np.eye(2)  # Reduced from I₃ to I₂
K_red, P_red, rho = design_lqr(Ad, Bd_red, Q, R_red)
```

**Key changes:**
- **State weight Q**: Unchanged (same regulation objectives)
- **Input weight R_red**: Reduced from 3×3 to 2×2
- **LQR gain K_red**: Reduced from (3×12) to (2×12)

**DARE solution:**
- Solves the same Riccati equation, but with reduced input matrix
- The optimal gain K_red is different from K (can't just remove a row)

### Step 3: Simulate Closed-Loop System

```python
results = simulate_lqr_reduced(Ad, Bd_red, Cy, K_red, L, x0, xhat0, N, Ts)
```

**Control law:**
- Part 3: u[k] = -K @ xhat[k] (3 inputs)
- Part 4: u_red[k] = -K_red @ xhat[k] (2 inputs: u1 and u2)

**Observer update:**
- Observer also uses reduced input: xhat[k+1] = Ad @ xhat[k] + Bd_red @ u_red[k] + L @ (y[k] - yhat[k])
- This ensures observer and plant use the same input

### Step 4: Compare Performance

```python
J_red = compute_cost(results['x'], results['u_red'], Cy, N)
cost_increase = (J_red - J_part3) / J_part3 * 100
```

**Results:**
- Part 3 cost: J = 9.057×10⁷
- Part 4 cost: J_red = 1.348×10⁸
- **Cost increase: 48.84%**

## Understanding the Results

### Performance Degradation

**Why does removing u3 increase cost?**

1. **Reduced control authority:**
   - Fewer "knobs" to control the system
   - u1 and u2 must work harder to compensate
   - Less flexibility in control design

2. **Higher control effort:**
   - Max |u| increases from 3.60×10³ to 4.95×10³
   - Remaining inputs must be larger to achieve similar performance
   - More energy required

3. **Worse regulation:**
   - Outputs take longer to converge
   - Larger deviations from zero
   - Higher accumulated cost

### System Controllability

**Is the system still controllable with 2 inputs?**
- **Yes**: The system remains stabilizable
- u1 and u2 provide sufficient control authority
- Closed-loop spectral radius: 0.999518 (stable, similar to Part 3)

**Why is it still controllable?**
- The system has 3 inputs, but only needs 2 for controllability
- u1 and u2 can control all 12 states (system is over-actuated)
- Removing u3 reduces performance but doesn't break controllability

### Trade-Off Analysis

**Fundamental trade-off:**
- **More actuators** → Better performance, more flexibility
- **Fewer actuators** → Lower cost, simpler design, but worse performance

**In this case:**
- Removing u3 increases cost by 48.84%
- System is still stable and controllable
- Performance degradation is significant but acceptable

## Key Insights

### 1. Actuator Redundancy

The system has **actuator redundancy**:
- 3 inputs available
- Only 2 needed for controllability
- The third input (u3) provides performance benefits

**In practice:**
- Redundant actuators improve performance and reliability
- If one actuator fails, system can still operate (with degraded performance)
- Trade-off: Cost vs performance vs reliability

### 2. Control Authority

**Control authority** refers to the system's ability to affect its states:
- More inputs → More control authority → Better performance
- Fewer inputs → Less control authority → Worse performance

**In this case:**
- u1 and u2 provide sufficient authority for stability
- u3 provides additional authority for better performance
- Removing u3 reduces authority → higher cost

### 3. Optimal Control with Constraints

LQR automatically adapts to reduced inputs:
- K_red is different from K (not just removing a row)
- DARE finds the optimal gain for the reduced system
- Controller is still optimal, but for a more constrained system

## Comparison with Part 3

| Metric | Part 3 (3 inputs) | Part 4 (2 inputs) | Change |
|--------|-------------------|-------------------|--------|
| **LQR gain shape** | (3, 12) | (2, 12) | Reduced |
| **Spectral radius** | 0.999463 | 0.999518 | +0.006% |
| **Total cost J** | 9.057×10⁷ | 1.348×10⁸ | +48.84% |
| **Max \|u\|** | 3.597×10³ | 4.951×10³ | +37.6% |

**Key observations:**
1. **Stability maintained**: Spectral radius still < 1.0
2. **Significant cost increase**: 48.84% higher cost
3. **Higher control effort**: Max |u| increases by 37.6%
4. **System still works**: Controllability preserved

## What Happens Next?

Part 4 demonstrates the impact of actuator reduction. In subsequent parts:
- **Part 5**: Designs Kalman filter for noisy systems
- **Part 6**: Combines LQR + Kalman filter (LQG)
- **Part 7**: Analyzes sensor augmentation (opposite of Part 4: adding sensors instead of removing actuators)

## Summary

Part 4 successfully:
1. ✓ Created reduced input matrix (removed u3)
2. ✓ Redesigned LQR controller with 2 inputs
3. ✓ Verified closed-loop stability (spectral radius = 0.999518)
4. ✓ Simulated reduced-input control
5. ✓ Quantified performance degradation (48.84% cost increase)
6. ✓ Demonstrated system remains controllable with 2 inputs

**Key finding**: Removing one input (u3) significantly degrades performance (48.84% cost increase) but the system remains stable and controllable. This demonstrates the value of having all three inputs available for control.
