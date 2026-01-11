# Part 0: Baseline Verification Report

## Objective

Verify the system model by discretizing the continuous-time matrices and running a baseline open-loop simulation to establish the baseline behavior of the discretized system.

## Approach

1. Built continuous-time model matrices (A, B, C) from `prep_final.m` definitions
2. Discretized using zero-order hold (ZOH) at sampling time Ts = 0.01 seconds
3. Simulated open-loop system with zero input for N = 1000 steps (10 seconds)
4. Generated plots for system output and all mass displacements

## Key Results

- **Matrix dimensions:**
  - A: (12, 12) - state matrix
  - B: (12, 3) - input matrix  
  - C: (1, 12) - output matrix (measures displacement of mass 1 only)
  
- **Discretization:** Successfully discretized using ZOH method at Ts = 0.01s

- **Simulation:** Open-loop simulation completed for 1000 time steps with zero input

## Figures

- **`outputs/output_plot.png`**: System output y = Cx showing the displacement of mass 1 (single output trace)
- **`outputs/displacements_plot.png`**: All 6 mass displacements (x₁ through x₆) showing the internal state evolution

## Findings

The system demonstrates oscillatory behavior in open-loop with the given initial condition (x₆ = 1). The output (mass 1 displacement) shows a damped oscillatory response, and all six mass displacements show coordinated motion consistent with the spring-chain dynamics. The system is marginally stable, as expected for an undamped spring-mass system without control input.