---
name: Create anchor document
overview: Create a comprehensive anchor document at docs/00_anchor.md that serves as the central reference for the EE571 final project, documenting the 6-mass spring system, model conventions, deliverables structure, and workflow for all 7 parts plus optional conversion.
todos: []
---

# Create

Anchor Document for EE571 Final Project

## Overview

Create `docs/00_anchor.md` as the central reference document for the entire project. This document will establish conventions, file structure, and workflow guidelines that all parts must follow.

## Key Sections to Include

### 1. Project Summary

- Brief description: 6-mass spring chain system
- 12-state model: [x1..x6, x1dot..x6dot]
- 3 inputs: u1 (tension on spring 2), u2 (tension on spring 6), u3 (force on mass 2)
- Task sequence: observability → observer → LQR → reduced input → Kalman filter → LQG → sensor analysis

### 2. File Inventory and Role

- **Reference file**: `matlab/prep_final.m`
- Authoritative source for: A, B, C, Ts=0.01, x0, N=1000
- Contains discrete-time model via c2d
- **Known issue**: C is 1x12 (only x1 measured) but legend shows d1..d6 - plots must match actual outputs

### 3. Model Conventions

- **State ordering**: [x1, x2, x3, x4, x5, x6, x1dot, x2dot, x3dot, x4dot, x5dot, x6dot]
- **Input matrix B**: 12×3, nonzero rows indicate which states are directly actuated
- **Discretization**: Zero-order hold (ZOH) at Ts=0.01s, match MATLAB `c2d(ss(A,B,C,0), Ts)`

### 4. Deliverables Structure

- **Python implementations**: `python/part1.py` through `python/part7.py`
- **MATLAB implementations**: `matlab/part1.m` through `matlab/part7.m` (optional Part 8)
- **Documentation per part**: `docs/01_part1/plan.md`, `docs/01_part1/closeout.md`, etc.

### 5. Global Decisions

- **Simulation horizon**: Default N=1000 steps (10s at Ts=0.01) unless part specifies otherwise
- **Cost metrics**: Finite-horizon J = Σ(uᵀu + y₁² + y₆²) for Part 3, adapt as needed
- **Input metrics**: Max input magnitude per channel, total cost
- **Estimation metrics**: RMS error, per-state error, selected state tracking
- **Random seed**: Fixed seed for reproducibility (e.g., seed=42) for noisy simulations

### 6. Part Breakdown

- **Part 0**: Workspace setup + baseline verification (verify prep_final.m matches expectations)
- **Part 1**: Observability analysis + Kalman decomposition
- **Part 2**: Add sensor at mass 6 (C becomes 2×12), observer design, estimation simulation
- **Part 3**: LQR design with cost J = Σ(uᵀu + y₁² + y₆²), uses estimated states
- **Part 4**: Remove u3, redesign LQR, compare cost and max input vs Part 3
- **Part 5**: Kalman filter with v~N(0,0.1I_p), w~N(0,0.05I_m)
- **Part 6**: Combine Part 3 LQR + Part 5 Kalman (LQG), compare outputs to Part 3
- **Part 7**: Two C matrix cases (4×12 and 6×12), evaluate sensor benefit
- **Part 8** (optional): Python→MATLAB conversion and validation

### 7. How to Use This Repo

- Each chat session works on ONE part only
- Before coding: ensure `docs/XX_partYY/plan.md` exists
- After completion: create `docs/XX_partYY/closeout.md` with results summary
- Reference anchor document for conventions and decisions

## Implementation Notes

- Use clear headings and checklists (not long prose)
- Include cross-references to prep_final.m for matrix definitions
- Note the plotting bug in prep_final.m (C vs legend mismatch)