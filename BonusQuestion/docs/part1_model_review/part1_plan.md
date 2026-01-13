# Part 1: Model and Signals Review

**Status: COMPLETED**

## Scope

Understand what is simulated and what signals are available for control and plotting.

## Key Findings

### Plant State Vector
`x = [X, Y, psi, vx, vy, r]` (6 states)
- `X, Y`: Global position [m]
- `psi`: Heading angle [rad]
- `vx, vy`: Body-frame velocities [m/s]
- `r`: Yaw rate [rad/s]

### Input Vector
`u = [delta, ax]` (2 inputs)
- `delta`: Steering angle [rad], saturated to [-25, +25] deg
- `ax`: Longitudinal acceleration [m/s^2], saturated to [-6, +3]

### Error State Vector
`x_e = [vy, r, ey, epsi, ev]` (5 states)
- `vy`: Lateral velocity (from plant state)
- `r`: Yaw rate (from plant state)
- `ey`: Cross-track error
- `epsi`: Heading error (wrapped to [-pi, pi])
- `ev`: Speed error (vx - v_ref)

### Control Structure
```
u = u_ff + u_reg
```
- Feedforward: `delta_ff = (lf + lr) * kappa_ref`, `ax_ff = a_ref`
- Regulation: `u_reg = -K @ x_e`

## Acceptance Checks

- [x] Plant state vector identified
- [x] Input vector identified
- [x] Reference signals identified
- [x] Error signals identified
- [x] Error state vector structure documented
- [x] Feedforward structure understood
