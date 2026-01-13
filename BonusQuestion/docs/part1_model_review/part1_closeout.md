# Part 1 Closeout

## Summary of Work

Completed comprehensive model and signals review for the Vehicle Reference Tracking Project. This phase involved:

1. **Plant state vector identification**: Documented the 6-state nonlinear plant model `[X, Y, ψ, v_x, v_y, r]`
2. **Input vector identification**: Documented the 2-input control vector `[δ, a_x]` with saturation limits
3. **Reference signals identification**: Identified all 6 reference signals and their generation methods
4. **Error signals identification**: Documented the 3 error signals `e_y`, `e_ψ`, `e_v` and their computation
5. **Error state vector construction**: Mapped the 5-state error vector `x_e = [v_y; r; e_y; e_ψ; e_v]` to code variables
6. **Feedforward structure**: Documented feedforward terms and their purpose
7. **Continuous-time model matrices**: Reviewed `Ac` and `Bc` structure and meaning
8. **Variable mapping table**: Created comprehensive code-to-mathematics mapping
9. **Integration points**: Identified exact locations where controller will be added
10. **Plot templates**: Reviewed existing plotting infrastructure

**No controller implementation was performed** - this phase is review/documentation only.

## Files Changed/Added

- **Updated**: `docs/part1_model_review/part1_closeout.md` (this file) with complete model review

## How to Run / Reproduce

This is a review phase with no executable code changes. To verify the review findings:

1. Open `vehicle_dlqr.m` in MATLAB
2. Verify the locations identified in this document match the code
3. Check that no controller code exists at lines 72-77 and 113-118 (placeholders only)

## Checks Performed

### Plant State Vector Identification
- ✅ Plant state `x(k,:) = [X, Y, ψ, v_x, v_y, r]` identified at line 84
- ✅ Initial condition structure verified at line 87
- ✅ State extraction in simulation loop verified at lines 96-97
- ✅ All 6 states mapped to mathematical symbols

### Input Vector Identification
- ✅ Input vector `u = [δ, a_x]` identified
- ✅ Saturation limits verified: steering `[-25°, +25°]`, acceleration `[-6, 3] m/s²`
- ✅ Input logging structure verified at line 89
- ✅ Placeholder location for input calculation identified at lines 113-118

### Reference Signals Identification
- ✅ All 6 reference signals identified and mapped
- ✅ Reference generation function `integrate_reference()` reviewed
- ✅ Reference usage in simulation loop verified at lines 99-103

### Error Signals Identification
- ✅ All 3 error signals identified: `e_y`, `e_ψ`, `e_v`
- ✅ Error computation function `lateral_heading_error()` reviewed
- ✅ Error logging structure verified at lines 90-92, 125-127

### Error State Vector Construction
- ✅ Error state vector structure `x_e = [v_y; r; e_y; e_ψ; e_v]` fully documented
- ✅ Mapping from code variables to error state components complete
- ✅ Construction location identified (after line 107, before line 113)

### Feedforward Structure
- ✅ Feedforward terms identified and documented
- ✅ Computation location verified at lines 110-111

### Continuous-Time Model Matrices
- ✅ `Ac` (5×5) and `Bc` (5×2) matrices identified at lines 54-71
- ✅ Matrix structure and meaning documented

### Variable Mapping Table
- ✅ Complete mapping table created (see below)

### Integration Points
- ✅ Controller design location identified (lines 72-77)
- ✅ Error state construction location identified (after line 107)
- ✅ Regulation input calculation location identified (lines 113-118)
- ✅ Feedforward combination location identified (after regulation, before saturation)
- ✅ Saturation application location verified (lines 121-122)

### Plot Templates
- ✅ All 3 plot templates reviewed and documented

## Outputs Produced

### Variable Mapping Table

| Code Variable | Mathematical Symbol | Physical Meaning | Units | Code Location |
|--------------|-------------------|------------------|-------|---------------|
| **Plant State Vector** |
| `x(k,1)` | `X` | Global X position | [m] | Lines 84, 87, 96 |
| `x(k,2)` | `Y` | Global Y position | [m] | Lines 84, 87, 96 |
| `x(k,3)` | `ψ` | Heading angle | [rad] | Lines 84, 87, 96 |
| `x(k,4)` | `v_x` | Longitudinal velocity | [m/s] | Lines 84, 87, 96 |
| `x(k,5)` | `v_y` | Lateral velocity | [m/s] | Lines 84, 87, 96 |
| `x(k,6)` | `r` | Yaw rate | [rad/s] | Lines 84, 87, 96 |
| **Input Vector** |
| `steering_input` | `δ` | Steering angle | [rad] | Lines 113-118, 121, 124 |
| `throttle` | `a_x` | Longitudinal acceleration | [m/s²] | Lines 113-118, 122, 124 |
| **Reference Signals** |
| `Xref` | `X_ref` | Reference X position | [m] | Lines 37, 100, 146 |
| `Yref` | `Y_ref` | Reference Y position | [m] | Lines 37, 100, 146 |
| `psiref` | `ψ_ref` | Reference heading angle | [rad] | Lines 37, 100 |
| `v_ref` | `v_ref` | Reference speed | [m/s] | Lines 34, 101 |
| `kappa_ref` | `κ_ref` | Reference curvature | [1/m] | Lines 33, 103 |
| `a_ref` | `a_ref` | Reference acceleration | [m/s²] | Lines 35, 102 |
| **Error Signals** |
| `ey` | `e_y` | Cross-track error | [m] | Lines 106, 125 |
| `epsi` | `e_ψ` | Heading error (wrapped to [-π, π]) | [rad] | Lines 106, 126 |
| `ev` | `e_v` | Speed error (`v_x - v_ref`) | [m/s] | Lines 107, 127 |
| **Error State Vector** |
| `x_e(1)` | `v_y` | Lateral velocity (from plant state) | [m/s] | To be constructed: `x(k,5)` |
| `x_e(2)` | `r` | Yaw rate (from plant state) | [rad/s] | To be constructed: `x(k,6)` |
| `x_e(3)` | `e_y` | Cross-track error | [m] | To be constructed: `ey` |
| `x_e(4)` | `e_ψ` | Heading error | [rad] | To be constructed: `epsi` |
| `x_e(5)` | `e_v` | Speed error | [m/s] | To be constructed: `ev` |
| **Feedforward Terms** |
| `steering_feed_forward` | `δ_ff` | Feedforward steering = `(l_f + l_r) * κ_ref` | [rad] | Line 110 |
| `throttle_feed_forward` | `a_{x,ff}` | Feedforward acceleration = `a_ref` | [m/s²] | Line 111 |
| **Model Matrices** |
| `Ac` | `A_c` | Continuous-time error state matrix (5×5) | - | Lines 54-71 |
| `Bc` | `B_c` | Continuous-time input matrix (5×2) | - | Lines 54-71 |
| **Parameters** |
| `params.m` | `m` | Vehicle mass | [kg] | Line 16 |
| `params.Iz` | `I_z` | Yaw moment of inertia | [kg·m²] | Line 17 |
| `params.lf` | `l_f` | Front axle distance from CG | [m] | Line 18 |
| `params.lr` | `l_r` | Rear axle distance from CG | [m] | Line 19 |
| `params.Cf` | `C_f` | Front cornering stiffness | [N/rad] | Line 20 |
| `params.Cr` | `C_r` | Rear cornering stiffness | [N/rad] | Line 21 |
| `Vx0` | `V_{x0}` | Nominal linearization speed | [m/s] | Line 30 |
| `Ts` | `T_s` | Controller sampling time | [s] | Line 24 |
| `Tend` | `T_{end}` | Simulation duration | [s] | Line 25 |
| **Logged Signals** |
| `u_log` | - | Logged inputs `[δ, a_x]` | - | Lines 89, 124 |
| `ey_log` | - | Logged cross-track error | [m] | Lines 90, 125 |
| `epsi_log` | - | Logged heading error | [rad] | Lines 91, 126 |
| `ev_log` | - | Logged speed error | [m/s] | Lines 92, 127 |

### Plant State Vector Details

**Location**: Line 84 (`x = zeros(N,6)`) defines the state storage array

**Structure**: `x(k,:) = [X, Y, ψ, v_x, v_y, r]` where `k` is the time step index

**Initial Condition** (Line 87):
```matlab
x(1,:) = [ Xref(1)-2.0,  Yref(1)+1.0,  psiref(1)+deg2rad(8),  Vx0-5,  0.0,  0.0 ];
```

This defines baseline offsets from reference:
- `X(0) = X_ref(0) - 2.0 m`
- `Y(0) = Y_ref(0) + 1.0 m`
- `ψ(0) = ψ_ref(0) + 8°`
- `v_x(0) = V_{x0} - 5 m/s` (where `V_{x0} = 15 m/s`)
- `v_y(0) = 0 m/s`
- `r(0) = 0 rad/s`

**State Extraction** (Lines 96-97):
```matlab
X   = x(k,1);  Y   = x(k,2);  psi = x(k,3);
vx  = x(k,4);  vy  = x(k,5);  r   = x(k,6);
```

### Input Vector Details

**Location**: Line 89 (`u_log = zeros(N,2)`) defines input logging array

**Structure**: `u = [δ, a_x]` where:
- `δ` = `steering_input`: Steering angle [rad]
- `a_x` = `throttle`: Longitudinal acceleration [m/s²]

**Saturation Limits** (Lines 121-122):
- Steering: `δ ∈ [-25°, +25°]` → `[deg2rad(-25), deg2rad(25)]`
- Acceleration: `a_x ∈ [-6, +3] m/s²`

**Placeholder Location**: Lines 113-118 contain comments indicating where inputs should be calculated

### Reference Signals Details

**Generation** (Lines 33-37):
- `kappa_ref`: Reference curvature `κ_ref(t) = 0.01*sin(0.35*t) + 0.005*sin(0.10*t)` [1/m]
- `v_ref`: Reference speed `v_ref(t) = Vx0 + 1.0*sin(0.15*t)` [m/s]
- `a_ref`: Reference acceleration (approximated as `diff(v_ref)/Ts`) [m/s²]
- `[Xref, Yref, psiref]`: Reference pose integrated by `integrate_reference()` function

**Function**: `integrate_reference()` (Lines 207-219) integrates reference pose from speed and curvature:
- `ψ̇_ref = v_ref * κ_ref`
- `Ẋ_ref = v_ref * cos(ψ_ref)`
- `Ẏ_ref = v_ref * sin(ψ_ref)`

**Usage in Loop** (Lines 99-103):
```matlab
Xr   = Xref(k); Yr = Yref(k); psir = psiref(k);
vr   = v_ref(k);
ar   = a_ref(k);
kap  = kappa_ref(k);
```

### Error Signals Details

**Computation Location**: Lines 106-107 in simulation loop

**Cross-Track and Heading Errors** (Line 106):
```matlab
[ey, epsi] = lateral_heading_error(X,Y,psi, Xr,Yr,psir);
```

**Function**: `lateral_heading_error()` (Lines 221-227):
- Computes cross-track error `e_y` as projection of position error onto reference normal direction
- Computes heading error `e_ψ = wrapToPi(ψ - ψ_ref)` (wrapped to [-π, π])

**Speed Error** (Line 107):
```matlab
ev = vx - vr;
```

**Logging**: Lines 90-92 (initialization), 125-127 (in loop)

### Error State Vector Construction

**Mathematical Definition**: `x_e = [v_y; r; e_y; e_ψ; e_v]` (5 states)

**Code Mapping**:
- `x_e(1)` = `v_y` = `x(k,5)` (lateral velocity from plant state)
- `x_e(2)` = `r` = `x(k,6)` (yaw rate from plant state)
- `x_e(3)` = `e_y` = `ey` (cross-track error, computed at line 106)
- `x_e(4)` = `e_ψ` = `epsi` (heading error, computed at line 106)
- `x_e(5)` = `e_v` = `ev` = `vx - vr` (speed error, computed at line 107)

**Construction Location**: Should be built in simulation loop **after** error computation (after line 107) and **before** input calculation (before line 113)

**Current Status**: Not yet constructed - placeholder exists at lines 113-118

**Expected Code Structure** (to be added):
```matlab
% Build error state vector
x_e = [vy; r; ey; epsi; ev];
```

### Feedforward Structure Details

**Location**: Lines 110-111 in simulation loop

**Components**:
- `steering_feed_forward` → `δ_ff = (l_f + l_r) * κ_ref` (geometric steering for nominal curvature)
- `throttle_feed_forward` → `a_{x,ff} = a_ref` (reference acceleration)

**Purpose**: Feedforward handles nominal tracking along the reference trajectory. The regulator (to be implemented) will handle deviations from the reference (error states).

**Combination**: Feedforward and regulation inputs will be combined:
- `steering_input = steering_feed_forward + steering_reg`
- `throttle = throttle_feed_forward + ax_reg`

### Continuous-Time Error Model Matrices

**Location**: Lines 54-71

**Matrices**:
- `Ac`: 5×5 continuous-time error state matrix
- `Bc`: 5×2 continuous-time input matrix

**Model**: `ẋ_e = Ac * x_e + Bc * u_reg`

**Matrix Structure**:

**Rows 1-2: `[v_y; r]` dynamics** (lateral velocity and yaw rate)
- `Ac(1,1) = A11 = -(C_f + C_r)/(m*V_{x0})`
- `Ac(1,2) = A12 = -(V_{x0} + (l_f*C_f - l_r*C_r)/(m*V_{x0}))`
- `Ac(2,1) = A21 = -(l_f*C_f - l_r*C_r)/(I_z*V_{x0})`
- `Ac(2,2) = A22 = -(l_f²*C_f + l_r²*C_r)/(I_z*V_{x0})`
- `Bc(1,1) = B_{vy,δ} = C_f/m` (steering effect on lateral velocity)
- `Bc(2,1) = B_{r,δ} = l_f*C_f/I_z` (steering effect on yaw rate)

**Row 3: `e_y` dynamics** (cross-track error)
- `Ac(3,1) = 1` (lateral velocity contributes to cross-track error)
- `Ac(3,4) = V_{x0}` (heading error contributes to cross-track error)
- `e_y_dot = v_y + V_{x0} * e_ψ`

**Row 4: `e_ψ` dynamics** (heading error)
- `Ac(4,2) = 1` (yaw rate equals heading error rate)
- `e_ψ_dot = r` (reference curvature handled by feedforward)

**Row 5: `e_v` dynamics** (speed error)
- `Bc(5,2) = 1` (acceleration directly affects speed error)
- `e_v_dot = a_x` (reference acceleration handled by feedforward)

**Note**: The model assumes `u_reg = [δ_reg; a_{x,reg}]` is the regulation input (feedforward is separate).

### Integration Points for Controller

**1. Controller Design Location** (Lines 72-77):
- Placeholder comments: `% DESIGN YOUR CONTROLLER HERE!`
- Location: After `Ac` and `Bc` definition, before simulation loop
- Tasks for Part 2-4:
  - Discretize `(Ac, Bc)` to obtain `(Ad, Bd)` using `c2d_zoh_exact()`
  - Compute controller gain `K` (either `K_LQR` or `K_PP`)

**2. Error State Construction Location** (After line 107, before line 113):
- Should be built after error computation (lines 106-107)
- Before feedforward computation (line 110)
- Code structure:
  ```matlab
  % errors (line 106-107)
  [ey, epsi] = lateral_heading_error(X,Y,psi, Xr,Yr,psir);
  ev = vx - vr;
  
  % Build error state vector (TO BE ADDED)
  x_e = [vy; r; ey; epsi; ev];
  
  % feedforward (line 110-111)
  steering_feed_forward = (lf+lr)*kap;
  throttle_feed_forward = ar;
  ```

**3. Regulation Input Calculation Location** (Lines 113-118):
- Placeholder comments: `% CALCULATE THE INPUTS HERE!`
- Tasks:
  - Compute regulation input: `u_reg = -K * x_e`
  - Extract components: `steering_reg = u_reg(1)`, `ax_reg = u_reg(2)`
  - Combine with feedforward: `steering_input = steering_feed_forward + steering_reg`
  - Combine with feedforward: `throttle = throttle_feed_forward + ax_reg`

**4. Feedforward Combination**:
- Location: After regulation calculation, before saturation
- Order: Feedforward + Regulation → Saturation

**5. Saturation Application** (Lines 121-122):
- Applied after combining feedforward and regulation
- Steering: `min(max(steering_input, deg2rad(-25)), deg2rad(25))`
- Acceleration: `min(max(throttle, -6), 3)`

### Plot Templates Review

**Location**: Lines 144-173

**Plot 1: Trajectory** (Lines 145-151):
- Shows reference path (`Xref, Yref`) vs. vehicle path (`x(:,1), x(:,2)`)
- Uses: `Xref`, `Yref`, `x(:,1)`, `x(:,2)`

**Plot 2: Tracking Errors** (Lines 153-164):
- Three subplots showing error time histories:
  - `e_y` vs. `t` (cross-track error)
  - `e_ψ` vs. `t` (heading error, converted to degrees)
  - `e_v` vs. `t` (speed error)
- Uses: `t`, `ey_log`, `epsi_log`, `ev_log`

**Plot 3: Inputs** (Lines 166-173):
- Two subplots showing control inputs:
  - `δ` vs. `t` (steering angle, converted to degrees)
  - `a_x` vs. `t` (longitudinal acceleration)
- Uses: `t`, `u_log(:,1)`, `u_log(:,2)`

**Note**: All plots are already set up and will work once controller is implemented and inputs are calculated.

## Issues / TODOs for Next Part

### Handoff to Part 2 Agent

Part 2 agent should:

1. **Discretization Task**:
   - Location: Lines 72-77 (after `Ac`, `Bc` definition)
   - Use function: `[Ad, Bd] = c2d_zoh_exact(Ac, Bc, Ts)`
   - Verify: `Ad` is 5×5, `Bd` is 5×2
   - Sampling time: `Ts = 0.02 s` (already defined at line 24)

2. **Error State Vector Confirmation**:
   - Structure: `x_e = [v_y; r; e_y; e_ψ; e_v]` (5 states)
   - Mapping confirmed in this document
   - Construction location: After line 107, before line 113

3. **Model Matrices Location**:
   - `Ac` and `Bc` are defined at lines 54-71
   - Both matrices are ready for discretization
   - No modifications needed to matrices themselves

4. **Important Notes**:
   - Discretization happens **before** controller design
   - Same `Ad`, `Bc` matrices will be used by both regulators (Part 3 and Part 4)
   - Error state vector structure must match the discretized model

### Notes for Future Parts

- **Part 3 (LQR)**: Will use `Ad`, `Bd` from Part 2, compute `K_LQR` using `dlqr()`
- **Part 4 (Pole Placement)**: Will use same `Ad`, `Bd` from Part 2, compute `K_PP` using `place()` or `acker()`
- **Part 5 (Experiments)**: Will need both regulators working, run 6 cases total

### Verification Checklist

- [x] All plant states identified and mapped
- [x] All inputs identified with saturation limits
- [x] All reference signals identified
- [x] All error signals identified with computation method
- [x] Error state vector `x_e` structure fully documented
- [x] Feedforward structure understood
- [x] Variable mapping table complete
- [x] Integration points clearly identified
- [x] No controller code has been written
- [x] Closeout document complete
