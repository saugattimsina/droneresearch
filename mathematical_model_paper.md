# Autonomous Target Interception: Mathematical Model

This document specifies the formal mathematical framework for vision-based
target extraction, state estimation, and kinematic control of an autonomous
interception drone. All equations are dimensionally consistent; all symbols
used in text are defined in §1.2.

---

## 1. Notation and Frame Conventions

### 1.1 Coordinate Frames

| Frame    | Axes                                    | Convention        |
|----------|-----------------------------------------|-------------------|
| Optical  | x right, y down, z forward              | OpenCV            |
| Body     | x forward, y right, z down              | MAVLink FRD       |
| World    | x north, y east, z down (local NED)     | MAVLink           |

Optical axes expressed in body coordinates:
    optical_x = body_y,   optical_y = body_z,   optical_z = body_x

Attitude angles (from IMU / attitude EKF):
- Roll  φ: positive right-wing-down (rotation about body x = optical z)
- Pitch θ: positive nose-up (rotation about body y = optical x)
- Yaw   ψ: positive clockwise viewed from above (rotation about body z = optical y)

All control loops operate in the **level body FRD frame**.

### 1.2 Symbol Table

| Symbol        | Meaning                                          | Units      |
|---------------|--------------------------------------------------|------------|
| f_px          | Pixel focal length                               | px         |
| W_frame       | Image width                                      | px         |
| θ_HFOV        | Horizontal field of view                         | rad        |
| W_target      | Physical target width                            | m          |
| H_target      | Physical target height                           | m          |
| w_px, h_px    | Bounding-box width / height                      | px         |
| (c_x, c_y)    | Principal point                                  | px         |
| (u, v)        | Bounding-box centroid                            | px         |
| Z, X, Y       | Target range, lateral, vertical (optical)        | m          |
| s             | √(w_px · h_px)                                   | px         |
| σ_j,x², σ_j,s²| Per-axis jerk variance                           | px²/s⁶     |
| τ             | Vision→control latency                           | s          |
| τ_v           | Inner-loop velocity-tracking time constant       | s          |
| Δt            | Filter time step                                 | s          |
| Δt_loop       | Control-loop period                              | s          |
| N             | ProNav navigation constant                       | —          |
| V_c           | Closing velocity                                 | m/s        |
| λ_h, λ_v      | Horizontal / vertical LOS angle                  | rad        |
| λ̇_h, λ̇_v      | Horizontal / vertical LOS rate                   | rad/s      |
| ψ̇             | Drone yaw rate                                   | rad/s      |
| Z_ref         | Standoff distance (PD mode)                      | m          |
| Z_commit      | ProNav terminal threshold (lower bound)          | m          |
| Z_high        | ProNav blend upper bound                         | m          |
| w(Z)          | Blend weight (0 = PD, 1 = ProNav)                | —          |
| R_base        | Baseline measurement covariance                  | px²        |
| ρ             | Scale-adaptive inflation factor                  | —          |
| α             | EMA smoothing coefficient                        | —          |
| AR            | Bounding-box aspect ratio                        | —          |
| N_coast       | Coast frames before loss declaration             | —          |
| N_lost        | Frames without detection → track drop            | —          |
| χ²₃(0.99)     | Chi-square 3-DOF 99% quantile (11.34)            | —          |

---

## 2. Vision-to-Spatial Projection

### 2.1 Focal Length

    f_px = (W_frame / 2) / tan(θ_HFOV / 2)

**Calibration note.** On real hardware, f_px and (c_x, c_y) should be
obtained from a one-time intrinsic calibration (e.g., Zhang), not from
nominal HFOV, which carries ±2–5% manufacturing tolerance.

### 2.2 Depth from Bounding Box

The naive pairing Z = f_px · W_target / max(w_px, h_px) is dimensionally
inconsistent whenever the taller pixel dimension does not correspond to the
target's physical width. For a target with known W_target and H_target, use
the yaw-invariant estimate:

    Z = sqrt( f_px² · W_target · H_target / (w_px · h_px) )

This is exact when the target is fronto-parallel and remains a useful
small-angle approximation under mild yaw.

**Known limitation (yaw collapse).** Under heavy yaw, one dimension of the
bounding box collapses toward zero and Z is over-estimated no matter which
formula is used. This is a fundamental monocular ambiguity. Mitigations:
(a) fuse with stereo or a known-pose marker, (b) reject depth updates when
the observed AR deviates from the target's nominal AR by more than the gate
in §6.2, (c) fall back to the last KF-predicted scale when AR is not viable.

### 2.3 Lateral and Vertical Back-Projection

    X = (u − c_x) · Z / f_px
    Y = (v − c_y) · Z / f_px

Error in Z propagates multiplicatively into X and Y; this is accepted and
compensated downstream by the Kalman filter.

---

## 3. State Estimation: 9-State Kinematic Kalman Filter

### 3.1 State Vector

    x = [ x  y  s  v_x  v_y  v_s  a_x  a_y  a_s ]ᵀ

where (x, y) are the bounding-box centroid in pixel space and s = √(w·h)
is the box scale in pixels. All three position components are in pixel
space so that the observation model is linear and time-invariant.

### 3.2 Transition Model

Constant-acceleration Newtonian propagation with time step Δt:

    F = blockdiag(F_axis, F_axis, F_axis)

    F_axis = [ 1  Δt  Δt²/2 ]
             [ 0   1   Δt   ]
             [ 0   0    1   ]

    x_{k|k-1} = F · x_{k-1|k-1}

### 3.3 Process Noise (Jerk-Driven)

The only exogenous input to a constant-acceleration model is jerk. For each
axis:

    G_axis = [ Δt³/6 ]
             [ Δt²/2 ]
             [  Δt   ]

    Q_axis = G_axis · σ_j² · G_axisᵀ        (3×3)

    Q = blockdiag(Q_x, Q_y, Q_s)

**Units.** σ_j is jerk in px/s³, so σ_j² has units px²/s⁶. Substituting
into G_axis (units [s³, s², s]) gives position-position entries of Q_axis
equal to Δt⁶σ_j²/36, with units px²·s⁶/s⁶ = px². Velocity-velocity entries
have units px²/s², and acceleration-acceleration entries have units px²/s⁴.
All consistent with the mixed state vector.

**Tuning.** Nominal values for a small quadrotor at 30 Hz:

    σ_j,x² = σ_j,y² = 3.0 × 10⁵ px²/s⁶   (σ_j ≈ 550 px/s³)
    σ_j,s²           = 5.0 × 10⁴ px²/s⁶   (σ_j ≈ 220 px/s³)

These reflect the target's and drone's combined jerk in pixel space. The
block-diagonal Q_axis construction scales automatically with Δt; no
retuning is required for framerate changes.

### 3.4 Observation Model

The CNN detector provides only position and scale:

    z_k = [ u  v  s_meas ]ᵀ

    H = [ 1 0 0 0 0 0 0 0 0 ]
        [ 0 1 0 0 0 0 0 0 0 ]
        [ 0 0 1 0 0 0 0 0 0 ]

### 3.5 Measurement Noise (Scale-Adaptive)

Baseline noise from CNN centroid jitter (≈2 px position, 3 px scale, 1σ):

    R_base = diag(4, 4, 9)

Scale-adaptive inflation for depth ambiguity (small s → large uncertainty),
clamped to prevent the filter from becoming over-confident on very large
boxes:

    ρ_raw = 100 / max(10, s_meas)
    ρ     = clamp(ρ_raw, 0.5, 10)
    R_k   = R_base · ρ²

**Interpretation.** At s_meas = 100 px the inflation is unity and the
filter trusts the detector as much as R_base allows. As s_meas falls toward
10 px, ρ rises to 10 and R is inflated 100×, forcing K → 0 and letting the
kinematic prediction coast through the depth-ambiguous regime. At large
s_meas the clamp at ρ = 0.5 prevents unphysical confidence (R_k ≥ 0.25·R_base).

### 3.6 Latency Compensation

Total vision-to-control latency is τ ≈ 100 ms. The measurement z_k was
captured at wall-clock time t_capture = t_k − τ_arrival, where τ_arrival is
the transport delay from camera shutter to filter input. Control is computed
at wall-clock time t_ctrl, typically one control-loop period after the
filter update.

**(a) Timestamp the measurement at capture.** Store t_capture alongside
z_k. Use Δt = t_capture − t_{k−1} in the transition matrix for that update.

**(b) Propagate the posterior to the control instant before computing
errors.** After the filter update, compute:

    Δτ = t_ctrl − t_capture
    x_ctrl = F(Δτ) · x_{k|k}
    P_ctrl = F(Δτ) · P_{k|k} · F(Δτ)ᵀ + Q(Δτ)

Here Δτ is the elapsed time from when the pixels were exposed to when the
control command will be transmitted — not the loop period and not the full
transport delay. At 30 Hz control with τ_arrival ≈ 100 ms, Δτ ≈ 130 ms.

### 3.7 Filter Equations

**Predict:**
    x_{k|k-1} = F · x_{k-1|k-1}
    P_{k|k-1} = F · P_{k-1|k-1} · Fᵀ + Q

**Update (Joseph form for numerical stability):**
    S = H · P_{k|k-1} · Hᵀ + R_k
    K = P_{k|k-1} · Hᵀ · S⁻¹
    x_{k|k} = x_{k|k-1} + K · (z_k − H · x_{k|k-1})
    P_{k|k} = (I − K·H) · P_{k|k-1} · (I − K·H)ᵀ + K · R_k · Kᵀ

**Initialization.** P_0 = diag(100, 100, 100, 10⁴, 10⁴, 10⁴, 10⁶, 10⁶, 10⁶)
reflects high initial uncertainty. The filter converges within ~5 frames
given a valid track.

### 3.8 Track Management

- **Coast window:** N_coast = 15 frames (~0.5 s). Predict, skip update.
- **Loss declaration:** N_lost = 45 frames (~1.5 s). Drop track, re-search.
- **Gating:** Reject z_k whose Mahalanobis distance
  (z_k − H·x_{k|k-1})ᵀ · S⁻¹ · (z_k − H·x_{k|k-1}) exceeds χ²₃(0.99) ≈ 11.34.

---

## 4. Kinematic Control Law

### 4.1 Error Functions

    e_x = x̂_ctrl − c_x
    e_y = ŷ_ctrl − c_y
    e_z = Z_filtered − Z_ref

The pixel errors (e_x, e_y) are pre-compensation; §4.2 rotates them into the
level body frame.

### 4.2 Optical Attitude Compensation

The camera is body-fixed. Roll (rotation about the optical z-axis) rotates
the image plane; pitch (rotation about the optical x-axis) shifts the image
vertically.

**Step 1 — Roll rotation (2D).** Applying the inverse roll about the optical
axis:

    e_x' = e_x · cos φ − e_y · sin φ
    e_y' = e_x · sin φ + e_y · cos φ

This is a true 2D rotation, valid at arbitrary roll angle. It correctly
couples the two axes: a target at nonzero e_y is displaced in e_x by roll,
and vice versa.

**Step 2 — Pitch shift.** With the rolled errors in hand, apply pitch as a
vertical projection:

    e_x,level = e_x'
    e_y,level = f_px · tan( arctan(e_y' / f_px) − θ )

The pitch step is exact for pure pitch and first-order accurate for combined
roll-pitch at moderate angles.

**Approximation for small angles.** For |φ| < 15° and |θ| < 15°:

    e_x,level ≈ e_x − e_y · φ
    e_y,level ≈ e_x · φ + e_y − f_px · θ

The small-angle form is sufficient for typical tracking where the inner
loop holds roll and pitch within a few degrees.

**Validity bound.** Beyond |φ|, |θ| > 45° the combined-rotation approximation
degrades. For those regimes use the full rotation matrix R_body_optical and
reproject analytically.

### 4.3 PD Control Signals

**Yaw Rate (rad/s):**

    V_yaw = K_P,ψ · e_x,level + K_D,ψ · v̂_x

    K_P,ψ = 0.0090 rad/s per px
    K_D,ψ = 0.0020 rad per px

**Altitude Command (m/s):**

    V_alt = K_P,alt · e_y,level + K_D,alt · v̂_y

    K_P,alt = 0.0060 m/s per px
    K_D,alt = 0.0020 m per px

**Forward Approach (m/s):**

Depth Z is not a Kalman state; its rate is obtained by EMA-smoothed finite
difference of the metric error:

    ė_z         = (e_z,k − e_z,k−1) / Δt
    ė̃_z         = α · ė_z + (1 − α) · ė̃_z,k−1        (α = 0.20)
    V_fwd       = K_P,fwd · e_z + K_D,fwd · ė̃_z

    K_P,fwd = 0.40 (m/s) per m
    K_D,fwd = 0.05 (m/s) per (m/s)

**Gain unit interpretation.** Pixel-error gains implicitly carry the
pixel→rad and pixel→meter conversions through f_px and Z; the numeric
values were tuned at 800 px focal length and remain valid as long as f_px
does not change by more than ~25%. Re-tune after any camera or resolution
change.

**Why no integral term.** Steady-state errors are suppressed by the
autopilot's velocity outer loop, and integral action without anti-windup
destabilizes under saturation. If integral action is later added, clamp the
integrator and use back-calculation anti-windup.

### 4.4 Saturation

Applied to the final blended command (§5.5), before MAVLink transmission:

    |V_yaw| ≤ 2.0 rad/s
    |V_alt| ≤ 3.0 m/s
    |V_fwd| ≤ 10.0 m/s

---

## 5. Terminal Interception

### 5.1 Handoff Schedule

    w(Z) = 1                                  if Z ≤ Z_commit
         = (Z_high − Z)/(Z_high − Z_commit)   if Z_commit < Z < Z_high
         = 0                                  if Z ≥ Z_high

    Z_high = 5.0 m,   Z_commit = 3.5 m

w = 0 → pure PD; w = 1 → pure ProNav; linear in between.

### 5.2 Line-of-Sight Rate

The LOS rate is a purely geometric quantity in the image plane. From the
pinhole projection with the Kalman horizontal centroid x̂ and its rate v̂_x:

    λ̇_image,h = (f_px · v̂_x) / (f_px² + x̂²)

Similarly for the vertical axis with ŷ, v̂_y:

    λ̇_image,v = (f_px · v̂_y) / (f_px² + ŷ²)

**Derivation.** The target is at world (X, Z) with X = x̂·Z/f_px. LOS
angle λ = arctan(X/Z), so λ̇ = (Ẋ·Z − X·Ż)/(X² + Z²). Substituting
Ẋ = (v̂_x·Z + x̂·Ż)/f_px and simplifying, the Ż terms cancel identically
and λ̇ = f_px·v̂_x/(f_px² + x̂²). Units: (px·px/s)/(px²) = rad/s. ∎

**Yaw compensation.** The camera is body-fixed. If the drone yaws at ψ̇,
the inertial LOS rate is:

    λ̇_h = λ̇_image,h + ψ̇
    λ̇_v = λ̇_image,v

Yaw compensation applies to the horizontal axis only. (The drone does not
roll or pitch fast enough during terminal phase for their optical
contributions to matter; if desired, add φ̇ and θ̇ terms with the same
sign convention as the attitude compensation in §4.2.)

### 5.3 Dynamic Closing Velocity

Closing velocity is the negative range rate, computed from the filtered
inverse-depth signal:

    V_c = − Ż_filtered

**Sign convention and clamp.** If the target flees faster than the drone
closes, Ż > 0 and V_c < 0 would reverse the ProNav command sign. Clamp:

    V_c = max(0, − Ż_filtered)

If V_c remains 0 for more than 2 s during terminal phase, the intercept is
infeasible; abort terminal and return to PD pursuit.

### 5.4 Per-Axis ProNav Acceleration

The classical PN law produces one acceleration command per transverse axis:

    a_cmd,h = N · V_c · λ̇_h            (m/s², lateral/altitude)
    a_cmd,v = N · V_c · λ̇_v            (m/s², altitude)

with N = 4.0.

**Velocity formulation.** The PD controllers of §4 output velocities, so
before blending the ProNav output is converted to a velocity increment:

    V_ProNav,yaw = V_cmd,prev,yaw + a_cmd,h · Δt_loop
    V_ProNav,alt = V_cmd,prev,alt + a_cmd,v · Δt_loop
    V_ProNav,fwd = V_fwd|_{Z = Z_high}    (hold constant during terminal)

`V_fwd|_{Z=Z_high}` is the forward-velocity command latched at the moment
of handoff. Holding it constant prevents the terminal loop from attempting
to modulate closing velocity, which would destabilize the PN geometry.

`V_cmd,prev,*` is the previous loop's transmitted command. Seeding the
increment with the prior command guarantees continuity at the handoff
instant.

### 5.5 Blended Command

For each axis:

    V_yaw,cmd = (1 − w) · V_yaw,PD + w · V_ProNav,yaw
    V_alt,cmd = (1 − w) · V_alt,PD + w · V_ProNav,alt
    V_fwd,cmd = (1 − w) · V_fwd,PD + w · V_ProNav,fwd

Then §4.4 saturation is applied. The result is transmitted via MAVLink
`SET_POSITION_TARGET_LOCAL_NED` with the velocity-only type mask.

**Continuity.** w(Z) is continuous in Z, Z is continuous in time, and
V_ProNav,yaw and V_ProNav,alt are seeded with the previous transmitted
command. The blended command is therefore continuous across both the
upper bound Z_high and the lower bound Z_commit. Its time derivative has a
single kink at each boundary but remains bounded — no attitude snap.

---

## 6. Outlier Rejection

### 6.1 Inverse-Depth Median Filter

Operates on Z⁻¹ because depth errors scale quadratically with range while
inverse-depth errors scale linearly:

    Z_filtered = [ M( Z_k⁻¹, Z_{k−1}⁻¹, …, Z_{k−n+1}⁻¹ ) ]⁻¹      (n = 5)

A window of 5 samples at 30 Hz gives 167 ms of smoothing — enough to reject
single-frame depth spikes without introducing perceptible lag.

### 6.2 Aspect-Ratio Gating

Maintains a 30-frame rolling median AR_median:

    AR_k = w_px / h_px

Reject the detection if:

    AR_k > 2.5 · AR_median   OR   AR_k < AR_median / 2.5

**Cold-start policy.** For the first 30 frames after track acquisition, the
AR gate is disabled and the Mahalanobis gate in §3.8 is the sole line of
defense. This trades initial robustness for the ability to acquire a track.

**Rationale.** A partial detection (e.g., a single rotor arm) produces a
bounding box with approximately 2× the nominal AR. The 2.5× threshold
rejects these while allowing normal AR variation from target yaw.

---

## 7. Failure Modes and Policy

| Condition                        | Detection                        | Response                              |
|----------------------------------|----------------------------------|---------------------------------------|
| Detector dropout ≤ 15 frames     | No z_k, coast window active      | KF predicts; no update; no control refresh |
| Detector dropout > 45 frames     | No z_k, coast window expired     | Track dropped; re-enter search        |
| Mahalanobis gate exceeded        | d² > χ²₃(0.99) = 11.34           | Reject z_k; coast                     |
| AR gate exceeded                 | AR ratio > 2.5× from median      | Reject z_k; coast                     |
| Target out of frame              | x̂_ctrl or ŷ_ctrl outside [0, W_frame]×[0, H_frame] | Command yaw toward last in-frame detection |
| V_c = 0 for > 2 s at Z < 6 m     | Range rate non-negative          | Abort terminal; return to PD pursuit  |
| Command saturated > 200 ms       | Setpoint equals clamp            | Log event; investigate gain recalibration |

---

## 8. Stability and Tuning Notes

### 8.1 Linearized Yaw Loop

For the yaw axis, model the autopilot's inner velocity loop as a
first-order lag with time constant τ_v. A stationary target at world
azimuth ψ_T, drone yaw ψ, gives pixel error e_x ≈ f_px·(ψ_T − ψ) and
Kalman rate v̂_x ≈ −f_px·ψ̇. Substituting into §4.3:

    τ_v · ë_x + (1 + K_D,ψ · f_px) · ė_x + K_P,ψ · f_px · e_x = 0

**Note.** The Z-dependence that appeared in earlier drafts of this document
was an error. Image-plane feedback does not introduce a range factor into
the yaw loop; the pixel error is a direct measure of body-frame azimuth,
independent of range.

Standard second-order form:

    ω_n = sqrt( K_P,ψ · f_px / τ_v )
    ζ   = (1 + K_D,ψ · f_px) / (2 · τ_v · ω_n)

At the nominal operating point f_px = 800 px, τ_v = 0.15 s, and the gains
from §4.3:

    ω_n = sqrt( 0.0090 · 800 / 0.15 ) ≈ 6.9 rad/s   (≈ 1.1 Hz)
    ζ   = (1 + 0.0020 · 800) / (2 · 0.15 · 6.9) ≈ 1.25

This is a stable, mildly overdamped closed loop with roughly 1 Hz bandwidth
and no overshoot. The damping margin (ζ > 1) provides robustness against
model error in τ_v and against the lag introduced by the detector.

### 8.2 Altitude Loop

The altitude loop is structurally identical to yaw, with the vertical pixel
error e_y,level replacing e_x,level and V_alt replacing V_yaw. The gain
analysis from §8.1 carries over with K_P,alt, K_D,alt:

    ω_n = sqrt( 0.0060 · 800 / 0.15 ) ≈ 5.7 rad/s
    ζ   = (1 + 0.0020 · 800) / (2 · 0.15 · 5.7) ≈ 1.52

Also stable and overdamped.

### 8.3 Forward Loop

The forward loop is slower by construction (K_P,fwd = 0.40 m/s per m,
first-order with EMA-derivative damping). Closed-loop bandwidth is on the
order of α/Δt ≈ 6 rad/s before the EMA lag is accounted for, and the
dominant lag is the EMA filter itself. No aggressive tuning is needed since
the loop is only active until Z reaches Z_commit.

### 8.4 ProNav Stability

Classical PN with N ≥ 2 is asymptotically stable in the homing phase
against maneuvering targets, with miss distance bounded by the target's
lateral acceleration bandwidth. N = 4.0 is conservative; higher N increases
responsiveness at the cost of command authority near the commit point. The
blend in §5.1 restricts ProNav activity to the terminal band where LOS
rate is well-estimated.

### 8.5 Latency Margin

Total loop latency budget:

| Stage                                | Delay (ms) |
|--------------------------------------|------------|
| Camera exposure + readout            | 15         |
| CNN inference on companion computer  | 40         |
| Projection + filter update           | 5          |
| Control computation + MAVLink TX     | 10         |
| Autopilot RX + inner-loop response   | 30         |
| **Total**                            | **≈ 100**  |

§3.6 compensates the state for this delay before computing errors. At 5 m/s
closing, uncompensated latency causes 0.5 m of positional lag; propagation
by Δτ removes most of it, leaving residual error set by uncertainty in τ.

### 8.6 Unmodeled Dynamics

Accepted as residual error, handled by increasing σ_j² or R_base if
dominant:

- Actuator nonlinearity (ESC deadband, prop wash)
- Aerodynamic drag at high lateral velocity
- Rolling-shutter skew during fast yaw
- Target acceleration above the modeled jerk variance

---

## 9. Summary of Known Limitations

1. **Monocular depth ambiguity.** Z is trustworthy only when the target's
   apparent aspect matches its nominal aspect. Heavy yaw biases Z upward.
2. **CNN centroid noise.** R_base assumes 2 px / 3 px jitter, valid for a
   well-trained detector on clear targets; degrades under occlusion or
   motion blur.
3. **Constant-acceleration target model.** Evasive maneuvers with jerk
   above σ_j produce filter lag; the ProNav phase is designed to close the
   loop on this by directly nulling LOS rate.
4. **Latency τ assumed constant.** Real systems jitter by ±20 ms; residual
   positional error is small but nonzero.
5. **No obstacle avoidance.** Model assumes clear line of sight from launch
   to intercept.
6. **No collision-avoidance at impact.** ProNav drives to intercept. If
   capture (net, gripper) rather than kinetic impact is intended, the
   terminal phase must be modified.
7. **Attitude compensation valid for moderate angles.** Beyond |φ|,|θ| > 45°
   use the full rotation matrix rather than the small-angle coupling form.

---

*End of model specification.*
