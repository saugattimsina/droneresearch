# Autonomous Target Interception: Final Mathematical Specification

This document specifies the formal mathematical framework for vision-based target extraction, state estimation, and kinematic control of an autonomous interception drone. All equations are dimensionally consistent and mathematically hardened against evasive maneuvers, depth estimation artifacts, and terminal phase instabilities.

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
- Roll  $\phi$: positive right-wing-down (rotation about body x = optical z)
- Pitch $\theta$: positive nose-up (rotation about body y = optical x)
- Yaw   $\psi$: positive clockwise viewed from above (rotation about body z = optical y)

All control loops operate in the **level body FRD frame**.

---

## 2. Vision-to-Spatial Projection

### 2.1 Focal Length
$$f_{px} = \frac{W_{frame} / 2}{\tan(\theta_{HFOV} / 2)}$$
*Calibration note:* On hardware, $f_{px}$ and the principal point $(c_x, c_y)$ must be obtained from intrinsic calibration, not nominal HFOV.

### 2.2 Depth from Bounding Box
For a target with known physical width $W_{target}$ and height $H_{target}$, depth is estimated using the area-based formulation:
$$Z = \sqrt{ \frac{f_{px}^2 \cdot W_{target} \cdot H_{target}}{w_{px} \cdot h_{px}} }$$
*Note:* Under heavy yaw, bounding box area shrinks, biasing $Z$ upward. This is mitigated downstream via adaptive measurement covariance penalties (§3.4).

### 2.3 Lateral and Vertical Back-Projection
$$X = \frac{(u - c_x) \cdot Z}{f_{px}}$$
$$Y = \frac{(v - c_y) \cdot Z}{f_{px}}$$

---

## 3. State Estimation: 9-State Interacting Multiple Model (IMM) Filter

To prevent coasting during bang-bang maneuvers, the tracking filter is an Interacting Multiple Model (IMM) running two parallel kinematic hypotheses.

### 3.1 State Vector
$$\mathbf{x} = [ x, y, s, v_x, v_y, v_s, a_x, a_y, a_s ]^T$$
where $(x, y)$ are the bounding-box centroid in pixel space and $s = \sqrt{w \cdot h}$ is the box scale in pixels.

### 3.2 IMM Models and Transition
Both models use the same state space and transition matrix $\mathbf{F}$ (Newtonian constant-acceleration with time step $\Delta t$).
1.  **$M_1$ (Constant Velocity - CV):** Process noise $\mathbf{Q}_{CV}$ uses a near-zero jerk variance ($\sigma_{j, CV}^2 = 10^1$), naturally decaying acceleration.
2.  **$M_2$ (Constant Acceleration - CA):** Process noise $\mathbf{Q}_{CA}$ uses nominal jerk variance ($\sigma_{j, CA}^2 = 3 \times 10^5$ for X/Y, $5 \times 10^4$ for scale).

The Markov transition matrix for the models is:
$$ \Pi = \begin{bmatrix} 0.90 & 0.10 \\ 0.10 & 0.90 \end{bmatrix} $$

### 3.3 IMM Step 1: Mixing
Mixing probabilities:
$$ \mu_{i|j}(k-1) = \frac{1}{\bar{c}_j} \pi_{ij} \mu_i(k-1), \quad \text{where } \bar{c}_j = \sum_i \pi_{ij} \mu_i(k-1) $$

Mixed initial state and covariance (for model $j$):
$$ \hat{\mathbf{x}}_{0j}(k-1|k-1) = \sum_i \mu_{i|j}(k-1) \hat{\mathbf{x}}_i(k-1|k-1) $$
$$ \mathbf{P}_{0j}(k-1|k-1) = \sum_i \mu_{i|j}(k-1) \left[ \mathbf{P}_i + (\hat{\mathbf{x}}_i - \hat{\mathbf{x}}_{0j})(\hat{\mathbf{x}}_i - \hat{\mathbf{x}}_{0j})^T \right] $$

### 3.4 Measurement Model & Yaw-Consistency Downgrade
The measurement vector is $\mathbf{z}_k = [u, v, s_{meas}]^T$. Baseline noise $\mathbf{R}_{base} = \text{diag}(4, 4, 9)$.

To mitigate depth ambiguity, we compute the aspect ratio deviation:
$$\Delta_{AR} = \max \left( \frac{AR_k}{AR_{median}}, \frac{AR_{median}}{AR_k} \right)$$
We inflate the scale measurement variance via a scale factor $\rho = \text{clamp}(100 / \max(10, s_{meas}), 0.5, 10)$ and a yaw-penalty factor $\rho_{yaw}$:
$$ \rho_{yaw} = \begin{cases} 
      1 & \text{if } \Delta_{AR} \leq 1.15 \\
      1 + 50 (\Delta_{AR} - 1.15)^2 & \text{if } 1.15 < \Delta_{AR} \leq 2.5 \\
      \infty \text{ (Reject)} & \text{if } \Delta_{AR} > 2.5 
\end{cases} $$
*(Note: $\rho_{yaw}$ is disabled during the first 30 frames for cold-start).*

The final measurement noise covariance is:
$$\mathbf{R}_k = \text{diag}(R_{base, (1,1)}, R_{base, (2,2)}, R_{base, (3,3)} \cdot \rho^2 \cdot \rho_{yaw}^2)$$

### 3.5 IMM Step 2-4: Filter, Likelihood, and Combination
Each model performs a standard Kalman update using $\mathbf{z}_k$ and $\mathbf{R}_k$ to compute $\hat{\mathbf{x}}_{j, k|k}$ and $\mathbf{P}_{j, k|k}$.

Likelihood and mode probability update:
$$ \Lambda_j(k) = (2\pi)^{-3/2} |\mathbf{S}_j|^{-1/2} \exp \left( -\frac{1}{2} \boldsymbol{\nu}^T \mathbf{S}_j^{-1} \boldsymbol{\nu} \right) $$
$$ \mu_j(k) = \frac{\Lambda_j(k) \bar{c}_j}{\sum_i \Lambda_i(k) \bar{c}_i} $$

Combined output state and covariance:
$$\hat{\mathbf{x}}_{k|k} = \sum_j \mu_j(k) \hat{\mathbf{x}}_{j, k|k}$$
$$ \mathbf{P}_{k|k} = \sum_j \mu_j(k) \left[ \mathbf{P}_{j, k|k} + (\hat{\mathbf{x}}_{j, k|k} - \hat{\mathbf{x}}_{k|k})(\hat{\mathbf{x}}_{j, k|k} - \hat{\mathbf{x}}_{k|k})^T \right] $$

*(Latency compensation propagates $\hat{\mathbf{x}}_{k|k}$ forward by $\Delta\tau = t_{ctrl} - t_{capture}$ prior to extracting control errors).*

---

## 4. Kinematic Control Law

### 4.1 Exact Optical Attitude Compensation
The camera is body-fixed. To project the optical vector $\mathbf{v}_{opt} = [e_x, e_y, f_{px}]^T$ into the stabilized level frame regardless of extreme bank/pitch, we apply the exact 3D rotation matrix $\mathbf{R}$ (Roll $\phi$ about Body X/Optical Z; Pitch $\theta$ about Body Y/Optical X):
$$ \mathbf{R} = 
\begin{bmatrix}
\cos\phi & -\sin\phi & 0 \\
\sin\phi \cos\theta & \cos\phi \cos\theta & -\sin\theta \\
\sin\phi \sin\theta & \cos\phi \sin\theta & \cos\theta
\end{bmatrix}
$$
$$\mathbf{v}_{level} = \mathbf{R} \cdot \mathbf{v}_{opt}$$
$$e_{x,level} = f_{px} \frac{\mathbf{v}_{level, 1}}{\mathbf{v}_{level, 3}}, \quad e_{y,level} = f_{px} \frac{\mathbf{v}_{level, 2}}{\mathbf{v}_{level, 3}}$$

### 4.2 PD Pursuit Control Signals
**Altitude and Yaw (PD):**
$$V_{yaw, PD} = K_{P,\psi} \cdot e_{x,level} + K_{D,\psi} \cdot \hat{v}_x$$
$$V_{alt, PD} = K_{P,alt} \cdot e_{y,level} + K_{D,alt} \cdot \hat{v}_y$$
**Forward Approach:**
$$V_{fwd, PD} = K_{P,fwd} \cdot (Z_{filtered} - Z_{ref}) + K_{D,fwd} \cdot \tilde{\dot{e}}_z$$

---

## 5. Terminal Interception

### 5.1 Line-of-Sight Rate & ProNav
When entering terminal phase ($Z < 5.0$m), we calculate LOS rates and closing velocity $V_c = \max(0, -\dot{Z}_{filtered})$.
$$\dot{\lambda}_{image, h} = \frac{f_{px} \cdot \hat{v}_x}{f_{px}^2 + \hat{x}^2}, \quad \dot{\lambda}_{image, v} = \frac{f_{px} \cdot \hat{v}_y}{f_{px}^2 + \hat{y}^2}$$
Inertial yaw compensation: $\dot{\lambda}_h = \dot{\lambda}_{image, h} + \dot{\psi}$.

ProNav acceleration commands:
$$a_{cmd,h} = N \cdot V_c \cdot \dot{\lambda}_h, \quad a_{cmd,v} = N \cdot V_c \cdot \dot{\lambda}_{image, v}$$
Converted to velocity increments:
$$V_{ProNav, yaw} = V_{cmd, prev, yaw} + a_{cmd, h} \Delta t$$
$$V_{ProNav, alt} = V_{cmd, prev, alt} + a_{cmd, v} \Delta t$$

### 5.2 Blended Command and Freeze-Yaw
We define a blend weight $w(Z)$ linearly interpolating from 0 (at $Z_{high} = 5.0$m) to 1 (at $Z_{commit} = 3.5$m).
To prevent PD centering from fighting ProNav lead angle, the yaw command smoothly interpolates to a feedforward tracking term:
$$V_{yaw, cmd} = (1 - w(Z)) \cdot V_{yaw, PD} + w(Z) \cdot \left(\dot{\lambda}_h + K_{cam, ff}(Z) \cdot e_{x, level}\right)$$
Where the re-centering gain decays to zero at commit: $K_{cam, ff}(Z) = 0.002 \cdot \frac{Z - Z_{commit}}{Z_{high} - Z_{commit}}$.
*(The altitude and forward axes blend identically using standard interpolation).*

### 5.3 Direction Consistency & Hysteresis
If the ProNav tracking breaks geometry, the commands will fight:
$$\text{Condition: } \text{sgn}(\dot{\lambda}_h) \cdot \text{sgn}(V_{ProNav, yaw}) < 0$$
*   **Entry (Abort):** If persistent for 5 frames, clamp $w=0$ (revert to pure PD pursuit).
*   **Exit (Re-commit):** If signs match for 10 frames and $Z > 1.5$m, resume blending.

---

## 6. Track Management and Gating Policies
1.  **Inverse-Depth Median:** Operates on $Z^{-1}$ over 5 samples to reject spikes.
2.  **Mahalanobis Gating:** Rejects measurements where $\boldsymbol{\nu}^T \mathbf{S}_j^{-1} \boldsymbol{\nu} > 11.34$.
3.  **Coast Window:** If gated, IMM predicts without update for up to 15 frames. Drop track at 45 frames.
4.  **Aspect-Ratio Gating:** Enforced via $\rho_{yaw} = \infty$ for $\Delta_{AR} > 2.5$.
5.  **Saturation:** Final commands clamped before MAVLink transmission ($V_{yaw} \leq 2.0$, $V_{alt} \leq 3.0$, $V_{fwd} \leq 10.0$).

---
*End of model specification.*
