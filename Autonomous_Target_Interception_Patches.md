# Autonomous Target Interception: Mathematical Patches & Revised Specifications

This document formalizes the mathematical amendments to the baseline Autonomous Target Interception specification. These patches harden the system against evasive maneuvers, depth estimation artifacts, and terminal phase guidance instabilities.

---

## Patch 1: Yaw-Consistency Downgrade (Update to §3.5 & §6.2)

To mitigate "yaw collapse" where the bounding box area shrinks during target yaw (falsely inflating range $Z$), we introduce a continuous measurement covariance penalty based on aspect ratio deviation.

**Aspect Ratio Deviation:**
$$\Delta_{AR} = \max \left( \frac{AR_k}{AR_{median}}, \frac{AR_{median}}{AR_k} \right)$$

**Scale-Adaptive Inflation Penalty:**
The baseline noise matrix $\mathbf{R}_k$ is inflated strictly in the scale (depth) channel based on the yaw penalty factor $\rho_{yaw}$. 
$$ \rho_{yaw} = \begin{cases} 
      1 & \text{if } \Delta_{AR} \leq 1.15 \\
      1 + \kappa (\Delta_{AR} - 1.15)^2 & \text{if } 1.15 < \Delta_{AR} \leq 2.5 \\
      \infty \text{ (Reject)} & \text{if } \Delta_{AR} > 2.5 
\end{cases} $$
Where $\kappa \approx 50$ is the penalty weight.

**Application:**
$$R_{k,(3,3)} = R_{base,(3,3)} \cdot \rho^2 \cdot \rho_{yaw}^2$$

**Cold-Start Policy:** The yaw-consistency downgrade is strictly disabled during the 30-frame cold-start window to guarantee initial track acquisition.

---

## Patch 2: Interacting Multiple Model (IMM) Upgrade (Update to §3.3)

To drastically reduce the coast time when tracking a target executing high-jerk maneuvers, the single Constant-Acceleration (CA) filter is upgraded to a 9-state same-dimension Interacting Multiple Model (IMM) filter.

**Models and Parameters:**
1.  **$M_1$ (Constant Velocity - CV):** Process noise $\mathbf{Q}_{CV}$ uses a near-zero jerk variance ($\sigma_{j, CV}^2 = 10^1$).
2.  **$M_2$ (Constant Acceleration - CA):** Process noise $\mathbf{Q}_{CA}$ uses nominal jerk variance ($\sigma_{j, CA}^2 = 3 \times 10^5$).
3.  **Transition Matrix ($\Pi$):** $\Pi = \begin{bmatrix} 0.90 & 0.10 \\ 0.10 & 0.90 \end{bmatrix}$

**Step 1: Mixing**
Mixing probabilities:
$$ \mu_{i|j}(k-1) = \frac{1}{\bar{c}_j} \pi_{ij} \mu_i(k-1), \quad \text{where } \bar{c}_j = \sum_i \pi_{ij} \mu_i(k-1) $$

Mixed initial state and covariance (for model $j$):
$$ \hat{\mathbf{x}}_{0j}(k-1|k-1) = \sum_i \mu_{i|j}(k-1) \hat{\mathbf{x}}_i(k-1|k-1) $$
$$ \mathbf{P}_{0j}(k-1|k-1) = \sum_i \mu_{i|j}(k-1) \left[ \mathbf{P}_i + (\hat{\mathbf{x}}_i - \hat{\mathbf{x}}_{0j})(\hat{\mathbf{x}}_i - \hat{\mathbf{x}}_{0j})^T \right] $$
*(Where $\mathbf{P}_i$ and $\hat{\mathbf{x}}_i$ are at $k-1|k-1$).*

**Step 2: Model-Conditioned Filtering**
Using standard Kalman predict/update equations initialized with $\hat{\mathbf{x}}_{0j}$ and $\mathbf{P}_{0j}$, each model computes $\hat{\mathbf{x}}_{j, k|k}$ and $\mathbf{P}_{j, k|k}$.

**Step 3: Likelihood and Probability Update**
$$ \Lambda_j(k) = (2\pi)^{-3/2} |\mathbf{S}_j|^{-1/2} \exp \left( -\frac{1}{2} \boldsymbol{\nu}^T \mathbf{S}_j^{-1} \boldsymbol{\nu} \right) $$
$$ \mu_j(k) = \frac{\Lambda_j(k) \bar{c}_j}{\sum_i \Lambda_i(k) \bar{c}_i} $$

**Step 4: Output Combination**
Combined state:
$$\hat{\mathbf{x}}_{k|k} = \sum_j \mu_j(k) \hat{\mathbf{x}}_{j, k|k}$$

Combined posterior covariance:
$$ \mathbf{P}_{k|k} = \sum_j \mu_j(k) \left[ \mathbf{P}_{j, k|k} + (\hat{\mathbf{x}}_{j, k|k} - \hat{\mathbf{x}}_{k|k})(\hat{\mathbf{x}}_{j, k|k} - \hat{\mathbf{x}}_{k|k})^T \right] $$

---

## Patch 3: ProNav Freeze-Yaw and Direction Consistency (Update to §5.5)

To prevent the PD centering loop from fighting the ProNav lead-angle generation during the terminal phase, the yaw channels are mathematically decoupled and smoothly blended at the transition boundary.

**Freeze-Yaw Feedforward & Blended Command:**
During the blend zone ($Z_{commit} < Z < Z_{high}$), the commanded yaw smoothly interpolates between the standard PD law and the feedforward ProNav tracking term using the blend weight $w(Z)$:
$$V_{yaw, cmd} = (1 - w(Z)) \cdot (K_{P, \psi} e_{x, level} + K_{D, \psi} \hat{v}_x) + w(Z) \cdot (\dot{\lambda}_h + K_{cam, ff}(Z) \cdot e_{x, level})$$
$$ K_{cam, ff}(Z) = 0.002 \cdot \frac{Z - Z_{commit}}{Z_{high} - Z_{commit}} $$

**Direction Consistency Check & Hysteresis:**
The system constantly checks if the ProNav geometry is broken:
$$\text{Condition: } \text{sgn}(\dot{\lambda}_h) \cdot \text{sgn}(V_{ProNav, yaw}) < 0$$

*   **Hysteresis Entry (Abort):** If the condition persists for 5 consecutive frames, clamp $w=0$ (abort ProNav, revert to pure PD).
*   **Hysteresis Exit (Re-commit):** If the signs match for 10 consecutive frames AND $Z > 1.5$m, resume the blend $w(Z)$.

---

## Patch 4: Explicit Full-Rotation Matrix (Update to §4.2)

To eliminate truncation errors from the small-angle approximation at high bank/pitch angles ($>45^\circ$), we implement an exact 3D vector rotation.

**Optical Frame Projection Matrix:**
For a given roll $\phi$ (around Body X/Optical Z) and pitch $\theta$ (around Body Y/Optical X), the projection matrix $\mathbf{R}$ is:
$$ \mathbf{R} = 
\begin{bmatrix}
\cos\phi & -\sin\phi & 0 \\
\sin\phi \cos\theta & \cos\phi \cos\theta & -\sin\theta \\
\sin\phi \sin\theta & \cos\phi \sin\theta & \cos\theta
\end{bmatrix}
$$

Let the normalized pixel vector in the optical frame be $\mathbf{v}_{opt} = [e_x, e_y, f_{px}]^T$.
We compute the leveled vector: $\mathbf{v}_{level} = \mathbf{R} \cdot \mathbf{v}_{opt}$.

**Exact Level-Frame Pixel Errors:**
$$e_{x,level} = f_{px} \frac{\mathbf{v}_{level, 1}}{\mathbf{v}_{level, 3}}$$
$$e_{y,level} = f_{px} \frac{\mathbf{v}_{level, 2}}{\mathbf{v}_{level, 3}}$$

*(Note: Yaw compensation is mathematically distinct and remains handled downstream via $\dot{\lambda}_{inertial} = \dot{\lambda}_{image} + \dot{\psi}$ as specified in §5.2. There is no conflict or double-counting).*
