# Complete System Mathematics: Line-by-Line Guide
This is the complete, unfiltered mathematical pipeline. You can use these exact formulas in Excel, Python, or MATLAB to test raw YOLO data ($u, v, width, height$) across multiple frames.

---

## Phase 1: Camera Setup (Run Once)
Before processing any frames, establish the intrinsic properties of the camera inside the letterboxed 480x480 frame.

**1. Known Constants:**
* `FRAME_WIDTH` = 480
* `FRAME_HEIGHT` = 480
* `W` (Target Physical Width) = 0.35 meters
* `c_x` (Image Center X) = 240
* `c_y` (Image Center Y) = 240
* `HFOV` = 102° (Assuming the 120° AF lens's horizontal FOV; adjust to your exact HFOV)

**2. Calculate Pixel Focal Length ($f_{px}$):**
* `f_px = (FRAME_WIDTH / 2.0) / tan(HFOV_degrees / 2.0 * π / 180)`
* *Example for 102° HFOV:* `f_px = 240 / tan(51°) = 240 / 1.2349 = 194.34 px`

---

## Phase 2: Frame-by-Frame Geometry (Run per YOLO Detection)
For every frame $t$, you receive a YOLO bounding box: `box_x, box_y, box_w, box_h`.

**1. Safety Check (Edge Clipping):**
* If `box_x < 4` OR `box_y < 4` OR `(box_x + box_w) > 476` OR `(box_y + box_h) > 476`:
  * **ACTION:** DROP THE FRAME. The box is clipped. Math will be wrong.

**2. Extract YOLO Centers & Max Dimension:**
* `u = box_x + (box_w / 2.0)`
* `v = box_y + (box_h / 2.0)`
* `w_px = max(box_w, box_h)` *(Takes max to assume the drone's true symmetrical span)*

**3. Euclidean 3D Coordinate Calculation:**
* `Z_meters = (f_px * W) / w_px`
* `X_meters = ((u - c_x) * Z_meters) / f_px`
* `Y_meters = ((v - c_y) * Z_meters) / f_px`

*(Now you have the exact 3D position in meters relative to the camera lens)*

---

## Phase 3: Kalman Filter Physics (Run per Frame)
You need to track the time difference (`dt`) in seconds since the last frame.

### Step 3A: Predict (Where should it be now?)
The physics engine steps forward using the velocities ($v_x, v_y, v_s$) and accelerations ($a_x, a_y, a_s$) calculated from the *previous* frame.

* `x_pred = x_prev + (v_x * dt) + (0.5 * a_x * dt^2)`
* `y_pred = y_prev + (v_y * dt) + (0.5 * a_y * dt^2)`
* `scale_pred = scale_prev + (v_s * dt) + (0.5 * a_s * dt^2)`
* `v_x = v_x + (a_x * dt)`
* `v_y = v_y + (a_y * dt)`

### Step 3B: Dynamic Noise Gating (Do we trust YOLO?)
* `meas_scale = sqrt(box_w * box_h)`
* `size_ratio = 100.0 / max(10.0, meas_scale)`
* `Measurement_Noise (R) = 1.0 * (size_ratio^2)`
* *Logic:* If the drone is tiny (far away), $R$ becomes massive. The Kalman filter will ignore YOLO jitter and rely heavily on physics (`x_pred`).

### Step 3C: Correct (Fuse YOLO with Physics)
Standard Kalman gain ($K$) equations merge the YOLO measurement (`u, v, meas_scale`) with the Prediction (`x_pred, y_pred, scale_pred`). The output is the final, smoothed coordinate:
* `x_smoothed`, `y_smoothed`, `scale_smoothed`
* *Note: The Kalman filter also outputs updated continuous velocities (`v_x, v_y`) in pixels/second!*

---

## Phase 4: Flight Controller PID Math
Translate the smoothed Kalman outputs into drone velocities ($m/s$ or $rad/s$).

**1. Calculate Errors:**
* `Error_X_px = x_smoothed - c_x`
* `Error_Y_px = y_smoothed - c_y`
* `Error_Z_m = Z_meters - Desired_Distance_m`  *(e.g., 3.0m)*

**2. Calculate PID Commands (Using Kalman Velocity for D-Term):**
Instead of calculating a noisy derivative (`(Error_current - Error_prev)/dt`), inject the Kalman velocity directly to perfectly dampen the movement!

* **Yaw Command (rad/s):**
  * `Cmd_Yaw = (Kp_yaw * Error_X_px) + (Kd_yaw * v_x_kalman)`
* **Altitude Command (m/s):**
  * `Cmd_Down = (Kp_down * Error_Y_px) + (Kd_down * v_y_kalman)`
* **Forward Command (m/s):**
  * *First, calculate derivative of Z:* `dZ = (Z_meters - Z_prev) / dt`
  * *Then filter it:* `filtered_dZ = (0.20 * dZ) + (0.80 * filtered_dZ_prev)`
  * `Cmd_Fwd = (Kp_fwd * Error_Z_m) + (Kd_fwd * filtered_dZ)`

**3. Apply Deadbands & Slew Rate Limits:**
* **Deadband:** If `abs(Cmd_Yaw) < 5.0`, set `Cmd_Yaw = 0` (Prevents micro-twitching).
* **Slew Limit:** `Final_Cmd = Prev_Cmd + clamp(Cmd - Prev_Cmd, -Max_Accel * dt, Max_Accel * dt)`

*(These `Final_Cmd` variables are what get packed into MAVLink and sent directly to ArduPilot!)*
