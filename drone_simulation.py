import numpy as np

class CentroidKalmanFilter:
    def __init__(self):
        # 9-State Vector: [x, y, scale, vx, vy, vscale, ax, ay, ascale]
        self.statePost = np.zeros((9, 1), dtype=np.float32)
        self.statePre = np.zeros((9, 1), dtype=np.float32)
        
        # Error Covariance (P)
        self.errorCovPost = np.eye(9, dtype=np.float32)
        self.errorCovPost[0:3, 0:3] *= 20.0
        self.errorCovPost[3:6, 3:6] *= 150.0
        self.errorCovPost[6:9, 6:9] *= 300.0
        
        # Process Noise Covariance (Q)
        self.processNoiseCov = np.zeros((9, 9), dtype=np.float32)
        self.processNoiseCov[0:3, 0:3] = np.eye(3) * 50.0   # Position/Scale noise
        self.processNoiseCov[3:6, 3:6] = np.eye(3) * 150.0  # Velocity noise
        self.processNoiseCov[6:9, 6:9] = np.eye(3) * 300.0  # Acceleration noise
        
        # Measurement Matrix (H) - We only measure [x, y, scale]
        self.measurementMatrix = np.zeros((3, 9), dtype=np.float32)
        self.measurementMatrix[0, 0] = 1.0
        self.measurementMatrix[1, 1] = 1.0
        self.measurementMatrix[2, 2] = 1.0
        
        # Base Measurement Noise (R)
        self.base_measurement_noise = 1.0
        self.measurementNoiseCov = np.eye(3, dtype=np.float32) * self.base_measurement_noise
        
        self.initialized = False
        self.previous_area = 0.0

    def initialize(self, x, y, area):
        self.statePost[0, 0] = x
        self.statePost[1, 0] = y
        self.statePost[2, 0] = np.sqrt(area) if area > 0 else 100.0
        self.statePost[3:, 0] = 0.0 # Zero out velocities and accelerations
        self.statePre = np.copy(self.statePost)
        self.initialized = True
        self.previous_area = area

    def get_transition_matrix(self, dt):
        """Builds Newtonian physics transition matrix (F)"""
        dt = np.clip(dt, 0.01, 0.25)
        dt2 = 0.5 * dt * dt
        
        F = np.eye(9, dtype=np.float32)
        # Position += v*dt + 0.5*a*dt^2
        F[0, 3], F[0, 6] = dt, dt2
        F[1, 4], F[1, 7] = dt, dt2
        F[2, 5], F[2, 8] = dt, dt2
        # Velocity += a*dt
        F[3, 6] = dt
        F[4, 7] = dt
        F[5, 8] = dt
        return F

    def predict(self, dt):
        if not self.initialized:
            return None
            
        F = self.get_transition_matrix(dt)
        
        # statePre = F * statePost
        self.statePre = np.dot(F, self.statePost)
        
        # errorCovPre = F * errorCovPost * F^T + Q
        self.errorCovPre = np.dot(np.dot(F, self.errorCovPost), F.T) + self.processNoiseCov
        
        # Velocity and Acceleration Clamping (Safety limits)
        self.statePre[3:6, 0] = np.clip(self.statePre[3:6, 0], -700.0, 700.0)
        self.statePre[6:9, 0] = np.clip(self.statePre[6:9, 0], -2000.0, 2000.0)
        
        return {
            'x': self.statePre[0, 0], 
            'y': self.statePre[1, 0],
            'vx': self.statePre[3, 0],
            'vy': self.statePre[4, 0]
        }

    def correct(self, meas_x, meas_y, meas_area):
        meas_scale = np.sqrt(meas_area) if meas_area > 0 else np.sqrt(self.previous_area)
        
        # Dynamic Noise Gating: Increase noise R if the drone is far away (small scale)
        size_ratio = 100.0 / max(10.0, meas_scale)
        r_scale = self.base_measurement_noise * (size_ratio * size_ratio)
        self.measurementNoiseCov[2, 2] = np.clip(r_scale, self.base_measurement_noise, self.base_measurement_noise * 25.0)
        
        # Measurement Vector (Z)
        Z = np.array([[meas_x], [meas_y], [meas_scale]], dtype=np.float32)
        
        # Kalman Gain (K) = P_pre * H^T * (H * P_pre * H^T + R)^-1
        H = self.measurementMatrix
        S = np.dot(np.dot(H, self.errorCovPre), H.T) + self.measurementNoiseCov
        K = np.dot(np.dot(self.errorCovPre, H.T), np.linalg.inv(S))
        
        # statePost = statePre + K * (Z - H * statePre)
        y = Z - np.dot(H, self.statePre)
        self.statePost = self.statePre + np.dot(K, y)
        
        # errorCovPost = (I - K * H) * errorCovPre
        I = np.eye(9, dtype=np.float32)
        self.errorCovPost = np.dot(I - np.dot(K, H), self.errorCovPre)
        
        self.previous_area = meas_area
        return {
            'x': self.statePost[0, 0], 
            'y': self.statePost[1, 0],
            'vx': self.statePost[3, 0],
            'vy': self.statePost[4, 0],
            'ax': self.statePost[6, 0],
            'ay': self.statePost[7, 0]
        }

# ==========================================================
# Example Simulation Usage for your AI Training Pipeline
# ==========================================================
if __name__ == "__main__":
    import random
    
    # 1. Initialize the Drone Simulation Math
    kf = CentroidKalmanFilter()
    
    # 2. Camera Constants
    FRAME_WIDTH = 480
    FRAME_HEIGHT = 480
    TARGET_WIDTH_M = 0.35 # 35cm
    HFOV = 102.0
    # Calculate Pixel Focal Length
    f_px = (FRAME_WIDTH / 2.0) / np.tan(np.radians(HFOV / 2.0))
    
    # Let's say your AI detects the drone here initially:
    kf.initialize(x=240, y=240, area=(40*40))
    
    print("Simulating 5 frames of AI YOLO detections...")
    for frame_step in range(1, 6):
        dt = 0.033 # Assume 30 FPS
        
        # Step 1: PREDICT (Where physics says the drone should be)
        prediction = kf.predict(dt)
        
        # Step 2: FAKE AI DETECTION (Add some noise to simulate a bad YOLO bounding box)
        yolo_x = 245 + random.uniform(-5, 5)
        yolo_y = 238 + random.uniform(-5, 5)
        yolo_w = 42 + random.uniform(-2, 2)
        yolo_h = 42 + random.uniform(-2, 2)
        yolo_area = yolo_w * yolo_h
        
        # Step 3: CORRECT (Kalman merges physics with your AI's noisy detection)
        smoothed = kf.correct(yolo_x, yolo_y, yolo_area)
        
        # Step 4: EUCLIDEAN GEOMETRY (Convert smoothed pixels to 3D distance)
        max_dim = max(yolo_w, yolo_h)
        Z_meters = (f_px * TARGET_WIDTH_M) / max_dim
        
        print(f"Frame {frame_step}:")
        print(f"  Raw AI YOLO: X={yolo_x:.1f}, Y={yolo_y:.1f}")
        print(f"  Kalman Smoothed: X={smoothed['x']:.1f}, Y={smoothed['y']:.1f}, Vx={smoothed['vx']:.1f} px/s")
        print(f"  Calculated Distance: {Z_meters:.2f} meters\n")
