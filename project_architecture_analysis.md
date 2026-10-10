# Drone Tracking Project: Full Architecture Analysis

After analyzing the contents of the `/home/saugat/baloonpoper/ballonpop` directory, here is a comprehensive breakdown of the entire software architecture. 

The software is an autonomous, vision-based drone interception system written in C++. It consists of several tightly integrated modules handling everything from camera capture and neural network inference to flight control and radio telemetry.

---

## 1. Vision & Perception Layer

### `camera_capture.cpp / .hpp`
*   **Purpose:** Manages the video feed.
*   **Capabilities:** It connects directly to the Raspberry Pi libcamera stack via GStreamer (or a standard USB webcam via OpenCV). It automatically handles "letterboxing," reshaping wide-angle video feeds (e.g., 4608x2592) into perfectly scaled, padded $480 \times 480$ squares to ensure no physical distortion stretches the drone image before detection.

### `balloon_detector.cpp / .hpp`
*   **Purpose:** The Neural Network layer.
*   **Capabilities:** Uses OpenCV's DNN module (Deep Neural Networks) to run a custom model (`balloon_model/`). It takes the 480x480 frame and returns bounding boxes (`Detection` objects). Notably, it even contains logic for **Fisheye Flattening** to mathematically correct ultra-wide lens barrel distortion before running the neural network!

---

## 2. Tracking & Math Layer

### `distance_estimator.cpp / .hpp`
*   **Purpose:** Translates 2D pixels to 3D reality.
*   **Capabilities:** Uses the Euclidean pinhole formulas (scaling the focal length automatically from the HFOV) to calculate the $Z, X, Y$ distances of the drone. Includes safety logic to completely abort calculations if the detection box is touching the edge of the camera frame (edge-clipping).

### `centroid_filter.cpp / .hpp`
*   **Purpose:** The mathematical "Brain" (Kalman Filter).
*   **Capabilities:** Runs a 9-state 3D Acceleration Kalman Filter. It doesn't just record where the target is; it learns how fast it's moving and accelerating. It dynamically scales its "trust" in the YOLO detections based on how far away the target is. If the camera misses the target for a few frames, the filter enters "coasting mode," blindly guessing where the target went to keep the camera locked smoothly.

---

## 3. Flight & Control Layer

### `chase_controller.cpp / .hpp`
*   **Purpose:** Commands the drone to move.
*   **Capabilities:** Translates the 3D distances and Kalman velocities into actual flight speeds. It runs 4 independent PID loops (Forward, Lateral, Down, Yaw). It uses the Kalman filter's smooth continuous velocity for D-term dampening, preventing drone twitching. It also supports "Sky-Cam Mode" (if the camera points straight up) and a "Blind Charge" (terminal ramming maneuver at the end of a chase).

### `mavlink_client.cpp / .hpp`
*   **Purpose:** Talks to the Flight Controller (ArduPilot/PX4).
*   **Capabilities:** A heavy MAVSDK wrapper that handles arming, taking off, and safety checks. More importantly, it runs a dedicated background thread that streams velocity setpoints to the drone at **30 Hz**. If this stream drops, the flight controller's offboard failsafe would trigger, so this dedicated thread ensures the drone never loses its connection.

---

## 4. Telemetry & Support Layer

### `lora_link.cpp / .hpp`
*   **Purpose:** Long-range ground control.
*   **Capabilities:** Interacts directly with an SX1262 LoRa radio chip over SPI. It maintains a continuous listening state, dropping corrupted packets and authenticating commands with a shared key. This is how you send commands to the drone (e.g., arm, land) from the ground without Wi-Fi!

### `main.cpp`
*   **Purpose:** The Orchestrator.
*   **Capabilities:** Over 4,000 lines tying the whole system together. It parses command-line arguments (like `--camera-index`, `--focal-length`), initializes the LoRa radio, starts the camera, runs the YOLO inference loop, feeds the Kalman filter, and finally pipes the output into the Chase Controller and MAVLink client at 30+ frames per second.

### `build.sh` & `run.sh`
*   Standard CMake compilation scripts and systemd service wrappers to ensure the software runs on boot automatically as a background daemon (`codespi.service`).
