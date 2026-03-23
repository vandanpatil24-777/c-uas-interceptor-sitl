# 🎯 C-UAS Kinematic Interceptor Simulator (3D SITL)

![Interceptor Demo](demo_interception.mp4) 

A multi-threaded, cross-platform Software-In-The-Loop (SITL) simulation environment built to test Counter-UAS (Unmanned Aerial Systems) interception algorithms. This system simulates an evasive rogue drone and actively tracks it down in 3D space using true Proportional Navigation (PN) kinematics.

## 🚀 System Architecture
This project intentionally steps away from pre-compiled flight controllers (like ArduPilot/PX4) to demonstrate raw avionics software engineering, UDP socket programming, and custom control loops.

* **The Target (Python):** `virtual_drone.py` simulates a rogue UAV flying a 3D evasive S-curve. It packs and broadcasts `GLOBAL_POSITION_INT` telemetry packets via MAVLink over a local UDP network.
* **The Avionics Brain (C++):** `interceptor_brain.cpp` is a high-speed, cross-platform (POSIX/Winsock) backend. It ingests the MAVLink UDP stream, calculates 3D Azimuth and Elevation Line-of-Sight (LOS) rates, and commands lateral/vertical G-forces at high frequency.
* **The Ground Control Station (Python):** `dashboard.py` is an asynchronous Streamlit and Plotly dashboard. It renders the tactical airspace in 3D and logs maneuver loads in real-time, utilizing UI-throttling to prevent render blocking.

## 🧮 The Mathematics (3D Proportional Navigation)
Instead of simple "Pure Pursuit" (tail-chasing), the C++ backend calculates interceptor accelerations ($a_c$) in both planes to predict the collision point:
* **Azimuth Plane:** $a_{az} = N \cdot V_c \cdot \dot{\lambda}_{az}$
* **Elevation Plane:** $a_{el} = N \cdot V_c \cdot \dot{\lambda}_{el}$
*(Where $N$ is the Navigation Gain, $V_c$ is closing velocity, and $\dot{\lambda}$ is the Line-of-Sight rate).*

## 💻 How to Run (Local Simulation)
1. **Compile the C++ Brain:** `g++ interceptor_brain.cpp -o interceptor_brain.exe -I ./mavlink -lws2_32` *(Windows)*
2. **Launch Target Drone:** `python virtual_drone.py`
3. **Launch 3D GCS:** `streamlit run dashboard.py`
4. **Launch Interceptor:** `.\interceptor_brain.exe`