# 🎯 C-UAS Kinetic Interceptor SITL Simulation

A Software-In-The-Loop (SITL) simulation architected to test Counter-UAS (Unmanned Aerial Systems) interception algorithms. This project simulates an evasive rogue drone broadcasting MAVLink telemetry over a local network, which is actively tracked and intercepted by a custom Ground Control Station (GCS) using Proportional Navigation kinematics.

## 🚀 Core Features
* **Real-Time MAVLink Telemetry:** The target drone simulates a Pixhawk flight controller, packing and broadcasting `GLOBAL_POSITION_INT` and `HEARTBEAT` packets over UDP (Port 14550).
* **Evasive Target Kinematics:** The virtual drone employs mathematical sine-wave oscillation to simulate aggressive evasive maneuvers (zigzagging) during flight.
* **Proportional Navigation (PN) Guidance:** The interceptor utilizes true PN guidance laws to calculate Line-of-Sight (LOS) rates and command lateral G-forces, predicting the target's trajectory rather than tail-chasing.
* **Decoupled Asynchronous Dashboard:** Built with Streamlit and Plotly, the UI rendering thread is fully decoupled from the physics/telemetry ingestion loop to ensure zero latency in kinematic calculations.

## 🧮 The Mathematics (Proportional Navigation)
The interceptor calculates its lateral acceleration ($a_c$) using the following guidance law:
$$a_c = N \cdot V_c \cdot \dot{\lambda}$$
Where $N$ is the Navigation Gain (tunable in the UI), $V_c$ is the closing velocity, and $\dot{\lambda}$ is the Line-of-Sight rate between the interceptor and the target.

## 🛠️ System Architecture
1. `virtual_drone.py`: The data source. Acts as the rogue UAV, generating coordinates and streaming them over UDP.
2. `dashboard.py`: The brain and UI. Ingests the UDP stream, unpacks the MAVLink integers, runs the PN interception math at high frequency, and visualizes the engagement on a locked-aspect tactical radar.

## 💻 How to Run
**1. Install Dependencies**
`pip install pymavlink streamlit plotly pandas`

**2. Launch the Target Drone (Terminal 1)**
`python virtual_drone.py`

**3. Launch the Interceptor GCS (Terminal 2)**
`streamlit run dashboard.py`