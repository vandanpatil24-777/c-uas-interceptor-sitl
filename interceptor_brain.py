import time
import math
from pymavlink import mavutil

# --- 1. SETUP TELEMETRY LISTENER ---
# 'udpin' means we are listening on this port for incoming data
print("Initializing Interceptor Brain...")
master = mavutil.mavlink_connection('udpin:127.0.0.1:14550')

# Wait for the first heartbeat from the virtual drone to confirm connection
print("Waiting for Target Heartbeat...")
master.wait_heartbeat()
print(f"Target Acquired! System ID: {master.target_system}")

# --- 2. INTERCEPTOR KINEMATIC SETUP ---
# Starting the interceptor slightly South-West of the target's spawn point
int_lat = 19.0100  
int_lon = 73.0500  
int_speed = 25.0       # m/s (Missile must be faster than the 15m/s target)
int_heading = 45.0     # Degrees (pointing roughly North-East)

# Proportional Navigation Constant
N = 4.0 

# Earth radius for distance math
R = 6378137.0 
previous_los_angle = None
last_time = time.time()

print("\n--- LAUNCHING INTERCEPTOR ---")

try:
    while True:
        # --- 3. DATA INGESTION ---
        # Catch the GPS packet (blocking=True means it waits for the next packet)
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if not msg:
            continue

        current_time = time.time()
        dt = current_time - last_time
        if dt == 0: continue # Prevent divide by zero on first loop
        last_time = current_time

        # Unpack the integers back into floating-point decimals
        target_lat = msg.lat / 1e7
        target_lon = msg.lon / 1e7
        target_alt = msg.alt / 1000.0
        
        # --- 4. RELATIVE KINEMATICS (The Math) ---
        # Convert lat/lon differences into approximate meters
        dy_meters = (target_lat - int_lat) * (math.pi / 180.0) * R
        dx_meters = (target_lon - int_lon) * (math.pi / 180.0) * R * math.cos(int_lat * math.pi / 180.0)

        # Calculate Distance to Target
        distance = math.sqrt(dx_meters**2 + dy_meters**2)

        if distance < 10.0:
            print(f"\n[BOOM] TARGET INTERCEPTED! Distance: {distance:.1f}m")
            break

        # Calculate Line-Of-Sight (LOS) Angle (Lambda)
        los_angle = math.atan2(dy_meters, dx_meters)

        # Calculate LOS Rate (Lambda Dot)
        if previous_los_angle is None:
            previous_los_angle = los_angle
        
        los_rate = (los_angle - previous_los_angle) / dt
        previous_los_angle = los_angle

        # --- 5. PROPORTIONAL NAVIGATION GUIDANCE ---
        # Formula: commanded_acceleration = N * closing_velocity * LOS_rate
        # For simplicity in this 2D text output, we approximate closing velocity as our speed
        closing_velocity = int_speed 
        lateral_acceleration = N * closing_velocity * los_rate

        # Convert acceleration into a turn rate (change in heading)
        turn_rate_rad = lateral_acceleration / int_speed
        int_heading += turn_rate_rad * dt * (180.0 / math.pi) # Convert back to degrees

        # Move the interceptor forward along its new heading
        int_dx = int_speed * math.cos(int_heading * math.pi / 180.0) * dt
        int_dy = int_speed * math.sin(int_heading * math.pi / 180.0) * dt

        # Update interceptor GPS coordinates
        int_lat += (int_dy / R) * (180.0 / math.pi)
        int_lon += (int_dx / R) * (180.0 / math.pi) / math.cos(int_lat * math.pi / 180.0)

        # --- 6. TELEMETRY OUTPUT ---
        print(f"Dist: {distance:.1f}m | Target: {target_lat:.5f}, {target_lon:.5f} | Interceptor: {int_lat:.5f}, {int_lon:.5f} | Turn Command: {lateral_acceleration:.2f} m/s^2")

except KeyboardInterrupt:
    print("\nTracking Terminated.")