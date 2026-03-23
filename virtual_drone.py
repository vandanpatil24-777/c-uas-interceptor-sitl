import time
import math
from pymavlink import mavutil

# --- 1. SETUP UDP CONNECTION ---
master = mavutil.mavlink_connection('udpout:127.0.0.1:14550', source_system=1)
print("Evasive Target Simulator Online. Broadcasting...")

# --- 2. INITIAL DRONE STATE ---
start_lat = 19.0269  
start_lon = 73.0641  
current_alt = 100.0    
speed_north = 25.0  

R = 6378137.0 
update_rate_hz = 10
dt = 1.0 / update_rate_hz

start_time = time.time()

try:
    while True:
        # --- 3. EVASIVE KINEMATICS ---
        t = time.time() - start_time
        distance_north = speed_north * t
        
        # A long, sweeping 50-second curve (150m wide)
        evasion_amplitude = 150.0 
        evasion_speed = 0.02       
        offset_east = evasion_amplitude * math.sin(2 * math.pi * evasion_speed * t)
        
        current_lat = start_lat + (distance_north / R) * (180.0 / math.pi)
        current_lon = start_lon + (offset_east / R) * (180.0 / math.pi) / math.cos(start_lat * math.pi / 180.0)

        # --- 4. MAVLINK PACKAGING ---
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_QUADROTOR,
            mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            0, 0, 0
        )

        master.mav.global_position_int_send(
            int(t * 1000),              
            int(current_lat * 1e7),     
            int(current_lon * 1e7),     
            int(current_alt * 1000),    
            int(current_alt * 1000),    
            int(speed_north * 100),     
            0,                          
            0,                          
            0                           
        )
        time.sleep(dt)

except KeyboardInterrupt:
    print("\nSimulation Terminated.")