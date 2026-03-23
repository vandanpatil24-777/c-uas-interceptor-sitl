import time
import math
import streamlit as st
import plotly.graph_objects as go
from pymavlink import mavutil

st.set_page_config(page_title="C-UAS Interceptor", layout="wide", initial_sidebar_state="expanded")
st.title("🎯 3D Tactical Interceptor Dashboard")

st.sidebar.header("ENGAGEMENT PARAMETERS")
N = st.sidebar.slider("PN GAIN (ProNav Multiplier)", min_value=1.0, max_value=8.0, value=4.0, step=0.1)
st.sidebar.caption("Low Gain (< 2.0) = Missile chases the tail.\nHigh Gain (> 3.5) = Missile predicts the curve.")

if st.sidebar.button("RESET SIMULATION"):
    for key in list(st.session_state.keys()):
        del st.session_state[key]
    st.rerun()

col1, col2, col3, col4 = st.columns(4)
metric_dist = col1.empty()
metric_turn = col2.empty()
metric_drone_alt = col3.empty()
metric_status = col4.empty()

chart_placeholder = st.empty()

@st.cache_resource
def get_mav_connection():
    return mavutil.mavlink_connection('udpin:127.0.0.1:14550')

master = get_mav_connection()

if 'init' not in st.session_state:
    st.session_state.init = True
    st.session_state.int_lat = 19.0100
    st.session_state.int_lon = 73.0500
    st.session_state.int_alt = 50.0          # START HIGHER: Launch from 50m
    st.session_state.int_heading_az = 45.0  
    st.session_state.int_heading_el = 60.0   # PITCH UP: Start by pointing 60 degrees up
    
    st.session_state.previous_los_az = None
    st.session_state.previous_los_el = None
    
    st.session_state.target_path_lat = []
    st.session_state.target_path_lon = []
    st.session_state.target_path_alt = []
    
    st.session_state.int_path_lat = []
    st.session_state.int_path_lon = []
    st.session_state.int_path_alt = []
    
    st.session_state.last_physics_time = None
    st.session_state.intercepted = False
    st.session_state.distance = 0.0
    st.session_state.g_force = 0.0
    st.session_state.target_alt = 0.0

int_speed = 35.0  
R = 6378137.0

# --- UI UPDATE TIMER (Prevents Flickering) ---
last_ui_update = time.time()
UI_UPDATE_RATE = 0.15 

if not st.session_state.intercepted:
    with st.spinner("Searching for MAVLink Telemetry Stream..."):
        master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    metric_status.metric("System Status", "TRACKING TARGET", "Active")

try:
    while not st.session_state.intercepted:
        packets_processed = 0
        
        while True:
            msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
            if not msg:
                break 
                
            current_time = msg.time_boot_ms / 1000.0 
            
            if st.session_state.last_physics_time is None:
                st.session_state.last_physics_time = current_time
                continue
                
            dt = current_time - st.session_state.last_physics_time
            if dt <= 0: continue 
            st.session_state.last_physics_time = current_time

            target_lat = msg.lat / 1e7
            target_lon = msg.lon / 1e7
            st.session_state.target_alt = msg.alt / 1000.0
            
            st.session_state.target_path_lat.append(target_lat)
            st.session_state.target_path_lon.append(target_lon)
            st.session_state.target_path_alt.append(st.session_state.target_alt)

            dy_north = (target_lat - st.session_state.int_lat) * (math.pi / 180.0) * R
            dx_east = (target_lon - st.session_state.int_lon) * (math.pi / 180.0) * R * math.cos(st.session_state.int_lat * math.pi / 180.0)
            dz_up = st.session_state.target_alt - st.session_state.int_alt

            ground_distance = math.sqrt(dx_east**2 + dy_north**2)
            st.session_state.distance = math.sqrt(ground_distance**2 + dz_up**2)

            if st.session_state.distance < 15.0:
                st.session_state.intercepted = True
                break

            los_az = math.atan2(dy_north, dx_east)
            los_el = math.atan2(dz_up, ground_distance)

            if st.session_state.previous_los_az is None: 
                st.session_state.previous_los_az = los_az
                st.session_state.previous_los_el = los_el
            
            los_rate_az = (los_az - st.session_state.previous_los_az) / dt
            los_rate_el = (los_el - st.session_state.previous_los_el) / dt
            
            st.session_state.previous_los_az = los_az
            st.session_state.previous_los_el = los_el

            accel_az = N * int_speed * los_rate_az
            accel_el = N * int_speed * los_rate_el
            
            st.session_state.g_force = math.sqrt(accel_az**2 + accel_el**2) / 9.81
            
            turn_rate_az = accel_az / int_speed
            turn_rate_el = accel_el / int_speed

            st.session_state.int_heading_az += turn_rate_az * dt * (180.0 / math.pi)
            st.session_state.int_heading_el += turn_rate_el * dt * (180.0 / math.pi)
            st.session_state.int_heading_el = max(-80.0, min(80.0, st.session_state.int_heading_el)) 

            vz = int_speed * math.sin(st.session_state.int_heading_el * math.pi / 180.0)
            ground_speed = int_speed * math.cos(st.session_state.int_heading_el * math.pi / 180.0)
            vx = ground_speed * math.cos(st.session_state.int_heading_az * math.pi / 180.0)
            vy = ground_speed * math.sin(st.session_state.int_heading_az * math.pi / 180.0)

            st.session_state.int_alt += vz * dt
            st.session_state.int_lat += (vy * dt / R) * (180.0 / math.pi)
            st.session_state.int_lon += (vx * dt / R) * (180.0 / math.pi) / math.cos(st.session_state.int_lat * math.pi / 180.0)

            st.session_state.int_path_lat.append(st.session_state.int_lat)
            st.session_state.int_path_lon.append(st.session_state.int_lon)
            st.session_state.int_path_alt.append(st.session_state.int_alt)
            
            packets_processed += 1

        if packets_processed == 0:
            time.sleep(0.02)
            continue
            
        current_time_real = time.time()
        
        # --- UI THROTTLING LOGIC ---
        if (current_time_real - last_ui_update > UI_UPDATE_RATE) or st.session_state.intercepted:
            last_ui_update = current_time_real

            if st.session_state.intercepted:
                metric_status.metric("System Status", "TARGET DESTROYED", "Impact")
                st.success("Target Intercepted Successfully in 3D Space!")
            
            metric_dist.metric("Slant Range", f"{st.session_state.distance:.1f} m")
            metric_turn.metric("Maneuver Load", f"{st.session_state.g_force:.2f} G")
            metric_drone_alt.metric("Target Altitude", f"{st.session_state.target_alt:.1f} m")

            fig = go.Figure()
            
            fig.add_trace(go.Scatter3d(
                x=st.session_state.target_path_lon[-300:], 
                y=st.session_state.target_path_lat[-300:], 
                z=st.session_state.target_path_alt[-300:],
                mode='lines+markers', 
                name='Evasive Target', 
                line=dict(color='red', width=4),
                marker=dict(size=2, color='red')
            ))
            
            fig.add_trace(go.Scatter3d(
                x=st.session_state.int_path_lon[-300:], 
                y=st.session_state.int_path_lat[-300:], 
                z=st.session_state.int_path_alt[-300:],
                mode='lines+markers', 
                name='Kinetic Interceptor', 
                line=dict(color='cyan', width=4),
                marker=dict(size=2, color='cyan')
            ))
            
            fig.update_layout(
                height=650, 
                template="plotly_dark", 
                scene=dict(
                    xaxis_title="Longitude",
                    yaxis_title="Latitude",
                    zaxis_title="Altitude (m)",
                    aspectmode='cube' 
                ),
                margin=dict(l=0, r=0, b=0, t=0)
            )
            chart_placeholder.plotly_chart(fig, use_container_width=True)

except Exception as e:
    st.error(f"Simulation ended or error occurred: {e}")