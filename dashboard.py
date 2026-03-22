import time
import math
import streamlit as st
import plotly.graph_objects as go
from pymavlink import mavutil

st.set_page_config(page_title="C-UAS Interceptor", layout="wide", initial_sidebar_state="expanded")
st.title("🎯 Tactical Interceptor Dashboard")

st.sidebar.header("ENGAGEMENT PARAMETERS")
N = st.sidebar.slider("PN GAIN (ProNav Multiplier)", min_value=1.0, max_value=8.0, value=4.0, step=0.1)
st.sidebar.caption("Low Gain (< 2.0) = Missile chases the tail and misses.\nHigh Gain (> 3.5) = Missile predicts the curve and hits.")

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
    st.session_state.int_heading = 45.0
    st.session_state.previous_los_angle = None
    st.session_state.target_path_lat = []
    st.session_state.target_path_lon = []
    st.session_state.int_path_lat = []
    st.session_state.int_path_lon = []
    st.session_state.last_physics_time = None
    st.session_state.intercepted = False
    st.session_state.distance = 0.0
    st.session_state.g_force = 0.0
    st.session_state.target_alt = 0.0

int_speed = 45.0  
R = 6378137.0

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

            dy_meters = (target_lat - st.session_state.int_lat) * (math.pi / 180.0) * R
            dx_meters = (target_lon - st.session_state.int_lon) * (math.pi / 180.0) * R * math.cos(st.session_state.int_lat * math.pi / 180.0)
            st.session_state.distance = math.sqrt(dx_meters**2 + dy_meters**2)

            if st.session_state.distance < 15.0:
                st.session_state.intercepted = True
                break

            los_angle = math.atan2(dy_meters, dx_meters)
            if st.session_state.previous_los_angle is None: 
                st.session_state.previous_los_angle = los_angle
            
            los_rate = (los_angle - st.session_state.previous_los_angle) / dt
            st.session_state.previous_los_angle = los_angle

            lateral_acceleration = N * int_speed * los_rate
            st.session_state.g_force = lateral_acceleration / 9.81
            turn_rate_rad = lateral_acceleration / int_speed
            st.session_state.int_heading += turn_rate_rad * dt * (180.0 / math.pi)

            int_dx = int_speed * math.cos(st.session_state.int_heading * math.pi / 180.0) * dt
            int_dy = int_speed * math.sin(st.session_state.int_heading * math.pi / 180.0) * dt

            st.session_state.int_lat += (int_dy / R) * (180.0 / math.pi)
            st.session_state.int_lon += (int_dx / R) * (180.0 / math.pi) / math.cos(st.session_state.int_lat * math.pi / 180.0)

            st.session_state.int_path_lat.append(st.session_state.int_lat)
            st.session_state.int_path_lon.append(st.session_state.int_lon)
            
            packets_processed += 1

        if packets_processed == 0:
            time.sleep(0.05)
            continue
            
        if st.session_state.intercepted:
            metric_status.metric("System Status", "TARGET DESTROYED", "Impact")
            st.success("Target Intercepted Successfully!")
        
        metric_dist.metric("Distance to Target", f"{st.session_state.distance:.1f} m")
        metric_turn.metric("Turn Command (G-Force)", f"{st.session_state.g_force:.2f} G")
        metric_drone_alt.metric("Target Altitude", f"{st.session_state.target_alt} m")

        fig = go.Figure()
        fig.add_trace(go.Scatter(x=st.session_state.target_path_lon[-300:], y=st.session_state.target_path_lat[-300:], mode='lines+markers', name='Evasive Target', marker=dict(color='red', size=4)))
        fig.add_trace(go.Scatter(x=st.session_state.int_path_lon[-300:], y=st.session_state.int_path_lat[-300:], mode='lines+markers', name='Interceptor', marker=dict(color='cyan', size=4)))
        
        # THE FIX: Added scaleanchor to lock the map's aspect ratio
        fig.update_layout(
            height=650, 
            template="plotly_dark", 
            title="Live Tactical Interception Radar", 
            xaxis_title="Longitude", 
            yaxis=dict(title="Latitude", scaleanchor="x", scaleratio=1), # Locks aspect ratio 1:1
            margin=dict(l=20, r=20, t=40, b=20)
        )
        chart_placeholder.plotly_chart(fig, use_container_width=True)

except Exception as e:
    st.error(f"Simulation ended or error occurred: {e}")