# user_interface/app.py
import requests
import streamlit as st
import streamlit.components.v1 as components
import plotly.graph_objects as go
from time import sleep
import pandas as pd
from devices.compass import HMC5883L
from devices.drive import DriveController
from devices.gps import GPSController
from devices.lidar import LidarController
from devices.nav_cam import NavigationCameraController
from devices.pwm_controller import PWMController
from servicess.navigation_service import NavigationService

st.set_page_config(layout="wide")
st.session_state.gps_data = pd.DataFrame(columns=["lat", "lon"])
nav_cam_html = f'<iframe src="http://192.168.5.240:8080/?action=stream" width="100%" frameborder="0"></iframe>'


@st.cache_resource
def get_nav_service():
    lidar = LidarController()
    drive = DriveController(PWMController())
    gps = GPSController()
    return NavigationService(drive, lidar, gps)


def on_nav_cam_pan_position_changed():
    position = st.session_state.nav_cam_pan
    (NavigationCameraController(navigation_service.drive_controller.pwm_controller)
     .set_pan_position(position))


def get_gps_data(gps_controller: GPSController):
    return gps_controller.get_gps_reading()

def display_compass_heading():
    compass_heading = navigation_service.gps_controller.get_heading()
    fig = go.Figure(go.Scatterpolar(
        r=[1, 1],
        theta=[0, compass_heading],
        mode='lines',
        line=dict(color='red', width=4)
    ))
    fig.update_layout(
        title='Compass Heading',
        polar=dict(
            radialaxis=dict(visible=False),
            angularaxis=dict(
                tickmode='array',
                tickvals=[0, 90, 180, 270],
                ticktext=['N', 'E', 'S', 'W']
            )
        ),
        showlegend=False,
        width=500,
        height=500,
        template="plotly_dark"
    )
    return fig

def display_lidar_data():
    # Create a polar graph using Plotly
    lidar_last_reading_df = navigation_service.lidar_controller.get_last_reading()
    fig = go.Figure(go.Scatterpolar(
            r=lidar_last_reading_df["distance"],
            theta=lidar_last_reading_df["angle"],
            mode='lines+markers',
            marker=dict(
                    color=lidar_last_reading_df["quality"],  # Values to map to colors
                    colorscale='Viridis'  # Choose a colorscale
                    )
            ))
    fig.update_layout(
            title='Lidar Scan Data',
            polar=dict(
                    radialaxis=dict(visible=True, range=[0, 100]),
                    angularaxis=dict(
                            tickfont_size=8,
                            rotation=90,
                            direction="clockwise"
                            ),
                    ),
            width=500,
            height=500,
            template="plotly_dark"
            )
    return fig


navigation_service = get_nav_service()


@st.experimental_fragment(run_every=5)
def lidar_widget():
    lidar_fig = display_lidar_data()
    st.plotly_chart(lidar_fig)


@st.experimental_fragment(run_every=10)
def gps_widget():
    st.subheader("GPS Data")
    gps_data = get_gps_data(navigation_service.gps_controller)
    st.write(gps_data["datetime"])
    st.map(gps_data, size=2, zoom=17)
    st.write(gps_data["heading"])
    st.empty()


nav_col, compass_col, lidar_col = st.columns(3, gap="medium")

# Create buttons for drive methods
with nav_col:
    st.subheader("Drive Controls")
    # components.iframe("http://192.168.5.243/web/index.html", width=888, height=500)
    components.iframe("	http://192.168.5.243/cgi-bin/hi3510/snap.cgi?&-getstream&-chn=2", height=500)
    speed = st.slider("Speed", min_value=0, max_value=4095, key="speed")
    navigation_service.drive_controller.set_speed(speed)
    pan = st.slider("Camera Pan", min_value=512, max_value=90, key="nav_cam_pan")
    (NavigationCameraController(navigation_service.drive_controller.pwm_controller).set_pan_position(pan))
    with st.container():
        left_buttons, center_buttons, right_buttons = st.columns(3)
        with left_buttons:
            if st.button("Left", key="left"):
                navigation_service.drive_left()  # Example speed value
        with center_buttons:
            if st.button("Forward", key="fwd"):
                navigation_service.drive_forward()  # Example speed value
            if st.button("Stop", key="stop"):
                navigation_service.stop_driving()
            if st.button("Backward", key="bwd"):
                navigation_service.drive_backward()  # Example speed value
        with right_buttons:
            if st.button("Right", key="right"):
                navigation_service.drive_right()  # Example speed value
with compass_col:
    with st.container():
        cas_btn_1, cas_btn_2, right_laser_button, left_laser_button = st.columns(4)
        with cas_btn_1:
            if st.button("CAS On", key="cas_on"):
                navigation_service.collision_avoidance_system.turn_on()
        with cas_btn_2:
            if st.button("CAS Off", key="cas_off"):
                navigation_service.collision_avoidance_system.turn_off()
        with left_laser_button:
            if st.button("Laser On", key="laser_on"):
                requests.post("http://192.168.5.242:5000/laser/on")
        with right_laser_button:
            if st.button("Laser Off", key="laser_off"):
                requests.post("http://192.168.5.242:5000/laser/off")
    with st.container():
        gps_widget()
with lidar_col:
    lidar_widget()
    st.write(f"Collision Probability: {navigation_service.collision_avoidance_system.calculate_collision_probability() * 100}%")
    display_compass_heading()
