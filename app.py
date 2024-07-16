# user_interface/app.py

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
nav_cam_html = f'<iframe src="http://192.168.5.240:8080/?action=stream" width="100%" height="600" frameborder="0"></iframe>'


@st.cache_resource
def get_nav_service():
    lidar = LidarController()
    compass = HMC5883L()
    drive = DriveController(PWMController())
    gps = GPSController()
    return NavigationService(compass, drive, lidar, gps)


def on_nav_cam_pan_position_changed():
    position = st.session_state.nav_cam_pan
    (NavigationCameraController(navigation_service.drive_controller.pwm_controller)
     .set_pan_position(position))


def get_gps_data(gps_controller: GPSController):
    return gps_controller.get_longitude(), gps_controller.get_latitude()


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
            width=1000,
            height=1000,
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
    # st.map(lat=navigation_service.gps_controller.latitude,
    #        lon=navigation_service.gps_controller.longitude, size=2, zoom=17)
    # st.subheader("Compass Data")
    # st.write(f"Heading: {navigation_service.compass_controller.get_heading()}")
    st.empty()


nav_col, compass_col, lidar_col = st.columns(3, gap="medium")

# Create buttons for drive methods
with nav_col:
    st.subheader("Drive Controls")
    components.iframe("http://192.168.5.240:8080/?action=stream", width=800, height=600)
    speed = st.slider("Speed", min_value=0, max_value=4095, key="speed")
    navigation_service.drive_controller.set_speed(speed)
    pan = st.slider("Camera Pan", min_value=512, max_value=0, key="nav_cam_pan")
    (NavigationCameraController(navigation_service.drive_controller.pwm_controller).set_pan_position(pan))
    if st.button("Forward", key="fwd"):
        navigation_service.drive_forward(4095)  # Example speed value
    if st.button("Backward", key="bwd"):
        navigation_service.drive_backward(4095)  # Example speed value
    if st.button("Left", key="left"):
        navigation_service.drive_left(4000)  # Example speed value
    if st.button("Right", key="right"):
        navigation_service.drive_right(4000)  # Example speed value
    if st.button("Stop", key="stop"):
        navigation_service.stop_driving()
with compass_col:
    gps_widget()

with lidar_col:
    lidar_widget()
