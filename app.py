# user_interface/app.py

import streamlit as st
import plotly.graph_objects as go
from time import sleep
import pandas as pd
from devices.compass import HMC5883L
from devices.drive import DriveController
from devices.gps import GPSController
from devices.lidar import LidarController
from devices.pwm_controller import PWMController
from servicess.navigation_service import NavigationService

st.set_page_config(layout="wide")


@st.cache_resource
def get_nav_service():
    pwm_controller = PWMController()
    lidar = LidarController()
    compass = HMC5883L()
    drive = DriveController(pwm_controller)
    gps = GPSController()
    return NavigationService(compass, drive, lidar, gps)


def get_gps_data(gps_controller):
    lat = gps_controller.get_latitude()
    lon = gps_controller.get_longitude()
    return pd.DataFrame({"latitude": [lat], "longitude": [lon]}).dropna(how="any")


def display_lidar_data(lidar):
    # Create a polar graph using Plotly
    fig = go.Figure(go.Scatterpolar(
            r=lidar.get_scan_data(),
            theta=lidar.get_angles(),
            mode='lines+markers',
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
    st.subheader("Lidar Scan Data")
    lidar_fig = display_lidar_data(navigation_service.lidar_controller)
    st.plotly_chart(lidar_fig)


@st.experimental_fragment(run_every=60)
def gps_widget():
    st.subheader("GPS Data")
    st.map(get_gps_data(navigation_service.gps_controller), size=2, zoom=17)
    st.subheader("Compass Data")
    st.write(f"Heading: {navigation_service.compass_controller.get_heading()}")


nav_col, compass_col, lidar_col = st.columns(3, gap="medium")

# Create buttons for drive methods
with nav_col:
    st.subheader("Drive Controls")
    if st.button("Drive Forward", key="fwd"):
        navigation_service.drive_forward(4096)  # Example speed value
    if st.button("Drive Backward", key="bwd"):
        navigation_service.drive_backward(4096)  # Example speed value
    if st.button("Drive Left", key="left"):
        navigation_service.drive_left(4000)  # Example speed value
    if st.button("Drive Right", key="right"):
        navigation_service.drive_right(4000)  # Example speed value
    if st.button("Stop Driving", key="stop"):
        navigation_service.stop_driving()

    # Display GPS and compass data in a collapsable container

with compass_col:
    gps_widget()

with lidar_col:
    lidar_widget()
