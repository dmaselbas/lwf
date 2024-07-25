# user_interface/app.py
import requests
import streamlit as st
import streamlit.components.v1 as components
import plotly.graph_objects as go
import pandas as pd
from devices.compass import HMC5883L
from devices.drive import DriveController
from devices.gps import GPSController
from devices.lidar import LidarController
from devices.pwm_controller import PWMController
from devices.imu import MPU6050
from servicess.camera_service import CameraService
from servicess.navigation_service import NavigationService

st.set_page_config(layout="wide")
st.session_state.gps_data = pd.DataFrame(columns=["lat", "lon"])
nav_cam_html = f'<iframe src="http://192.168.5.240:8080/?action=stream" width="100%" frameborder="0"></iframe>'


@st.cache_resource
def get_pwm_controller():
    return PWMController()


@st.cache_resource
def get_nav_service():
    lidar = LidarController()
    drive = DriveController(get_pwm_controller())
    gps = GPSController()
    imu = MPU6050()
    return NavigationService(drive, lidar, gps, imu)


@st.cache_resource
def get_compass():
    compass = HMC5883L()
    return compass


@st.cache_resource
def get_cams():
    return CameraService(get_pwm_controller())


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


@st.experimental_fragment(run_every=2)
def get_imu_widget():
    imu_data = navigation_service.imu.get_all_data_as_df()
    st.subheader("IMU Data")
    st.dataframe(imu_data)


nav_col, compass_col, lidar_col = st.columns(3, gap="small")

# Create buttons for drive methods
with nav_col:
    st.subheader("Drive Controls")
    components.iframe("http://192.168.5.240:8080/?action=stream", height=500)
    # components.iframe("	http://192.168.5.243/cgi-bin/hi3510/snap.cgi?&-getstream&-chn=2", height=500)
    speed = st.slider("Speed", min_value=0, max_value=4095, key="speed")
    navigation_service.drive_controller.set_speed(speed)
    pan_col, tilt_col = st.columns(2)
    with pan_col:
        pan_x = st.slider("Targeting Camera Pan",
                          min_value=4095,
                          max_value=0,
                          key="classification_cam_pan")
        get_cams().classification_cam.set_pan_position(pan_x)
        pan_y = st.slider("Nav Camera Pan",
                          min_value=4095,
                          max_value=0,
                          key="nav_cam_pan")
        get_cams().nav_cam.set_pan_position(pan_y)
    with tilt_col:
        tilt_x = st.slider("Targeting Camera Tilt",
                           min_value=4095,
                           max_value=0,
                           key="classification_cam_tilt")
        get_cams().classification_cam.set_tilt_position(tilt_x)
        tilt_y = st.slider("Nav Camera Tilt",
                           min_value=4095,
                           max_value=0,
                           key="nav_cam_tilt")
        get_cams().nav_cam.set_tilt_position(tilt_y)
    with st.container():
        left_buttons, center_buttons, right_buttons = st.columns(3)
        with left_buttons:
            st.empty()
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
            st.empty()
            if st.button("Right", key="right"):
                navigation_service.drive_right()  # Example speed value
with compass_col:
    with st.container():
        cas_btn, laser_button = st.columns(2)
        with cas_btn:
            if st.button("CAS On", key="cas_on"):
                navigation_service.collision_avoidance_system.turn_on()
            if st.button("CAS Off", key="cas_off"):
                navigation_service.collision_avoidance_system.turn_off()
        with laser_button:
            if st.button("Laser On", key="laser_on"):
                requests.post("http://192.168.5.242:5000/laser/on")
            if st.button("Laser Off", key="laser_off"):
                requests.post("http://192.168.5.242:5000/laser/off")
    with st.container():
        gps_widget()
    with st.container():
        get_imu_widget()
with lidar_col:
    lidar_widget()
    st.write(f"Collision Probability: {navigation_service.collision_avoidance_system.calculate_collision_probability() * 100}%")
    display_compass_heading()
