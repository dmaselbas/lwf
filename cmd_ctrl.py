import streamlit as st
import streamlit.components.v1 as components
import plotly.graph_objects as go
import pandas as pd

from devices.drive import DriveClient
from devices.gps import GPSClient
from devices.laser import LaserClient
from devices.lidar import Lidar, LidarController
from devices.pwm_controller import PWMClient
from servicess.camera_service import CameraService
from servicess.navigation_service import NavigationService

st.set_page_config(layout="wide")
st.session_state.gps_data = pd.DataFrame(columns=["lat", "lon"])
laser_cam_html = f'<iframe src="http://192.168.5.242:8080/?action=stream" width="100%" height=500 frameborder="0"></iframe>'
nav_cam_html = f'<iframe src="http://192.168.5.243/cgi-bin/hi3510/snap.cgi?&-getstream&-chn=2" width="100%" height=500 frameborder="0"></iframe>'

# nav_cam_html = f'<iframe src="http://192.168.5.243/web/index.html" width="100%" frameborder="0"></iframe>'

@st.cache_resource
def get_pwm_controller():
    return PWMClient()


@st.cache_resource
def get_lidar_controller():
    return Lidar()

@st.cache_resource
def get_nav_service():
    return NavigationService(lidar)


@st.cache_resource
def get_cams():
    return CameraService()


@st.cache_resource
def get_gps():
    return GPSClient()

@st.cache_resource
def get_laser():
    return LaserClient()


@st.cache_resource
def get_drive():
    return DriveClient()


def get_gps_data(gps: GPSClient) -> pd.DataFrame:
    return gps.get_gps_reading()


def display_compass_heading():
    compass_heading = navigation_service.compass.get_bearing()
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
    lidar_last_reading_df = lidar.get_latest_data()
    fig = go.Figure(go.Scatterpolar(
            r=lidar_last_reading_df["distance"],
            theta=lidar_last_reading_df["angle"],
            mode='lines+markers',
            marker=dict(
                    color=lidar_last_reading_df["quality"],
                    colorscale='Viridis'
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


pwm_controller = get_pwm_controller()
cams = get_cams()
gps_controller = get_gps()
laser = get_laser()
drive = get_drive()
lidar = get_lidar_controller()
navigation_service = get_nav_service()


@st.experimental_fragment(run_every=5)
def lidar_widget():
    lidar_fig = display_lidar_data()
    st.plotly_chart(lidar_fig)


@st.experimental_fragment(run_every=10)
def gps_widget():
    st.subheader("GPS Data")
    gps_data = get_gps_data(gps_controller)
    if len(gps_data) > 0:
        st.map(gps_data, size=2, zoom=18, use_container_width=True)
        st.write(navigation_service.compass.get_bearing())
        st.empty()


@st.experimental_fragment(run_every=2)
def get_imu_widget():
    imu_data = navigation_service.imu.get_all_data_as_df()
    st.subheader("IMU Data")
    st.dataframe(imu_data)


def on_speed_change():
    speed = st.session_state.speed
    drive.set_speed(speed)


def on_pan_change():
    pan_x = st.session_state.classification_cam_pan
    pan_y = st.session_state.nav_cam_pan
    cams.classification_cam.set_pan_position(pan_x)
    cams.nav_cam.set_pan_position(pan_y)


def on_tilt_change():
    tilt_x = st.session_state.classification_cam_tilt
    tilt_y = st.session_state.nav_cam_tilt
    cams.classification_cam.set_tilt_position(tilt_x)
    cams.nav_cam.set_tilt_position(tilt_y)


def on_laser_position_change():
    laser_position = st.session_state.laser_position
    laser.move_to_position(laser_position)


nav_col, compass_col, lidar_col = st.columns(3, gap="small")

with nav_col:
    st.subheader("Drive Controls")
    components.iframe("http://192.168.5.243/cgi-bin/hi3510/snap.cgi?&-getstream&-chn=2", width=600, height=400)
    speed = st.slider("Speed", min_value=0, max_value=100, key="speed", on_change=on_speed_change)
    pan_col, tilt_col = st.columns(2)
    with pan_col:
        pan_x = st.slider("Targeting Camera Pan",
                          min_value=0,
                          max_value=180,
                          key="classification_cam_pan",
                          on_change=on_pan_change)
        pan_y = st.slider("Nav Camera Pan", min_value=0, max_value=180, key="nav_cam_pan", on_change=on_pan_change)
    with tilt_col:
        tilt_x = st.slider("Targeting Camera Tilt",
                           min_value=0,
                           max_value=180,
                           key="classification_cam_tilt",
                           on_change=on_tilt_change)
        tilt_y = st.slider("Nav Camera Tilt", min_value=0, max_value=180, key="nav_cam_tilt", on_change=on_tilt_change)
    with st.container():
        left_buttons, center_buttons, right_buttons = st.columns(3)
        with left_buttons:
            st.button("", key="blank1")
            if st.button("Left", key="left"):
                drive.left()
        with center_buttons:
            if st.button("Forward", key="fwd"):
                drive.forward()
            if st.button("Stop", key="stop"):
                drive.stop()
            if st.button("Backward", key="bwd"):
                drive.reverse()
        with right_buttons:
            st.button("", key="blank2")
            if st.button("Right", key="right"):
                drive.right()
with compass_col:
    with st.container():
        cas_btn, laser_move, laser_button = st.columns(3)
        with cas_btn:
            if st.button("CAS On", key="cas_on"):
                navigation_service.cas.turn_on()
            if st.button("CAS Off", key="cas_off"):
                navigation_service.cas.turn_off()
        with laser_move:
            if st.button("Enable Laser Cycle", key="laser_cycle_on"):
                laser.enable_cycle_motion()
            if st.button("Disable Laser Cycle", key="laser_cycle_off"):
                laser.disable_cycle_motion()
            st.select_slider("Laser Movement", options=range(0, 1000), key="laser_position", on_change=on_laser_position_change)
        with laser_button:
            if st.button("Laser On", key="laser_on"):
                laser.turn_on()
            if st.button("Laser Off", key="laser_off"):
                laser.turn_off()
            if st.button("Laser Pulse", key="laser_pulse"):
                laser.pulse_laser()
            if st.button("Motor On", key="laser_motor_on"):
                laser.motor_start()
            if st.button("Motor Off", key="laser_motor_off"):
                laser.motor_stop()
    with st.container():
        lidar_widget()
    with st.container():
        get_imu_widget()
with lidar_col:
    gps_widget()
    # st.write(f"Collision Probability: {navigation_service.cas.calculate_collision_probability() * 100}%")
    display_compass_heading()
    # components.iframe("http://192.168.5.242:8080/?action=stream", width=1280, height=720)
