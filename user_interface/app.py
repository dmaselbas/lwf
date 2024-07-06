# user_interface/app.py

import streamlit as st
from servicess.navigation_service import NavigationService

# Initialize the NavigationService instance
navigation_service = NavigationService()

# Create a new Streamlit tab
st.tab("Navigation Controller")

# Create a layout with buttons for each directional method
col1, col2, col3, col4 = st.columns(4)

# Add a video component to the left of the direction buttons
with col1:
    st.video(src="http://<raspberry_pi_ip>:8000/video.mjpg", format="video/mjpg")

with col2:
    if st.button("Left"):
        navigation_service.move_left()

with col3:
    if st.button("Up"):
        navigation_service.set_speed(50)

with col4:
    if st.button("Right"):
        navigation_service.move_right()

with st.expander("Advanced Controls"):
    col5, col6 = st.columns(2)

    with col5:
        speed = st.slider("Speed", min_value=0, max_value=100, value=0)
        if st.button("Set Speed"):
            navigation_service.set_speed(speed)

    with col6:
        if st.button("Stop"):
            navigation_service.stop()

# Run the app
if __name__ == "__main__":
    st.title("Weed Fucker 5000")
    st.sidebar.title("Navigation")
    st.sidebar.subheader("Control the robot's movement")
