# user_interface/app.py


import streamlit as st
from servicess.navigation_service import NavigationService

# Initialize the NavigationService instance
navigation_service = NavigationService(mqtt_broker_address="192.168.4.59")

# Create a layout with buttons for each directional method
col1, col2, col3, col4 = st.columns(4)

# Add a video component to the left of the direction buttons
with col1:
    pass
    # st.video("http://192.168.4.59:8080/stream?advanced_headers=1", format="video/mjpg", autoplay=True)

with col2:
    if st.button("Left"):
        navigation_service.move_left()

with col3:
    if st.button("Up"):
        navigation_service.set_speed(50)

with col4:
    if st.button("Right"):
        navigation_service.move_right()

with st.container():
    col5, col6 = st.columns(2)

    with col5:
        speed = st.slider("Speed", min_value=0, max_value=4095, value=0)
        if st.button("Set Speed"):
            navigation_service.set_speed(speed)

    with col6:
        if st.button("Stop"):
            navigation_service.stop()

# Run the app
if __name__ == "__main__":
    st.sidebar.title("Navigation")
    st.sidebar.subheader("Drive")
