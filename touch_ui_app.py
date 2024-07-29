import streamlit as st
import paho.mqtt.client as mqtt
# Set the page configuration
st.set_page_config(
    layout="centered",  # Use "centered" layout for small screens
    initial_sidebar_state="collapsed",  # Collapse the sidebar initially
)

mqtt_client = mqtt.Client()
mqtt_client.connect("mqtt.weedfucker.local", 1883, 60)
mqtt_client.loop_start()

def send_mqtt_message(topic, message):
    mqtt_client.publish(topic, message)

st.set_page_config(
    page_title="System Control",
    layout="centered",  # Use "centered" layout for small screens
    initial_sidebar_state="collapsed",  # Collapse the sidebar initially
)

shutdown_col, restart_col = st.columns(2, height=300, width=300, vertical=True, border=False)
with st.container(height=320):
    with restart_col:
        st.subheader("Restart Controls")
        if st.button("Restart Navigation System"):
            send_mqtt_message("/sbc/syscmd/nav-lwf", "restart")
            st.success("Restart command sent to Navigation System.")

        if st.button("Restart AI System"):
            send_mqtt_message("/sbc/syscmd/ai-lwf", "restart")
            st.success("Restart command sent to AI System.")

        if st.button("Restart UI System a.k.a this one"):
            send_mqtt_message("/sbc/syscmd/ui-lwf", "restart")
            st.success("Restart command sent to UI System.")

        if st.button("Restart All Systems"):
            send_mqtt_message("/sbc/syscmd/all", "restart")
            st.success("Restart command sent to All Systems.")
    with shutdown_col:
        st.subheader("Shutdown Controls")
        if st.button("Shutdown Navigation System"):
            send_mqtt_message("/sbc/syscmd/nav-lwf", "shutdown")
            st.success("Shutdown command sent to Navigation System.")

        if st.button("Shutdown AI System"):
            send_mqtt_message("/sbc/syscmd/ai-lwf", "shutdown")
            st.success("Shutdown command sent to AI System.")

        if st.button("Shutdown UI System a.k.a this one"):
            send_mqtt_message("/sbc/syscmd/ui-lwf", "shutdown")
            st.success("Shutdown command sent to UI System.")

        if st.button("Shutdown All Systems"):
            send_mqtt_message("/sbc/syscmd/all", "shutdown")
            st.success("Shutdown command sent to All Systems.")
