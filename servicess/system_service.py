import paho.mqtt.client as mqtt
import socket
import threading
import time
import os


class SystemService:

    def __init__(self):
        # Get the hostname
        hostname = socket.gethostname()

        # Initialize MQTT client
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        # Connect to the MQTT server
        self.client.connect("mqtt.weedfucker.local", 1883, 60)
        self.client.loop_start()

        # Subscribe to the topic
        self.topic = f"/sbc/syscmd/{hostname}"
        self.shutdown_topic = f"/sbc/syscmd/{hostname}"
        self.all_shutdown_topic = f"/sbc/syscmd/all"
        self.notification_topic = f"/sbc/system/notification/{hostname}"
        # Start the health check thread
        self.health_check_thread = threading.Thread(target=self.send_health_check)
        self.health_check_thread.daemon = True
        self.health_check_thread.start()

        self.running = True

    def shutdown(self):
        print("Shutting down SystemService...")
        self.client.publish(self.notification_topic, "shutting down")
        self.client.loop_stop()
        self.client.disconnect()
        self.running = False

    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code " + str(rc))
        client.subscribe(self.topic)
        client.subscribe(self.shutdown_topic)
        self.client.publish(self.notification_topic, "online")

    def on_message(self, client, userdata, msg):
        if msg.topic == self.shutdown_topic or msg.topic == self.all_shutdown_topic:
            print(f"Received {msg.payload.decode()} command.")
            if "shutdown" in msg.payload.decode():
                print("Shutting down the system.")
                os.system('shutdown -h now')
            if "restart" in msg.payload.decode():
                print("Restarting the system.")
                os.system('shutdown -r now')
            if "mount_storage" in msg.payload.decode():
                print("Attempting to mount share storage.")
                os.system('mount -a')

    def send_health_check(self):
        self.client.publish(self.notification_topic, "online")
        while self.running:
            self.client.publish("/sbc/system/health", "ping")
            time.sleep(5)


if __name__ == "__main__":
    system_service = SystemService()
    try:
        while system_service.running:
            pass  # Keep the main thread alive
    except KeyboardInterrupt:
        system_service.shutdown()
