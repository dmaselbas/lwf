import threading
import time

from rplidar import RPLidar
import paho.mqtt.client as mqtt

class LidarController:

    def __init__(self, on_update_callback=None):
        self.callback = on_update_callback
        self.publish_thread = None
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect("10.0.0.2", 1883)  # Replace with your MQTT broker address and port

        if on_update_callback is not None:
            self.mqtt_client.on_message = self.on_message
            self.mqtt_client.subscribe("/dev/lidar/update")

        if on_update_callback is None:
            self.lidar = RPLidar("/dev/ttyUSB0")
            self.publish_thread = threading.Thread(target=self.publish_lidar_data)
            self.publish_thread.start()

    def __del__(self):
        if self.lidar is not None:
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.disconnect()

        if self.publish_thread is not None:
            self.publish_thread.join()

        self.mqtt_client.disconnect()

    def on_message(self, client, userdata, message):
        if self.callback is not None:
            scan_data = message.payload.decode()
            self.callback(scan_data)

    def publish_lidar_data(self):
        last_health_time = time.time()

        while True:
            try:
                for scan in self.lidar.iter_scans():
                    # Process and publish the scan data
                    # Convert the scan data to a string or JSON format
                    scan_data = str(scan)  # Example: convert to string

                    # Publish the scan data to the MQTT topic
                    self.mqtt_client.publish("/dev/lidar/update", scan_data)
                current_time = time.time()
                if current_time - last_health_time >= 30:
                    # Get lidar health and publish it every 30 seconds
                    health_data = self.lidar.get_health()
                    self.mqtt_client.publish("/dev/lidar/health", str(health_data))
                    last_health_time = current_time

            except Exception as e:
                print(f"Error publishing lidar data: {e}")


            time.sleep(1)
