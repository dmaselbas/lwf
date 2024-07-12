import threading
import serial
import pynmea2


class GPSController:

    def __init__(self, on_update_callback=None):
        self.longitude = None
        self.latitude = None
        self.gps_data = None
        self.on_update_callback = on_update_callback
        self.gps = serial.Serial("/dev/ttyS2", baudrate=9600, timeout=10)
        self.process_thread = threading.Thread(target=self.process_gps_data)
        self.process_thread.start()

    def process_gps_data(self):
        while True:
            line = self.gps.readline().decode("utf-8", errors="ignore").strip()
            if line:
                try:
                    if "GGA" in line:  # RMC (Recommended Minimum Navigation Information)
                        msg = pynmea2.parse(line)
                        self.latitude = msg.latitude
                        self.longitude = msg.longitude
                        if self.on_update_callback:
                            self.on_update_callback({"latitude": msg.latitude, "longitude": msg.longitude})
                except pynmea2.nmea.ParseError:
                    print("Invalid GPS data")

    def get_latitude(self):
        return self.latitude

    def get_longitude(self):
        return self.longitude
