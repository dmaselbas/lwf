import atexit
import time
import pandas as pd
from datetime import datetime
from time import sleep

import requests
import smbus3 as smbus
import math
from logging import getLogger
import paho.mqtt.client as mqtt
import threading


class CompassClient:

    def __init__(self):
        self.bearing = 0.0
        self.temperature = 0.0

        self.mqtt_client = mqtt.Client(protocol=mqtt.MQTTv5)
        self.mqtt_client.on_connect = self._on_connect
        self.mqtt_client.on_message = self._on_message
        self.mqtt_client.connect("mqtt.weedfucker.local", 1883, 60)
        self.mqtt_client.loop_start()

    def _on_connect(self, client, userdata, flags, rc, properties):
        client.subscribe("/dev/compass/#")

    def _on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode('utf-8')

        if topic == "/dev/compass/bearing":
            self.bearing = float(payload)
        elif topic == "/dev/compass/temperature":
            self.temperature = float(payload)

    def get_bearing(self):
        return self.bearing

    def get_temp(self):
        return self.temperature


class HMC5883L:
    DFLT_BUS = 4
    DFLT_ADDRESS = 0x0d

    REG_XOUT_LSB = 0x00  # Output Data Registers for magnetic sensor.
    REG_XOUT_MSB = 0x01
    REG_YOUT_LSB = 0x02
    REG_YOUT_MSB = 0x03
    REG_ZOUT_LSB = 0x04
    REG_ZOUT_MSB = 0x05
    REG_STATUS_1 = 0x06  # Status Register.
    REG_TOUT_LSB = 0x07  # Output Data Registers for temperature.
    REG_TOUT_MSB = 0x08
    REG_CONTROL_1 = 0x09  # Control Register #1.
    REG_CONTROL_2 = 0x0a  # Control Register #2.
    REG_RST_PERIOD = 0x0b  # SET/RESET Period Register.
    REG_CHIP_ID = 0x0d  # Chip ID register.

    # Flags for Status Register #1.
    STAT_DRDY = 0b00000001  # Data Ready.
    STAT_OVL = 0b00000010  # Overflow flag.
    STAT_DOR = 0b00000100  # Data skipped for reading.

    # Flags for Status Register #2.
    INT_ENB = 0b00000001  # Interrupt Pin Enabling.
    POL_PNT = 0b01000000  # Pointer Roll-over.
    SOFT_RST = 0b10000000  # Soft Reset.

    # Flags for Control Register 1.
    MODE_STBY = 0b00000000  # Standby mode.
    MODE_CONT = 0b00000001  # Continuous read mode.
    ODR_10HZ = 0b00000000  # Output Data Rate Hz.
    ODR_50HZ = 0b00000100
    ODR_100HZ = 0b00001000
    ODR_200HZ = 0b00001100
    RNG_2G = 0b00000000  # Range 2 Gauss: for magnetic-clean environments.
    RNG_8G = 0b00010000  # Range 8 Gauss: for strong magnetic fields.
    OSR_512 = 0b00000000  # Over Sample Rate 512: less noise, more power.
    OSR_256 = 0b01000000
    OSR_128 = 0b10000000
    OSR_64 = 0b11000000  # Over Sample Rate 64: more noise, less power.

    def __init__(self,
                 i2c_bus=DFLT_BUS,
                 address=DFLT_ADDRESS,
                 output_data_rate=ODR_10HZ,
                 output_range=RNG_2G,
                 oversampling_rate=OSR_512):

        self.running = True
        self.logging = getLogger(__name__)
        self.address = address
        self.bus = smbus.SMBus(int(i2c_bus))
        self.output_range = output_range
        self._declination = 0.0
        self._calibration = [[1.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0],
                             [0.0, 0.0, 1.0]]
        chip_id = self._read_byte(self.REG_CHIP_ID)
        if chip_id != 0xff:
            msg = "Chip ID returned 0x%x instead of 0xff; is the wrong chip?"
            self.logging.warning(msg, chip_id)
        self.mode_cont = (self.MODE_CONT | output_data_rate | output_range
                          | oversampling_rate)
        self.mode_stby = (self.MODE_STBY | self.ODR_10HZ | self.RNG_2G | self.OSR_64)
        self.mode_continuous()
        self.readings: pd.DataFrame = None
        self.thread = threading.Thread(target=self.run)
        self.thread.daemon = True
        self.thread.start()

        # Initialize MQTT client
        self.mqtt_client = mqtt.Client(protocol=mqtt.MQTTv5)
        self.mqtt_client.connect("mqtt.weedfucker.local", 1883, 60)
        self.mqtt_client.loop_forever()

        atexit.register(self._shutdown)

    import requests

    def get_declination(self):
        # NOAA API endpoint for magnetic declination
        url = "https://www.ngdc.noaa.gov/geomag-web/calculators/calculateDeclination"
        latitude = 39.7392
        longitude = -104.9903
        now = datetime.now().strftime("%Y-%m-%d")
        # Parameters
        params = {
            "latitude":  latitude,
            "longitude": longitude,
            "date":      now,
            "format":    "json"
            }

        # Make the request
        response = requests.get(url, params=params)

        # Parse the JSON response
        if response.status_code == 200:
            data = response.json()
            declination = data['result'][0]['declination']
            return declination
        else:
            print(f"Error: {response.status_code}")
            return None

    def _shutdown(self):
        if self.running:
            print("Stopping Compas...")
            self.running = False
            self.mode_standby()
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()

    def __del__(self):
        """Once finished using the sensor, switch to standby mode."""
        self._shutdown()

    def mode_continuous(self):
        """Set the device in continuous read mode."""
        self._write_byte(self.REG_CONTROL_2, self.SOFT_RST)  # Soft reset.
        self._write_byte(self.REG_CONTROL_2, self.INT_ENB)  # Disable interrupt.
        self._write_byte(self.REG_RST_PERIOD, 0x01)  # Define SET/RESET period.
        self._write_byte(self.REG_CONTROL_1, self.mode_cont)  # Set operation mode.

    def mode_standby(self):
        """Set the device in standby mode."""
        self._write_byte(self.REG_CONTROL_2, self.SOFT_RST)
        self._write_byte(self.REG_CONTROL_2, self.INT_ENB)
        self._write_byte(self.REG_RST_PERIOD, 0x01)
        self._write_byte(self.REG_CONTROL_1, self.mode_stby)  # Set operation mode.

    def _write_byte(self, registry, value):
        self.bus.write_byte_data(self.address, registry, value)
        sleep(0.01)

    def _read_byte(self, registry):
        return self.bus.read_byte_data(int(self.address), registry)

    def _read_word(self, registry):
        """Read a two bytes value stored as LSB and MSB."""
        low = self.bus.read_byte_data(self.address, registry)
        high = self.bus.read_byte_data(self.address, registry + 1)
        val = (high << 8) + low
        return val

    def _read_word_2c(self, registry):
        """Calculate the 2's complement of a two bytes value."""
        val = self._read_word(registry)
        if val >= 0x8000:  # 32768
            return val - 0x10000  # 65536
        else:
            return val

    def get_data(self):
        """Read data from magnetic and temperature data registers."""
        i = 0
        [x, y, z, t] = [None, None, None, None]
        while i < 20:  # Timeout after about 0.20 seconds.
            status = self._read_byte(self.REG_STATUS_1)
            if status & self.STAT_OVL:
                # Some values have reached an overflow.
                msg = ("Magnetic sensor overflow.")
                if self.output_range == self.RNG_2G:
                    msg += " Consider switching to RNG_8G output range."
                self.logging.warning(msg)
            if status & self.STAT_DOR:
                # Previous measure was read partially, sensor in Data Lock.
                x = self._read_word_2c(self.REG_XOUT_LSB)
                y = self._read_word_2c(self.REG_YOUT_LSB)
                z = self._read_word_2c(self.REG_ZOUT_LSB)
                continue
            if status & self.STAT_DRDY:
                # Data is ready to read.
                x = self._read_word_2c(self.REG_XOUT_LSB)
                y = self._read_word_2c(self.REG_YOUT_LSB)
                z = self._read_word_2c(self.REG_ZOUT_LSB)
                t = self._read_word_2c(self.REG_TOUT_LSB)
                break
            else:
                # Waiting for DRDY.
                time.sleep(0.01)
                i += 1
        return [x, y, z, t]

    def get_magnet_raw(self):
        """Get the 3 axis values from magnetic sensor."""
        [x, y, z, t] = self.get_data()
        return [x, y, z]

    def get_magnet(self):
        """Return the horizontal magnetic sensor vector with (x, y) calibration applied."""
        [x, y, z] = self.get_magnet_raw()
        if x is None or y is None:
            [x1, y1] = [x, y]
        else:
            c = self._calibration
            x1 = x * c[0][0] + y * c[0][1] + c[0][2]
            y1 = x * c[1][0] + y * c[1][1] + c[1][2]
        return [x1, y1]

    def get_bearing_raw(self):
        """Horizontal bearing (in degrees) from magnetic value X and Y."""
        [x, y, z] = self.get_magnet_raw()
        if x is None or y is None:
            return None
        else:
            b = math.degrees(math.atan2(y, x))
            if b < 0:
                b += 360.0
            return b

    def get_bearing(self):
        """Horizontal bearing, adjusted by calibration and declination."""
        if self._declination is None:
            declination = self.get_declination()
            if declination:
                self.set_declination(declination)
        [x, y] = self.get_magnet()
        if x is None or y is None:
            return None
        else:
            b = math.degrees(math.atan2(y, x))
            if b < 0:
                b += 360.0
            b += self._declination
            if b < 0.0:
                b += 360.0
            elif b >= 360.0:
                b -= 360.0
        return b

    def get_temp(self):
        """Raw (uncalibrated) data from temperature sensor."""
        [x, y, z, t] = self.get_data()
        if t is not None:
            # Convert the raw temperature to Celsius (assuming the raw value is in Celsius)
            temp_celsius = t / 100.0  # Adjust this conversion factor as needed
            # Convert Celsius to Fahrenheit
            temp_fahrenheit = (temp_celsius * 9.0 / 5.0) + 32.0
            return temp_fahrenheit
        return None

    def set_declination(self, value):
        """Set the magnetic declination, in degrees."""
        try:
            d = float(value)
            if d < -180.0 or d > 180.0:
                self.logging.error(u'Declination must be >= -180 and <= 180.')
            else:
                self._declination = d
        except:
            self.logging.error(u'Declination must be a float value.')

    def get_declination(self):
        """Return the current set value of magnetic declination."""
        return self._declination

    def set_calibration(self, value):
        """Set the 3x3 matrix for horizontal (x, y) magnetic vector calibration."""
        c = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        try:
            for i in range(0, 3):
                for j in range(0, 3):
                    c[i][j] = float(value[i][j])
            self._calibration = c
        except:
            self.logging.error(u'Calibration must be a 3x3 float matrix.')

    def get_calibration(self):
        """Return the current set value of the calibration matrix."""
        return self._calibration

    declination = property(fget=get_declination,
                           fset=set_declination,
                           doc=u'Magnetic declination to adjust bearing.')

    calibration = property(fget=get_calibration,
                           fset=set_calibration,
                           doc=u'Transformation matrix to adjust (x, y) magnetic vector.')

    def run(self):
        while self.running:
            bearing = self.get_bearing()
            temp = self.get_temp()
            ts = datetime.utcnow()
            df = pd.DataFrame.from_records([{"timestamp": ts, "bearing": bearing, "temp": temp}])
            self.readings = df if self.readings is None else pd.concat([self.readings, df]).reset_index(drop=True).sort_values(by="timestamp")
            self.readings = self.readings.tail(25)  # Keep only the last 10
            if bearing is not None:
                self.mqtt_client.publish("/dev/compass/bearing", self.readings["bearing"].ewm(alpha=0.1).mean().values[-1], retain=True)
            if temp is not None:
                self.mqtt_client.publish("/dev/compass/temp", self.readings["temp"].ewm(alpha=0.1).mean().values[-1], retain=True)
            time.sleep(1)


if __name__ == "__main__":
    compass = HMC5883L()
