import smbus3 as smbus
import math

class HMC5883L:
    def __init__(self, bus_number=4, address=0x0D):
        self.bus = smbus.SMBus(bus_number)
        self.address = address

        # Set default configuration
        self.set_gain(0x00)  # +/- 0.88 Ga
        self.set_mode(0x00)  # Continuous-Measurement Mode

    def set_gain(self, gain):
        """Set the gain of the magnetometer."""
        self.bus.write_byte_data(self.address, 0x01, gain)

    def set_mode(self, mode):
        """Set the mode of the magnetometer."""
        self.bus.write_byte_data(self.address, 0x02, mode)

    def read_raw_data(self):
        """Read the raw magnetometer data."""
        data = self.bus.read_i2c_block_data(self.address, 0x03, 6)
        x = (data[0] << 8) | data[1]
        z = (data[2] << 8) | data[3]
        y = (data[4] << 8) | data[5]
        return x, y, z

    def read_magnetic_field(self):
        """Read the magnetic field strength in microtesla."""
        x, y, z = self.read_raw_data()

        # Convert raw data to magnetic field strength in µT
        # The conversion factors are based on the gain setting
        gain_factor = {
            0x00: 0.88,  # +/- 0.88 Ga
            0x20: 1.3,   # +/- 1.3 Ga
            0x40: 1.9,   # +/- 1.9 Ga
            0x60: 2.5,   # +/- 2.5 Ga
            0x80: 4.0,   # +/- 4.0 Ga
            0xA0: 4.7,   # +/- 4.7 Ga
            0xC0: 5.6,   # +/- 5.6 Ga
            0xE0: 8.1    # +/- 8.1 Ga
        }
        gain = self.bus.read_byte_data(self.address, 0x01) & 0xE0
        x_mgauss = x * gain_factor[gain]
        y_mgauss = y * gain_factor[gain]
        z_mgauss = z * gain_factor[gain]

        # Convert mgauss to µT
        x_ut = x_mgauss * 0.001
        y_ut = y_mgauss * 0.001
        z_ut = z_mgauss * 0.001

        return x_ut, y_ut, z_ut

    def get_heading(self):
        """Read the heading in degrees."""
        x, y, z = self.read_magnetic_field()

        if not x or not y or not z:
            return "Heading unavailable"
        # Calculate the heading angle using the arctangent function
        heading = math.degrees(math.atan2(y, x))

        # Adjust the heading angle to be within the range of 0 to 360 degrees
        if heading < 0:
            heading += 360

        return heading
