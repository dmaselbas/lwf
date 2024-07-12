import time
from smbus3 import SMBus


class PCA9685:
    def __init__(self, bus_num=2, address=0x40):
        self.bus = SMBus(bus_num, force=True)
        self.address = address
        self._initialize()

    def _initialize(self):
        # Mode Register 1: Auto-increment enabled
        self.write_byte_data(0x00, 0x20)
        # Mode Register 2: Output logic state inverted
        self.write_byte_data(0x01, 0x04)
        # Prescale to set PWM frequency to 50Hz for servos
        self.set_pwm_freq(20)

    def write_byte_data(self, reg, value):
        self.bus.write_byte_data(self.address, reg, value, force=True)
        time.sleep(0.01)

    def set_pwm_freq(self, freq_hz):
        prescale_val = int(25000000.0 / (4096 * freq_hz) - 1)
        old_mode = self.bus.read_byte_data(self.address, 0x00)
        new_mode = (old_mode & 0x7F) | 0x10  # Sleep
        self.write_byte_data(0x00, new_mode)
        self.write_byte_data(0xFE, prescale_val)
        self.write_byte_data(0x00, old_mode)
        time.sleep(0.005)
        self.write_byte_data(0x00, old_mode | 0x80)

    def set_pwm(self, channel, off, on=0):
        print(f"Setting PWM {channel} to {on} (off={off})")
        self.write_byte_data(0x06 + 4 * channel, on & 0xFF)
        self.write_byte_data(0x07 + 4 * channel, on >> 8)
        self.write_byte_data(0x08 + 4 * channel, off & 0xFF)
        self.write_byte_data(0x09 + 4 * channel, off >> 8)

    def set_servo_angle(self, channel, angle):
        pulse_length = 1000000.0 / 50 / 4096  # 50Hz frequency and 12-bit resolution
        pulse = (angle * 2000 / 180 + 500) / pulse_length
        self.set_pwm(channel, 0, int(pulse))


class PWMController:
    def __init__(self):
        self.pwm_1 = PCA9685(address=0x43, bus_num=2)
        self.pwm_2 = PCA9685(address=0x40, bus_num=2)

    def set_pwm(self, channel, off_value):
        if channel < 17:
            self.pwm_1.set_pwm(channel - 1, off_value)
        else:
            self.pwm_2.set_pwm(channel - 17, off_value)
