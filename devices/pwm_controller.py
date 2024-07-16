import paho.mqtt.client as mqtt
import time
from smbus3 import SMBus


class PCA9685:
    def __init__(self, address=0x43, busnum=2):
        self.bus = SMBus(busnum)
        self.address = address
        self.PCA9685_MODE1 = 0x00
        self.PCA9685_MODE2 = 0x01
        self.PCA9685_PRESCALE = 0xFE
        self.LED0_ON_L = 0x06
        self.LED0_OFF_L = 0x08
        self.ALL_LED_ON_L = 0xFA
        self.ALL_LED_OFF_L = 0xFC
        self.MODE1_RESTART = 0x80
        self.MODE1_SLEEP = 0x10
        self.MODE1_AI = 0x20
        self.MODE2_OUTDRV = 0x04

        self._init_device()
        self.pwm_values = [0] * 16
    def _init_device(self):
        self.set_all_pwm(0, 0)
        self.bus.write_byte_data(self.address, self.PCA9685_MODE2, self.MODE2_OUTDRV)
        self.bus.write_byte_data(self.address, self.PCA9685_MODE1, self.MODE1_AI | self.MODE1_SLEEP)
        time.sleep(0.005)
        mode1 = self.bus.read_byte_data(self.address, self.PCA9685_MODE1)
        mode1 = mode1 & ~self.MODE1_SLEEP
        self.bus.write_byte_data(self.address, self.PCA9685_MODE1, mode1)
        time.sleep(0.005)
        self.set_pwm_freq(60)

    def set_pwm_freq(self, freq_hz):
        prescale_val = 25000000.0
        prescale_val /= 4096.0
        prescale_val /= float(freq_hz)
        prescale_val -= 1.0
        prescale = int(prescale_val + 0.5)

        old_mode = self.bus.read_byte_data(self.address, self.PCA9685_MODE1)
        new_mode = (old_mode & 0x7F) | self.MODE1_SLEEP
        self.bus.write_byte_data(self.address, self.PCA9685_MODE1, new_mode)
        self.bus.write_byte_data(self.address, self.PCA9685_PRESCALE, prescale)
        self.bus.write_byte_data(self.address, self.PCA9685_MODE1, old_mode)
        time.sleep(0.005)
        self.bus.write_byte_data(self.address, self.PCA9685_MODE1, old_mode | self.MODE1_RESTART)

    def set_pwm(self, channel, off, on=0):
        off = max(min(off, 4095), 0)
        self.bus.write_byte_data(self.address, self.LED0_ON_L + 4 * channel, on & 0xFF)
        self.bus.write_byte_data(self.address, self.LED0_ON_L + 4 * channel + 1, on >> 8)
        self.bus.write_byte_data(self.address, self.LED0_OFF_L + 4 * channel, off & 0xFF)
        self.bus.write_byte_data(self.address, self.LED0_OFF_L + 4 * channel + 1, off >> 8)
        self.pwm_values[channel] = off
    def set_all_pwm(self, on, off):
        self.bus.write_byte_data(self.address, self.ALL_LED_ON_L, on & 0xFF)
        self.bus.write_byte_data(self.address, self.ALL_LED_ON_L + 1, on >> 8)
        self.bus.write_byte_data(self.address, self.ALL_LED_OFF_L, off & 0xFF)
        self.bus.write_byte_data(self.address, self.ALL_LED_OFF_L + 1, off >> 8)


class PWMController:
    def __init__(self):
        self.pwm1 = PCA9685(address=0x40)
        self.pwm2 = PCA9685(address=0x43)

    def set_pwm(self, channel, value):
        try:
            if channel < 17:
                channel -= 1  # Adjust for zero-based indexing in the PCA9685
                self.pwm1.set_pwm(channel,  value)
            else:
                channel -= 17  # Adjust for zero-based indexing in the PCA9685
                self.pwm2.set_pwm(channel, value)
        except Exception as e:
            print(f"Error setting PWM value: {e}")
