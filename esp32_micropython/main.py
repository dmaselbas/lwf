import machine
import network
machine.freq(24000000)

from machine import UART, I2C, Pin
from hmc5883l import HMC5883L
from servo import Servos
from motor import DCMotors
from neopixel import NeoPixel
from mpu9250 import MPU9250

ros2_host = UART(1, 115200, tx=33, rx=32)
gps_uart = UART(2, 9600, tx=14, rx=12)

pwm_i2c = I2C(0, scl=Pin(18), sda=Pin(19), freq=400000)
imu_i2c = I2C(1, scl=Pin(22), sda=Pin(21), freq=400000)

compass_sensor = HMC5883L(scl=4, sda=5)

motor_pwm = DCMotors(i2c=pwm_i2c, address=0x40)
servo_controller = Servos(i2c=pwm_i2c, address=0x41)
indicator_leds = NeoPixel(Pin(16), 1)

imu = MPU9250(imu_i2c)

def do_connect():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print('connecting to network...')
        wlan.connect('monkeyballs', 'chucksucks123')
        while not wlan.isconnected():
            pass
    print('network config:', wlan.ipconfig('addr4'))


def send_ros2_data(sensor_prefix, data):
    ros2_host.write(f'{sensor_prefix} {data}\n')

def send_ros2_ack():
    ros2_host.write('ack\n')
def send_compass_data():
    x, y, z = compass_sensor.read()
    data = compass_sensor.format_result(x, y, z)
    send_ros2_data('compass', data)

def send_imu_data():
    send_ros2_data("imu_accel_xyz", imu.accel.xyz)
    send_ros2_data("imu_gyro_xyz", imu.gyro.xyz)
    send_ros2_data("imu_mag_xyz", imu.mag.xyz)
    send_ros2_data("imu_temperature", imu.temperature)
    send_ros2_data("imu_accel_z", imu.accel.z)

def on_motor_update(channel, speed):
    motor_pwm.set_speed(channel, speed)
    send_ros2_ack()

def on_servo_update(channel, angle):
    servo_controller.set_angle(channel, angle)
    send_ros2_ack()

