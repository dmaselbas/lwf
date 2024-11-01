import machine
import network

machine.freq(24000000)

from datetime import timedelta
from machine import UART, I2C, Pin, WDT, RTC, PWM
from lwf_libs.hmc5883l import HMC5883L
from neopixel import NeoPixel
from lwf_libs.mpu9250 import MPU9250
from lwf_libs.hcsr04 import HCSR04
from lwf_libs.pca9685 import PCA9685, Servo

clock = RTC()
watchdog = WDT(timeout=2000)

ros2_host = UART(0, 115200, tx=1, rx=3)
# gps_uart = UART(2, 9600, tx=Pin(14), rx=Pin(12))

pwm_i2c = I2C(0, scl=Pin(18), sda=Pin(19), freq=400000)
imu_i2c = I2C(1, scl=Pin(22), sda=Pin(21), freq=400000)

compass_sensor = HMC5883L(scl=4, sda=5)

front_left_motor_dir = Pin(35, Pin.OUT)
rear_left_motor_dir = Pin(33, Pin.OUT)
front_right_motor_dir =Pin(22, Pin.OUT)
rear_right_motor_dir = Pin(14, Pin.OUT)

front_left_motor_pwm = PWM(Pin(34), freq=50, duty_u16=0)
rear_left_motor_pwm = PWM(Pin(32), freq=50, duty_u16=0)
front_right_motor_pwm = PWM(Pin(25), freq=50, duty_u16=0)
rear_right_motor_pwm = PWM(Pin(27), freq=50, duty_u16=0)

servo_tilt_pwm = PWM(Pin(17), freq=50, duty_u16=0)
servo_pan_pwm = PWM(Pin(18), freq=50, duty_u16=0)

status_led = NeoPixel(Pin(16), 1)
indicator_leds = NeoPixel(Pin(17), 48)

imu = MPU9250(imu_i2c)

# Sonar setup
front_ping_pin = machine.Pin(17, machine.Pin.OUT)
front_pong_pin = machine.Pin(17, machine.Pin.IN)
left_ping_pin = machine.Pin(14, machine.Pin.OUT)
left_pong_pin = machine.Pin(12, machine.Pin.IN)
back_ping_pin = machine.Pin(17, machine.Pin.OUT)
back_pong_pin = machine.Pin(17, machine.Pin.IN)
right_ping_pin = machine.Pin(17, machine.Pin.OUT)
right_pong_pin = machine.Pin(17, machine.Pin.IN)

front_sonar_sensor = HCSR04(trigger_pin=front_ping_pin, echo_pin=front_pong_pin)
left_sonar_sensor = HCSR04(trigger_pin=left_ping_pin, echo_pin=left_pong_pin)
back_sonar_sensor = HCSR04(trigger_pin=back_ping_pin, echo_pin=back_pong_pin)
right_sonar_sensor = HCSR04(trigger_pin=right_ping_pin, echo_pin=right_pong_pin)

front_ping_distance = 0
left_ping_distance = 0
back_ping_distance = 0
right_ping_distance = 0


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
    ros2_host.write(f'{sensor_prefix}|{data}\n')

def on_data():
   msg = ros2_host.readline().decode().strip()
   if msg.startswith('vel'):
       val = msg.split('|')[1]
       val = float(val)
       forward = True
       if val < 0:
           forward = False
        val = abs(val) * 4095

       front_left_motor_pwm.duty(int(val * 1023))
       front_right_motor_pwm.duty(int(val * 1023))
       rear_left_motor_pwm.duty(int(val * 1023))
       rear_right_motor_pwm.duty(int(val * 1023))

def send_compass_data():
    x, y, z = compass_sensor.read()
    degrees, mins = compass_sensor.read_heading()
    send_ros2_data('compass',f"{x}|{y}|{z}|{degrees}|{mins}")


def send_imu_data():
    imu.update()
    send_ros2_data("imu_accel_xyz", imu.accel.xyz)
    send_ros2_data("imu_gyro_xyz", imu.gyro.xyz)
    send_ros2_data("imu_mag_xyz", imu.mag.xyz)
    send_ros2_data("imu_temperature", imu.temperature)
    send_ros2_data("imu_accel_z", imu.accel.z)


def send_ping_data():
    # send_ros2_data("sonar_front", f"{front_sonar_sensor.distance_mm()}")
    send_ros2_data("sonar_left", f"{left_sonar_sensor.distance_mm()}")
    send_ros2_data("sonar_back", f"{back_sonar_sensor.distance_mm()}")
    send_ros2_data("sonar_right", f"{right_sonar_sensor.distance_mm()}")


def update_indicator_leds():
    pass

def report_slow_frame():
    send_ros2_data("slow_frame", "1")

ros2_host.irq(handler=on_data, trigger=UART.IRQ_RXIDLE)
while True:
    watchdog.feed()  # Feed the watchdog to prevent reset
    # send_imu_data()
    send_compass_data()
    send_ping_data()
    update_indicator_leds()
