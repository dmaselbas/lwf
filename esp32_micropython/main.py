import machine
import network

machine.freq(24000000)

from datetime import timedelta
from machine import UART, I2C, Pin, WDT, RTC, PWM
from lwf_libs.hmc5883l import HMC5883L
from neopixel import NeoPixel
from lwf_libs.mpu9250 import MPU9250
from lwf_libs.hcsr04 import HCSR04

watchdog = WDT(timeout=2000)

ros2_host = UART(0, 115200, tx=1, rx=3)

pwm_i2c = I2C(0, scl=Pin(18), sda=Pin(19), freq=400000)

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

def on_data(data):
    # Check if data starts with 'L' and contains 'R'
    if data.startswith("L") and "R" in data:
        try:
            # Extract duty cycle values
            left_value = data.split("L")[1].split("R")[0]
            right_value = data.split("R")[1]

            # Convert to integer
            left_duty_cycle = int(left_value)
            right_duty_cycle = int(right_value)

            # Clamp values between 0 and 1023
            left_duty_cycle = max(0, min(1023, left_duty_cycle))
            right_duty_cycle = max(0, min(1023, right_duty_cycle))

            # Set direction and absolute duty cycle for left motor
            if left_duty_cycle >= 0:
                front_left_motor_dir.value(1)  # Forward direction
                rear_left_motor_dir.value(1)  # Forward direction
                front_left_motor_pwm.duty_u16(int(left_duty_cycle * 64))  # Scale for 16-bit PWM
                rear_left_motor_pwm.duty_u16(int(left_duty_cycle * 64))  # Scale for 16-bit PWM
            else:
                front_left_motor_dir.value(0)  # Forward direction
                rear_left_motor_dir.value(0)  # Forward direction
                front_left_motor_pwm.duty_u16(int(abs(left_duty_cycle * 64)))  # Scale for 16-bit PWM
                rear_left_motor_pwm.duty_u16(int(abs(left_duty_cycle * 64)))  # Scale for 16-bit PWM

            # Set direction and absolute duty cycle for right motor
            if right_duty_cycle >= 0:
                front_right_motor_dir.value(1)  # Forward direction
                rear_right_motor_dir.value(1)  # Forward direction
                front_right_motor_pwm.duty_u16(int(right_duty_cycle * 64))  # Scale for 16-bit PWM
                rear_right_motor_pwm.duty_u16(int(right_duty_cycle * 64))  # Scale for 16-bit PWM
            else:
                front_right_motor_dir.value(0)  # Forward direction
                rear_right_motor_dir.value(0)  # Forward direction
                front_right_motor_pwm.duty_u16(int(abs(right_duty_cycle * 64)))  # Scale for 16-bit PWM
                rear_right_motor_pwm.duty_u16(int(abs(right_duty_cycle * 64)))  # Scale for 16-bit PWM

        except ValueError:
                # Handle any conversion errors
                print("Invalid data format")

# Interrupt handler for UART
def uart_interrupt_handler(uart):
    # Read the incoming line
    data = uart.readline()
    if data:
        # Decode and parse the data
        data_str = data.decode().strip()
        on_data(data_str)

def send_compass_data():
    x, y, z = compass_sensor.read()
    degrees, mins = compass_sensor.read_heading()
    send_ros2_data('compass',f"{x}|{y}|{z}|{degrees}|{mins}")


def send_ping_data():
    send_ros2_data("sonar_left", f"{left_sonar_sensor.distance_mm()}")
    send_ros2_data("sonar_back", f"{back_sonar_sensor.distance_mm()}")
    send_ros2_data("sonar_right", f"{right_sonar_sensor.distance_mm()}")


def update_indicator_leds():
    pass

def report_slow_frame():
    send_ros2_data("slow_frame", "1")

ros2_host.irq(handler=uart_interrupt_handler, trigger=UART.IRQ_RXIDLE)

while True:
    watchdog.feed()  # Feed the watchdog to prevent reset
    send_compass_data()
    send_ping_data()
    update_indicator_leds()
