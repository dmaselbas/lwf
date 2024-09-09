import rclpy
from rclpy.node import Node

from pwm_driver_interface.srv import SetPwmDutyCycle

class LwfPwmDriver(Node):

    def __init__(self):
        super().__init__('lwf_pwm_driver')
        self.svc = self.create_service(SetPwmDutyCycle, 'set_pwm_duty_cycle', self.set_pwm_duty_cycle_callback)

    def set_pwm_duty_cycle_callback(self, req, resp):
        return resp



def main():
    rclpy.init()
    pwm_driver = LwfPwmDriver()
    rclpy.spin(pwm_driver)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
