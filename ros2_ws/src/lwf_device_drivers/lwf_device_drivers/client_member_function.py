import sys
from http.client import responses

import rclpy
from rclpy.node import Node
from pwm_driver_interface.srv import SetPwmDutyCycle
class LwfPwmDriverClientAsync(Node):

    def __init__(self):
        super().__init__('lwf_pwm_driver_client')
        self.client = self.create_client(SetPwmDutyCycle, 'set_pwm_duty_cycle')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        self.pwm_duty_cycle_srv = self.client.service_proxy('set_pwm_duty_cycle')
        self.req = SetPwmDutyCycle.Request()

    def send_request(self, channel, duty_cycle):
        self.req.channel = channel
        self.req.duty_cycle = duty_cycle
        return self.client.call_async(self.req)

def main():
    rclpy.init()
    client = LwfPwmDriverClientAsync()
    future = client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    rclpy.spin_until_future_complete(client, future, timeout_sec=1.0)
    response = future.result()
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
