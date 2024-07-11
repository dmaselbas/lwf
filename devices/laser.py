from devices.pwm_controller import PWMController
import wiringpi

class LaserController:
    def __init__(self, controller:PWMController):
        self.controller = controller

    def on(self):
        self.controller.set_pwm(6, 4095, 0)  # Set channel 6 to 100% duty cycle

    def off(self):
        self.controller.set_pwm(6, 0, 0)  # Set channel 6 to 0% duty cycle

    def move_left(self):
        self.controller.set_pwm(5, 4095, 0)  # Set channel 5 to 100% duty cycle
        self.controller.set_pwm(4, 1023, 0)  # Set channel 4 to 25% duty cycle

    def move_right(self):
        self.controller.set_pwm(5, 0, 0)  # Set channel 5 to 0% duty cycle
        self.controller.set_pwm(4, 1023, 0)  # Set channel 4 to 25% duty cycle
