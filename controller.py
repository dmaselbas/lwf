import pyglet
from pyglet.window import key
from servicess.navigation_service import NavigationService
from devices.lidar import LidarController
from devices.laser import LaserClient

class GamepadController:
    def __init__(self):
        self.navigation_service = NavigationService(LidarController())
        self.laser_client = LaserClient()
        self.speed = 50  # Initial speed
        self.window = pyglet.window.Window(visible=False)  # Create an invisible window to capture events
        self.joystick = None

        # Get the first available joystick
        joysticks = pyglet.input.get_joysticks()
        if joysticks:
            self.joystick = joysticks[0]
            self.joystick.open()
            self.joystick.push_handlers(self)

        pyglet.clock.schedule_interval(self.update, 1/60.0)  # Update at 60Hz

    def on_joybutton_press(self, joystick, button):
        print(f'Button {button} pressed on {joystick}')  # Debug print for button press
        if button == 4:  # Left bumper
            self.laser_client.turn_on()
        elif button == 5:  # Right bumper
            self.laser_client.turn_off()
        elif button == 6:  # Back button
            self.laser_client.pulse_laser()
        elif button == 7:  # Start button
            self.laser_client.enable_cycle_motion()
        elif button == 8:  # Logitech button
            self.laser_client.disable_cycle_motion()
        elif button == 11:  # Up button on D-pad
            self.speed = min(self.speed + 5, 100)  # Increase speed, max 100
            self.navigation_service.set_drive_speed(self.speed)
        elif button == 12:  # Down button on D-pad
            self.speed = max(self.speed - 5, 0)  # Decrease speed, min 0
            self.navigation_service.set_drive_speed(self.speed)

    def on_joyaxis_motion(self, joystick, axis, value):
        print(f'Axis {axis} moved to {value} on {joystick}')  # Debug print for axis motion
        if axis == 'x':  # Left stick horizontal
            if value < -0.5:
                self.navigation_service.drive_left()
            elif value > 0.5:
                self.navigation_service.drive_right()
            else:
                self.navigation_service.stop_driving()
        elif axis == 'y':  # Left stick vertical
            if value < -0.5:
                self.navigation_service.drive_forward()
            elif value > 0.5:
                self.navigation_service.drive_backward()
            else:
                self.navigation_service.stop_driving()

    def update(self, dt):
        pass  # Update logic if needed

    def stop(self):
        self.navigation_service.stop_driving()
        self.laser_client.turn_off()
        self.laser_client.disable_cycle_motion()
        if self.joystick:
            self.joystick.close()

if __name__ == "__main__":
    try:
        gamepad_controller = GamepadController()
        pyglet.app.run()
    except ValueError as e:
        print(e)
    except KeyboardInterrupt:
        gamepad_controller.stop()
