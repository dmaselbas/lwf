import libcamera
import numpy as np

class Camera:
    def __init__(self, device_path='/dev/video0'):
        self.camera = libcamera.Camera(device_path)
        self.camera.capture_mode = 'jpeg'
        self.camera.resolution = (640, 480)
        self.camera.open()

    def capture_picture(self):
        capture = self.camera.create_capture()
        capture.capture()
        capture.wait()

        # Get the captured image data
        image_data = capture.get_buffer()

        # Convert the image data to a NumPy array
        image_array = np.frombuffer(image_data, dtype=np.uint8)

        # Reshape the array to match the image dimensions
        image_array = image_array.reshape(self.camera.resolution[1], self.camera.resolution[0], 3)

        return image_array

    def close(self):
        self.camera.close()

# Example usage
camera = Camera()
image = camera.capture_picture()
camera.close()

# Save the captured image as a JPEG file
import cv2
cv2.imwrite('captured_image.jpg', image)
