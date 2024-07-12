import cv2
import gi

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib


class TargetingCameraStream:

    def __init__(self):
        Gst.init(None)
        self.pipeline = self.create_pipeline()

    def capture_video(self):
        cap = cv2.VideoCapture(0)  # Assuming the camera is at index 0
        if not cap.isOpened():
            print("Error: Could not open camera.")
            return None
        return cap

    def create_pipeline(self):
        pipeline_description = """
        v4l2src ! video/x-raw,format=YUY2,width=1920,height=1080,framerate=30/1 ! videoconvert ! \
        video/x-raw,format=I420 ! mpph264enc ! h264parse ! rtph264pay config-interval=1 pt=96 ! \
        udpsink host=0.0.0.0 port=5000
        """
        pipeline = Gst.parse_launch(pipeline_description)
        return pipeline

    def start_pipeline(self):
        self.pipeline.set_state(Gst.State.PLAYING)

    def stop_pipeline(self):
        self.pipeline.set_state(Gst.State.NULL)
