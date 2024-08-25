import cv2


class WetWorkService:

    def __init__(self, video_stream_url="http://192.168.5.243/cgi-bin/hi3510/snap.cgi?&-getstream&-chn=2"):
        self.running = True
        self.video_capture = cv2.VideoCapture(video_stream_url)

    def shutdown(self):
        print("Shutting down WetWorkService...")
        self.video_capture.release()
        cv2.destroyAllWindows()

    def run(self):
        while self.running:
            ret, frame = self.video_capture.read()
            cv2.imshow('Wet Work Service', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
