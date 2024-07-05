import asyncio
import json
#apt install libopus-dev libvpx-dev
import cv2
from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aiortc.contrib.media import MediaBlackhole
from aiortc.rtcrtpsender import RTCRtpSender
from av import VideoFrame
from picamera2 import Picamera2

# Initialize the camera
camera = Picamera2()
camera.configure(camera.create_preview_configuration())
camera.start()

async def index(request):
    content = open('index.html', 'r').read()
    return web.Response(content_type='text/html', text=content)

async def offer(request):
    params = await request.json()
    offer = RTCSessionDescription(sdp=params['sdp'], type=params['type'])
    pc = RTCPeerConnection()

    @pc.on('icecandidate')
    def on_icecandidate(event):
        if event.candidate:
            asyncio.ensure_future(
                signaling.send(json.dumps({'candidate': event.candidate}))
            )

    @pc.on('datachannel')
    def on_datachannel(channel):
        pass

    @pc.on('track')
    def on_track(track):
        print('Track received:', track.kind)

    pc.addTrack(VideoStreamTrack())

    await pc.setRemoteDescription(offer)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    return web.Response(
        content_type='application/json',
        text=json.dumps({
            'sdp': pc.localDescription.sdp,
            'type': pc.localDescription.type
        })
    )

async def start_server():
    app = web.Application()
    app.router.add_get('/', index)
    app.router.add_post('/offer', offer)
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, '0.0.0.0', 8080)
    await site.start()

class VideoStreamTrack(VideoStreamTrack):
    def __init__(self):
        super().__init__()
        self.frame = None

    async def recv(self):
        pts, time_base = await self.next_timestamp()
        frame = camera.capture_array()
        video_frame = VideoFrame.from_ndarray(frame, format='bgr24')
        video_frame.pts = pts
        video_frame.time_base = time_base
        return video_frame

if __name__ == '__main__':
    signaling = None
    loop = asyncio.get_event_loop()
    loop.run_until_complete(start_server())
    loop.run_forever()
