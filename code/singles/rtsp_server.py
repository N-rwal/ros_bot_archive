# save as rtsp_server.py
import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer, GObject

Gst.init(None)

class RTSPMediaFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self):
        super(RTSPMediaFactory, self).__init__()

    def do_create_element(self, url):
        pipeline = (
            "v4l2src device=/dev/video0 ! videoconvert ! "
            "x264enc tune=zerolatency bitrate=500 speed-preset=ultrafast ! rtph264pay name=pay0 pt=96"
        )
        return Gst.parse_launch(pipeline)

server = GstRtspServer.RTSPServer()
factory = RTSPMediaFactory()
factory.set_shared(True)
mount_points = server.get_mount_points()
mount_points.add_factory("/live", factory)
server.attach(None)

print("Stream ready at rtsp://127.0.0.1:8554/live")
GObject.MainLoop().run()
