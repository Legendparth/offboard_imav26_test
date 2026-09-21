#!/usr/bin/env python3
"""
MLX90640 -> ROS 2, plus a browser view of what it is looking at.

Wraps homography_mlx_rgb/thermal_image.py's MLX90640Reader (retry/backoff on
I2C errors, background acquisition thread) and publishes every NEW frame:

    thermal/image     sensor_msgs/Image, 32FC1, 32 wide x 24 high, degrees C.
                      header.stamp is the wall-clock time the frame was read,
                      which is what thermal_drop uses to pick the attitude
                      the frame was taken at.
    thermal/hotspot   std_msgs/String  max_temp|row|col|ambient
    thermal/preview   sensor_msgs/Image, bgr8, the annotated view below.
                      Off by default; the MJPEG stream is the cheap way to
                      watch it.

THE VIDEO STREAM

    http://<jetson>:8082/         in any browser on the same network.

    NOT 8080 or 8081 -- window_detect owns 8080 and bar_detect owns 8081, and
    more than one of them may be up at once.

    What you see: the 32x24 frame upscaled with an inferno colormap, a THICK
    GREEN BOX round the hottest blob (the one the mission would fly to) with
    its temperature, a yellow cross on that blob's weighted centroid (the
    exact point the flight node aims at), thin grey boxes round every other
    warm blob, and a white crosshair at the centre of the frame -- which with
    the lens level is straight down, so the job of the flight is to bring the
    green box onto the crosshair.

    The boxes come from the SAME find_blobs() the flight node runs, so what
    you are watching is the detection itself and not a second opinion.

    stream_port:=0 turns it off.

Bench:
    ros2 run drone_testing thermal_sensor
    ros2 topic echo /thermal/hotspot
"""

import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from drone_testing.thermal_common import H, W, annotate, find_blobs


class _MjpegHandler(BaseHTTPRequestHandler):
    """Serves the newest annotated frame as multipart JPEG, for a browser."""

    node = None     # set by MjpegServer before the server starts

    def do_GET(self):
        if self.path in ('/', '/index.html'):
            self._send_page()
        elif self.path.startswith('/stream'):
            self._send_stream()
        elif self.path.startswith('/snapshot'):
            self._send_snapshot()
        else:
            self.send_error(404)

    def _send_page(self):
        body = (b"<html><head><title>thermal</title>"
                b"<style>body{background:#111;color:#eee;font-family:sans-serif;"
                b"margin:0;text-align:center}img{max-width:100%;"
                b"image-rendering:pixelated}</style></head>"
                b"<body><img src='/stream.mjpg'></body></html>")
        self.send_response(200)
        self.send_header('Content-Type', 'text/html')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_snapshot(self):
        frame = self.node.latest_jpeg()
        if frame is None:
            self.send_error(503, "no frame yet")
            return
        self.send_response(200)
        self.send_header('Content-Type', 'image/jpeg')
        self.send_header('Content-Length', str(len(frame)))
        self.end_headers()
        self.wfile.write(frame)

    def _send_stream(self):
        self.send_response(200)
        self.send_header('Cache-Control', 'no-cache, private')
        self.send_header('Content-Type',
                         'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        last = None
        try:
            while True:
                frame = self.node.latest_jpeg()
                if frame is None or frame is last:
                    time.sleep(0.02)
                    continue
                last = frame
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode())
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass    # the viewer closed the tab; not an error

    def log_message(self, *args):
        pass        # the default handler logs every frame to stderr


class MjpegServer:
    """Threaded HTTP server that never blocks the ROS callbacks."""

    def __init__(self, node, port):
        handler = type('_Handler', (_MjpegHandler,), {'node': node})
        self.server = ThreadingHTTPServer(('0.0.0.0', port), handler)
        self.server.daemon_threads = True
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def shutdown(self):
        # BaseException, not Exception. server.shutdown() blocks on the
        # serve_forever thread acknowledging it, and when this runs from a
        # Ctrl-C teardown the SIGINT lands INSIDE that wait and comes out as
        # KeyboardInterrupt -- which is a BaseException and sailed straight
        # through an `except Exception`, printing a socketserver traceback
        # over the node's closing summary. The thread is a daemon and dies
        # with the process regardless, so there is nothing to salvage here:
        # this is best-effort tidying and must never be the last thing the
        # operator sees.
        try:
            self.server.shutdown()
            self.server.server_close()
        except BaseException:
            pass


class ThermalSensor(Node):

    def __init__(self):
        super().__init__('thermal_sensor')

        self.refresh_hz = int(self.declare_parameter('refresh_hz', 8).value)
        self.publish_preview = bool(self.declare_parameter('publish_preview', False).value)
        self.preview_scale = int(self.declare_parameter('preview_scale', 20).value)
        self.stream_port = int(self.declare_parameter('stream_port', 8082).value)
        self.jpeg_quality = int(self.declare_parameter('jpeg_quality', 70).value)
        # Same defaults as the flight node's, so the boxes drawn here are the
        # boxes it would fly to. Change them together or the stream lies.
        self.min_contrast = float(self.declare_parameter('min_contrast', 3.0).value)
        self.min_blob_pixels = int(self.declare_parameter('min_blob_pixels', 2).value)

        # Imported here, not at module level, so a missing Blinka install fails
        # with a clear message instead of an import trace from setup's entry point.
        import adafruit_mlx90640
        from homography_mlx_rgb.thermal_image import MLX90640Reader

        rates = {
            1: adafruit_mlx90640.RefreshRate.REFRESH_1_HZ,
            2: adafruit_mlx90640.RefreshRate.REFRESH_2_HZ,
            4: adafruit_mlx90640.RefreshRate.REFRESH_4_HZ,
            8: adafruit_mlx90640.RefreshRate.REFRESH_8_HZ,
            16: adafruit_mlx90640.RefreshRate.REFRESH_16_HZ,
        }
        if self.refresh_hz not in rates:
            self.get_logger().warning(
                f"refresh_hz {self.refresh_hz} is not one of {sorted(rates)}; using 4.")
            self.refresh_hz = 4

        self.reader = MLX90640Reader(refresh_rate=rates[self.refresh_hz])
        self.reader.start()

        self.image_pub = self.create_publisher(Image, 'thermal/image', 10)
        self.hotspot_pub = self.create_publisher(String, 'thermal/hotspot', 10)
        self.preview_pub = (self.create_publisher(Image, 'thermal/preview', 10)
                            if self.publish_preview else None)

        self._jpeg = None
        self._jpeg_lock = threading.Lock()
        self.stream = MjpegServer(self, self.stream_port) if self.stream_port else None

        self.last_ts = None
        self.frames = 0
        self.started = time.monotonic()
        self.create_timer(0.02, self.poll)

        self.get_logger().warning(
            f"MLX90640 up at {self.refresh_hz} Hz, publishing thermal/image "
            "(32FC1, deg C)"
            + (f". WATCH IT AT http://<this-jetson>:{self.stream_port}/"
               if self.stream else ", no video stream (stream_port 0)."))

    # ------------------------------------------------------------ the frames

    def poll(self):
        frame = self.reader.get_latest_frame()
        if frame is None:
            if time.monotonic() - self.started > 5.0 and self.frames == 0:
                self.get_logger().warning(
                    "No MLX90640 frame yet -- check the I2C wiring.",
                    throttle_duration_sec=5.0)
            return
        if frame.timestamp == self.last_ts:
            return
        self.last_ts = frame.timestamp
        self.frames += 1

        grid = np.asarray(frame.grid, dtype=np.float32).reshape(H, W)
        stamp = Time()
        stamp.sec = int(frame.timestamp)
        stamp.nanosec = int((frame.timestamp - int(frame.timestamp)) * 1e9)

        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = 'thermal_camera'
        msg.height, msg.width = H, W
        msg.encoding = '32FC1'
        msg.is_bigendian = 0
        msg.step = W * 4
        msg.data = grid.tobytes()
        self.image_pub.publish(msg)

        blobs, ambient = find_blobs(grid.astype(float), self.min_contrast,
                                    min_pixels=self.min_blob_pixels)
        row, col = frame.max_pixel
        self.hotspot_pub.publish(String(
            data=f"{frame.max_temp:.2f}|{row}|{col}|{ambient:.2f}"))

        if self.preview_pub is not None or self.stream is not None:
            self._render(grid.astype(float), blobs, ambient, stamp)

    def _render(self, grid, blobs, ambient, stamp):
        import cv2
        img = annotate(grid, blobs, ambient, scale=self.preview_scale,
                       note=f"{self.refresh_hz}Hz #{self.frames}")

        if self.stream is not None:
            ok, buf = cv2.imencode('.jpg', img,
                                   [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            if ok:
                with self._jpeg_lock:
                    self._jpeg = buf.tobytes()

        if self.preview_pub is not None:
            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = 'thermal_camera'
            msg.height, msg.width = img.shape[0], img.shape[1]
            msg.encoding = 'bgr8'
            msg.step = img.shape[1] * 3
            msg.data = img.tobytes()
            self.preview_pub.publish(msg)

    def latest_jpeg(self):
        with self._jpeg_lock:
            return self._jpeg

    def destroy_node(self):
        if self.stream is not None:
            self.stream.shutdown()
        try:
            self.reader.close()
        except Exception as exc:
            self.get_logger().warning(f"Reader close failed: {exc}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ThermalSensor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
