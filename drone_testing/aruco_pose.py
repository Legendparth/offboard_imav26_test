#!/usr/bin/env python3
"""
Pose of a downward-facing ArUco marker, as a ROS 2 node.

Same detection and the same solvePnP maths as the standalone
aruco_down_pose.py -- IPPE_SQUARE on the four corners of one known-size
marker -- wrapped in a node that publishes the result instead of printing
it, and with the camera read moved onto its own thread.

    /aruco/detected     std_msgs/Bool             debounced: true after
                                                  detect_frames consecutive
                                                  hits, false after
                                                  lost_frames misses
    /aruco/point        geometry_msgs/PointStamped marker centre in the
                                                  CAMERA BODY frame, metres,
                                                  published only on a frame
                                                  where the pose solved
    /aruco/info         std_msgs/String           one human-readable line

    image_topic:=<topic> takes frames from ROS instead of a camera device,
    which is how it runs in the simulator and off a bag.
    marker_ids:=2,3     the markers that COUNT, comma separated. A mission
                        that visits several pads visits several ids, and one
                        that arms ON a marked pad must be able to leave that
                        pad's id out. Defaults to marker_id alone. With more
                        than one in frame the LARGEST is taken, which is the
                        nearest. /aruco/point's frame_id carries the id of
                        the marker each fix is of ("camera_body/2").
    min_marker_distance_rate:=0.02
                        needed when the marker lies on a square PAD -- see
                        the comment by it, and expect zero detections on a
                        perfectly clear image without it.
    browser             http://<jetson-ip>:8080/  MJPEG of the annotated
                                                  frame. stream_port:=0 off.

WHY THIS NODE PUBLISHES CAMERA-FRAME NUMBERS AND NOTHING ELSE
-------------------------------------------------------------
It deliberately does not know about PX4, NED, or the vehicle's attitude.
Rotating the marker vector into NED needs the airframe's roll/pitch/yaw,
which lives in the flight node, and doing it there means this node can be
run and trusted on a bench with no flight controller attached at all.

The frame is the one aruco_down_pose.py defined:

    +x  RIGHT in the image
    +y  UP in the image (towards the top)
    +z  UP, i.e. opposite to where the camera looks

so a marker below the camera has NEGATIVE z, and height is -z.

WHAT THE FLIGHT NODE DOES WITH IT
    With the camera mounted image-up towards the nose and image-right to
    the vehicle's right, the mapping into body FRD is

        forward = y      right = x      down = -z

    See precision_land.py, which is the only consumer.

ON THE SCALE OF THESE NUMBERS
-----------------------------
fx is derived from hfov_deg, not from a calibration, so it carries whatever
error the quoted field of view has. That error does NOT affect x and y:

    apparent marker width in px   p = fx_true * S / Z_true      (measured)
    solver, using fx = k*fx_true  Z = fx*S/p        = k*Z_true
                                  X = u*Z/fx        = u*Z_true/fx_true = X_true

The inflated range and the deflated bearing cancel exactly, so the LATERAL
offsets are right even when the FOV is wrong. The HEIGHT is not -- it is
scaled by k -- which is why the flight node uses the lidar for height and
takes only x and y from here.

What does not cancel is lens distortion: distortion_coeffs is zero by
default, and a wide lens bends the corners worst at the edge of the frame,
which is where the marker sits when the vehicle is most off-centre. Running
cv2.calibrateCamera on a chessboard and passing the real fx/fy/cx/cy and
distortion removes it. Until then, treat the numbers as good near the
centre and slightly optimistic at the edge.

BENCH TEST -- do this before it ever flies

    ros2 run drone_testing aruco_pose
    ros2 topic echo /aruco/info

Put the marker on the floor, hold the airframe over it, and check that the
reported FORWARD / RIGHT words match where the marker actually is relative
to the nose. precision_land.py has a `bench` mode that prints the same
thing in vehicle terms and commands nothing.
"""

import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool, String
from std_msgs.msg import Header

from drone_testing.window_detect import array_to_imgmsg, imgmsg_to_bgr


# OpenCV's optical frame is (x right, y DOWN, z FORWARD along the view axis).
# The body frame used here is (x right, y UP, z UP), so the two differ by a
# 180 degree roll about x. Applying this to tvec puts the marker centre in the
# body frame; applying it to the marker rotation lets yaw be read about +z.
R_CF = np.array([[1.0,  0.0,  0.0],
                 [0.0, -1.0,  0.0],
                 [0.0,  0.0, -1.0]])


# ------------------------------------------------------------- mjpeg stream
#
# Same shape as the server in window_detect.py: it hands out JPEG at whatever
# rate the viewer can take and DROPS frames rather than queueing them, so a
# slow laptop on bad WiFi slows only itself and never the detection loop.

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
        body = (b"<html><head><title>aruco down</title>"
                b"<style>body{background:#111;color:#eee;font-family:sans-serif;"
                b"margin:0;text-align:center}img{max-width:100%}</style></head>"
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
        self.thread = threading.Thread(target=self.server.serve_forever,
                                       daemon=True)
        self.thread.start()

    def shutdown(self):
        # BaseException, not Exception: server.shutdown() blocks waiting for
        # the serve_forever thread, and on a Ctrl-C teardown the SIGINT lands
        # inside that wait as a KeyboardInterrupt, which is a BaseException
        # and went straight through `except Exception` as a socketserver
        # traceback. The thread is a daemon and dies with the process anyway.
        try:
            self.server.shutdown()
            self.server.server_close()
        except BaseException:
            pass


def body_words(forward, right):
    """'FORWARD 0.20 m, RIGHT 0.30 m' -- the sign check a human can read."""
    return (f"{'FORWARD' if forward >= 0 else 'BACK'} {abs(forward):.2f} m, "
            f"{'RIGHT' if right >= 0 else 'LEFT'} {abs(right):.2f} m")


class ArucoPose(Node):

    # ---- camera -----------------------------------------------------------
    CAMERA_INDEX = 0
    # 4:3 on purpose. The vertical field of view is what decides how low the
    # vehicle can go before the marker stops fitting in the frame, and 4:3 is
    # markedly taller than 16:9 on this sensor: with a 0.80 m marker and a
    # 78 deg horizontal FOV the marker fills the frame vertically at 0.66 m in
    # 4:3 but already at 0.88 m in 16:9. Lower is better -- it is 20 cm less
    # of blind descent. 800x600 also runs at 30 fps where 1280x960 drops to 15,
    # and for a control loop the frame rate is worth more than the pixels.
    WIDTH = 800
    HEIGHT = 600
    FOURCC = 'MJPG'
    CAMERA_FPS = 30.0

    # ---- the marker -------------------------------------------------------
    MARKER_ID = 0
    MARKER_SIZE = 0.80          # m, edge length
    ARUCO_DICT = 'DICT_5X5_50'
    HFOV_DEG = 78.0             # horizontal field of view. See the header:
                                # this scales the reported HEIGHT but cancels
                                # out of x and y.

    # ---- detection --------------------------------------------------------
    DETECT_RATE = 20.0          # Hz the newest frame is processed at
    DETECT_FRAMES = 3           # consecutive hits before /aruco/detected goes true
    LOST_FRAMES = 5             # consecutive misses before it goes false again

    # ---- output -----------------------------------------------------------
    STREAM_PORT = 8080          # 0 disables the browser stream
    STREAM_SCALE = 0.6
    JPEG_QUALITY = 70
    IMAGE_ROTATE = 0            # 0 | 90 | 180 | 270, applied BEFORE detection.
                                # Use this if the camera is bolted on rotated:
                                # the +x-right / +y-up frame follows the
                                # rotated image, so the mounting convention
                                # the flight node assumes stays true.

    def __init__(self):
        super().__init__('aruco_pose')

        self.camera_index = int(self.declare_parameter(
            'camera_index', self.CAMERA_INDEX).value)
        self.width = int(self.declare_parameter('width', self.WIDTH).value)
        self.height = int(self.declare_parameter('height', self.HEIGHT).value)
        self.fourcc = str(self.declare_parameter('fourcc', self.FOURCC).value)
        self.camera_fps = float(self.declare_parameter(
            'camera_fps', self.CAMERA_FPS).value)

        self.marker_id = int(self.declare_parameter(
            'marker_id', self.MARKER_ID).value)
        # MORE THAN ONE ACCEPTABLE MARKER.
        #
        #   A mission that visits several pads visits several IDs. The thermal
        #   drop flies over platform_1 (id 2) on its way out and lands on
        #   platform_2 (id 3), and it must ignore the takeoff pad (id 0) it is
        #   sitting on when it arms -- so "any marker" is wrong and one id is
        #   not enough. marker_ids is the set that counts; marker_id stays as
        #   the default and as the single-marker spelling, so nothing that
        #   already passes marker_id changes behaviour.
        #
        #   With several in frame the LARGEST is taken, which is the nearest
        #   one and therefore the one the aircraft is actually over.
        #   A COMMA-SEPARATED STRING and not an integer array, because a
        #   launch file can only hand a node a substitution, which is a
        #   string: an array parameter would have to be built with a
        #   ParameterValue wrapper at every call site, and getting that
        #   wrong yields "Type of parameter value is not supported" at
        #   startup rather than anything about markers.
        raw = str(self.declare_parameter('marker_ids', '').value)
        ids = []
        for part in raw.replace('[', ' ').replace(']', ' ').replace(',', ' ').split():
            try:
                ids.append(int(part))
            except ValueError:
                self.get_logger().error(
                    f"marker_ids: '{part}' is not a number; ignoring it.")
        self.marker_ids = ids if ids else [self.marker_id]
        # The id of the marker in the LAST solved frame, for the log line and
        # for /aruco/point's frame_id. A consumer that cares which pad it is
        # looking at reads it there; one that does not, ignores it.
        self.last_marker_id = self.marker_ids[0]
        self.marker_size = float(self.declare_parameter(
            'marker_size', self.MARKER_SIZE).value)
        dict_name = str(self.declare_parameter(
            'aruco_dict', self.ARUCO_DICT).value)
        self.hfov_deg = float(self.declare_parameter(
            'hfov_deg', self.HFOV_DEG).value)

        detect_rate = float(self.declare_parameter(
            'detect_rate', self.DETECT_RATE).value)
        self.detect_frames = int(self.declare_parameter(
            'detect_frames', self.DETECT_FRAMES).value)
        self.lost_frames = int(self.declare_parameter(
            'lost_frames', self.LOST_FRAMES).value)

        self.stream_port = int(self.declare_parameter(
            'stream_port', self.STREAM_PORT).value)
        self.stream_scale = float(self.declare_parameter(
            'stream_scale', self.STREAM_SCALE).value)
        self.jpeg_quality = int(self.declare_parameter(
            'jpeg_quality', self.JPEG_QUALITY).value)
        self.image_rotate = int(self.declare_parameter(
            'image_rotate', self.IMAGE_ROTATE).value)
        self.show_gui = bool(self.declare_parameter('show_gui', False).value)

        # Optional real calibration. Left empty by default, in which case fx
        # comes from hfov_deg and distortion is assumed zero -- see the header
        # for exactly what that costs you.
        fx = float(self.declare_parameter('fx', 0.0).value)
        fy = float(self.declare_parameter('fy', 0.0).value)
        cx = float(self.declare_parameter('cx', 0.0).value)
        cy = float(self.declare_parameter('cy', 0.0).value)
        dist = list(self.declare_parameter(
            'distortion_coeffs', [0.0, 0.0, 0.0, 0.0, 0.0]).value)
        self._cal = (fx, fy, cx, cy)
        self.D = np.array(dist, dtype=np.float64).reshape(-1, 1)

        if not hasattr(cv2.aruco, dict_name):
            raise SystemExit(f"Unknown aruco_dict '{dict_name}'.")
        dict_id = getattr(cv2.aruco, dict_name)

        # cv2.aruco was rewritten in OpenCV 4.7: getPredefinedDictionary and
        # an ArucoDetector object replaced Dictionary_get and a free
        # detectMarkers. ROS 2 Humble ships 4.5.4, which has only the old one.
        # Both are supported here, chosen at runtime, because the alternative
        # is a node that dies on import on half the machines it runs on --
        # which is exactly what it did.
        # MARKERS ON A SMALL, SQUARE PAD.
        #
        #   A marker lying on a square plate has a SECOND square contour
        #   around it -- the plate's own edge -- concentric with it and only
        #   slightly bigger. cv2.aruco treats two candidates whose corners
        #   are within minMarkerDistanceRate of each other (5% of the image
        #   by default) as the same thing and keeps the LARGER, so the
        #   plate's outline swallows the marker and nothing is ever
        #   detected: the quad is found, the bits are sampled a cell off,
        #   and it is rejected. In the simulated arena the pad is 1.10 m and
        #   the marker 0.88 m, about 2% of the frame apart, and the default
        #   loses every single frame while the stream looks perfect.
        #
        #   0.05 is OpenCV's default and is left alone, because on the real
        #   course the marker is not ringed by anything. The simulator
        #   passes 0.02. A value of 0 disables the merge entirely and,
        #   confusingly, also detects nothing -- do not use it as "off".
        self.min_marker_distance_rate = float(self.declare_parameter(
            'min_marker_distance_rate', 0.05).value)

        if hasattr(cv2.aruco, 'ArucoDetector'):
            dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
            params = cv2.aruco.DetectorParameters()
            # Sub-pixel corner refinement. This is the difference between a
            # corner good to a pixel and one good to a tenth, and every
            # centimetre of lateral accuracy comes through those four corners.
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            params.minMarkerDistanceRate = self.min_marker_distance_rate
            detector = cv2.aruco.ArucoDetector(dictionary, params)
            self._detect = detector.detectMarkers
        else:
            dictionary = cv2.aruco.Dictionary_get(dict_id)
            params = cv2.aruco.DetectorParameters_create()
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            params.minMarkerDistanceRate = self.min_marker_distance_rate
            self._detect = lambda gray: cv2.aruco.detectMarkers(
                gray, dictionary, parameters=params)
        self.get_logger().info(
            f"OpenCV {cv2.__version__}, "
            f"{'ArucoDetector' if hasattr(cv2.aruco, 'ArucoDetector') else 'legacy aruco'} API.")
        if fx <= 0.0:
            self.get_logger().warning(
                f"No calibration given: fx derived from hfov_deg={self.hfov_deg:.1f}. "
                "x/y are unaffected by a wrong FOV, HEIGHT is scaled by it, and "
                "lens distortion is NOT corrected. Do not use the height.")

        s = self.marker_size / 2.0
        # TL, TR, BR, BL -- cv2.aruco's corner order, and the order
        # SOLVEPNP_IPPE_SQUARE requires. Do not reorder these.
        self.objp = np.array([[-s,  s, 0],
                              [ s,  s, 0],
                              [ s, -s, 0],
                              [-s, -s, 0]], dtype=np.float64)

        self.K = None
        self.detected = False
        self._hits = 0
        self._misses = 0

        self._frame = None
        self._frame_seq = 0
        self._processed_seq = -1
        self._frame_lock = threading.Lock()
        self._jpeg = None
        self._stop = threading.Event()

        self.detected_pub = self.create_publisher(Bool, '/aruco/detected', 10)
        self.point_pub = self.create_publisher(PointStamped, '/aruco/point', 10)
        self.info_pub = self.create_publisher(String, '/aruco/info', 10)

        # A ROS image topic instead of a camera device. The simulator has no
        # /dev/video, and on the bench it is sometimes easier to replay a bag
        # than to point a camera at the floor. Everything downstream -- the
        # detection, the solve, the frames, the stream -- is the same either
        # way; only where the pixels come from changes.
        # Re-publish the frames this node captures, so a SECOND detector can
        # see the same pixels. One v4l2 device cannot be opened twice, and
        # line_detect needs the same downward camera this node owns -- so the
        # owner shares rather than the other one prising the device open.
        # Empty (the default) publishes nothing and costs nothing.
        self.publish_frames = str(
            self.declare_parameter('publish_frames', '').value).strip()
        self.frame_pub = None
        if self.publish_frames:
            from sensor_msgs.msg import Image as _Image
            # Depth 1 and best-effort: a late frame is not worth having, and a
            # slow subscriber must never back-pressure the detection loop.
            from rclpy.qos import qos_profile_sensor_data as _sensor_qos
            self.frame_pub = self.create_publisher(_Image, self.publish_frames,
                                                   _sensor_qos)
            self.get_logger().warning(
                f"Re-publishing frames on {self.publish_frames} for a second "
                "detector (line_detect). This is the SAME image this node "
                "detects markers in, rotation included.")

        self.image_topic = str(self.declare_parameter('image_topic', '').value).strip()
        self.cap = None
        if self.image_topic:
            from sensor_msgs.msg import Image
            from rclpy.qos import qos_profile_sensor_data
            self.create_subscription(Image, self.image_topic,
                                     self._on_image, qos_profile_sensor_data)
            self.get_logger().warning(
                f"Frames from {self.image_topic}, not from a camera device.")
        else:
            self._open_camera()

        if self.cap is not None:
            # The grab runs on its own thread and always keeps only the latest
            # frame. cap.read() blocks for a frame interval, and doing that
            # inside a ROS timer would stall this node's executor for 33 ms at
            # a time.
            self._grab_thread = threading.Thread(target=self._grab_loop, daemon=True)
            self._grab_thread.start()

        self.stream = None
        if self.stream_port:
            try:
                self.stream = MjpegServer(self, self.stream_port)
                self.get_logger().info(
                    f"Browser stream on http://<jetson-ip>:{self.stream_port}/")
            except Exception as exc:
                self.get_logger().error(
                    f"Could not start the stream on port {self.stream_port}: {exc}")

        self.timer = self.create_timer(1.0 / max(detect_rate, 1.0), self.detect_once)
        self.get_logger().warning(
            "ArUco down-camera pose: id "
            f"{'/'.join(str(i) for i in self.marker_ids)}, "
            f"{self.marker_size * 100:.0f} cm marker, {dict_name}, "
            f"processing at {detect_rate:.0f} Hz.")

    def _on_image(self, msg):
        """One frame off a ROS topic, into the same slot the grabber fills."""
        try:
            frame = imgmsg_to_bgr(msg)
        except Exception as exc:
            self.get_logger().error(f"Cannot convert image frame: {exc}",
                                    throttle_duration_sec=5.0)
            return
        with self._frame_lock:
            self._frame = frame
            self._frame_seq += 1

    def _open_camera(self):
        """Open the capture device, or say why not and carry on without one.

        Not opening is not fatal. This node is one detector among several in a
        launch file, and exiting here used to take it out of a flight that had
        no use for it yet -- in the simulator there is no /dev/video at all.
        With no camera it simply never publishes, and the flight node already
        treats "no marker" as a thing that happens.
        """
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            self.get_logger().error(
                f"Could not open camera {self.camera_index}. This node will "
                "publish nothing. Pass image_topic:=<topic> to take frames "
                "from ROS instead of a device.")
            self.cap = None
            return
        if len(self.fourcc) == 4:  # device path only, from _open_camera
            self.cap.set(cv2.CAP_PROP_FOURCC,
                         cv2.VideoWriter_fourcc(*self.fourcc))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        if self.camera_fps > 0.0:
            self.cap.set(cv2.CAP_PROP_FPS, self.camera_fps)
        # One-deep buffer: we always want the NEWEST frame. A queued frame is
        # latency, and latency in a landing loop is phase lag.
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(
            f"Camera {self.camera_index} open at {actual_w}x{actual_h} "
            f"@ {self.camera_fps:.0f} fps.")

    # ------------------------------------------------------------- capture

    def _grab_loop(self):
        while not self._stop.is_set():
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.02)
                continue
            with self._frame_lock:
                self._frame = frame
                self._frame_seq += 1

    def latest_jpeg(self):
        return self._jpeg

    # ------------------------------------------------------------ intrinsics

    def _intrinsics(self, w, h):
        fx, fy, cx, cy = self._cal
        if fx > 0.0:
            return np.array([[fx, 0, cx if cx > 0 else w / 2.0],
                             [0, fy if fy > 0 else fx, cy if cy > 0 else h / 2.0],
                             [0, 0, 1.0]], dtype=np.float64)
        f = (w / 2.0) / math.tan(math.radians(self.hfov_deg) / 2.0)
        return np.array([[f, 0, w / 2.0],
                         [0, f, h / 2.0],
                         [0, 0, 1.0]], dtype=np.float64)

    # -------------------------------------------------------------- detect

    def detect_once(self):
        with self._frame_lock:
            if self._frame is None or self._frame_seq == self._processed_seq:
                return      # nothing new since last time
            frame = self._frame
            self._processed_seq = self._frame_seq

        if self.image_rotate:
            rot = {90: cv2.ROTATE_90_CLOCKWISE,
                   180: cv2.ROTATE_180,
                   270: cv2.ROTATE_90_COUNTERCLOCKWISE}.get(self.image_rotate)
            if rot is not None:
                frame = cv2.rotate(frame, rot)

        h, w = frame.shape[:2]
        if self.K is None:
            self.K = self._intrinsics(w, h)

        # After the rotation, so the second detector's flip_lr / cam_yaw dials
        # mean the same thing as this node's do.
        if self.frame_pub is not None:
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = 'down_camera'
            self.frame_pub.publish(array_to_imgmsg(frame, 'bgr8', header))

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detect(gray)
        seen = ids.flatten().tolist() if ids is not None else []

        pose = None
        quad = None
        # Every wanted marker in the frame, biggest first: the biggest is the
        # nearest, and the nearest is the one the aircraft is over.
        wanted = [(cv2.contourArea(corners[i].reshape(4, 2).astype(np.float32)), i)
                  for i, mid in enumerate(seen) if mid in self.marker_ids]
        if wanted:
            _, i = max(wanted)
            c = corners[i].reshape(4, 2).astype(np.float64)
            found, rvec, tvec = cv2.solvePnP(self.objp, c, self.K, self.D,
                                             flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if found:
                pose = tuple(float(v) for v in R_CF @ tvec.reshape(3))
                quad = c
                self.last_marker_id = int(seen[i])

        self._update_debounce(pose is not None)

        if pose is not None:
            x, y, z = pose
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            # Which marker this fix is of, for a consumer that cares. The
            # frame is still the camera body frame; the suffix names the pad.
            msg.header.frame_id = f'camera_body/{self.last_marker_id}'
            msg.point.x, msg.point.y, msg.point.z = x, y, z
            self.point_pub.publish(msg)
            # forward = y, right = x, for the mounting this package assumes.
            line = (f"id {self.last_marker_id} SEEN  cam x={x:+.3f} y={y:+.3f} "
                    f"z={z:+.3f} m  height={-z:.3f} m  |  marker is "
                    f"{body_words(y, x)} of the camera")
        else:
            line = (f"id {'/'.join(str(i) for i in self.marker_ids)} not "
                    f"visible (seen: {seen or '-'})")

        self.detected_pub.publish(Bool(data=self.detected))
        self.info_pub.publish(String(data=line))
        self.get_logger().info(line, throttle_duration_sec=1.0)

        self._render(frame, quad, line)

    def _update_debounce(self, hit):
        """Same debounce shape as window_detect: N hits on, M misses off.

        A single frame either way is noise -- a glint, a motion-blurred
        corner -- and neither a lock nor a dropout should turn on one.
        """
        if hit:
            self._misses = 0
            self._hits += 1
            if not self.detected and self._hits >= self.detect_frames:
                self.detected = True
                self.get_logger().warning("Marker ACQUIRED.")
        else:
            self._hits = 0
            self._misses += 1
            if self.detected and self._misses >= self.lost_frames:
                self.detected = False
                self.get_logger().warning("Marker LOST.")

    # -------------------------------------------------------------- render

    def _render(self, frame, quad, line):
        if self.stream is None and not self.show_gui:
            return

        img = frame.copy()
        h, w = img.shape[:2]
        if quad is not None:
            cv2.polylines(img, [quad.reshape(-1, 1, 2).astype(int)],
                          True, (0, 255, 0), 3)
        cv2.drawMarker(img, (w // 2, h // 2), (90, 90, 100),
                       cv2.MARKER_CROSS, 28, 1)
        cv2.putText(img, line[:78], (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 255) if quad is not None else (120, 120, 255),
                    1, cv2.LINE_AA)

        if self.show_gui:
            cv2.imshow('aruco down', img)
            cv2.waitKey(1)

        if self.stream is not None:
            out = img
            if 0.0 < self.stream_scale < 1.0:
                out = cv2.resize(img, None, fx=self.stream_scale,
                                 fy=self.stream_scale,
                                 interpolation=cv2.INTER_AREA)
            ok, buf = cv2.imencode(
                '.jpg', out, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            if ok:
                self._jpeg = buf.tobytes()

    # ------------------------------------------------------------- shutdown

    def destroy_node(self):
        self._stop.set()
        if self.stream is not None:
            self.stream.shutdown()
        try:
            self.cap.release()
        except Exception:
            pass
        if self.show_gui:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ArucoPose()
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
