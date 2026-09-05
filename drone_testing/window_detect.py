"""
Window detection from the ZED, as a ROS 2 node.

Same detection logic as the original standalone script -- HSV threshold on
green, largest contour, convex hull approximated to a quadrilateral, corner
depths sampled just inside each corner -- but the frames now come from the
zed_wrapper node over ROS topics instead of from a pyzed stream.

    rgb    /zed/zed_node/rgb/image_rect_color        (rectified LEFT camera)
    depth  /zed/zed_node/depth/depth_registered      (32FC1, metres, aligned
                                                      to the same left frame)

Both topic names are parameters, so if your zed_wrapper build names them
differently you do not have to touch the code:

    ros2 run drone_testing window_detect --ros-args \
        -p image_topic:=/zed/zed_node/rgb/image_rect_color \
        -p depth_topic:=/zed/zed_node/depth/depth_registered

Check what your wrapper actually publishes with:

    ros2 topic list | grep zed

Depth is NOT synchronised with the image through a message filter: the most
recent depth frame is kept and used if it is younger than depth_max_age.
The two come out of the same SDK grab at the same rate, so approximate
pairing is what a synchroniser would give anyway, and a missing or stale
depth frame degrades to "detect the window, report no distances" instead of
dropping the detection entirely.

WHAT IT PUBLISHES

    /window_detected        std_msgs/Bool     debounced: true only after
                                              detect_frames consecutive hits,
                                              false after lost_frames misses
    /window_info            std_msgs/String   pipe-separated detail line,
                                              see publish_info()
    /window_detection/image sensor_msgs/Image the annotated frame

HOW TO SEE IT

    terminal   this node logs one line a second either way, and one WARN the
               moment the detection latches or is lost
               ros2 topic echo /window_detected
               ros2 topic echo /window_info
    picture    ros2 run rqt_image_view rqt_image_view /window_detection/image
    on the LCD lcd_status shows "win YES/no" on row 4 (it subscribes to
               /window_detected itself)

`-p show_windows:=true` brings back the two cv2.imshow windows from the
original script. That needs a display, so leave it false on the Jetson
unless you are sitting in front of it with a monitor plugged in.
"""

import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from cv_bridge import CvBridge


HSV_RANGES = {
    "blue": [(np.array([95, 80, 40]), np.array([130, 255, 255]))],
    "red": [
        (np.array([0, 100, 60]), np.array([10, 255, 255])),
        (np.array([170, 100, 60]), np.array([180, 255, 255])),
    ],
    "green": [(np.array([35, 40, 30]), np.array([90, 255, 255]))],
}


def get_median_depth(depth_img, u, v, box=3):
    h, w = depth_img.shape[:2]
    u = min(max(u, box), w - 1 - box)
    v = min(max(v, box), h - 1 - box)
    depth_values = depth_img[v - box:v + box + 1, u - box:u + box + 1].flatten()
    valid_depths = depth_values[(depth_values > 0) & (~np.isnan(depth_values)) & (~np.isinf(depth_values))]
    if len(valid_depths) > 0:
        return np.median(valid_depths)
    return 0.0


def sample_corner_depth(depth_img, u, v, center, inset=10, box=4):
    cu, cv = center
    du, dv = cu - u, cv - v
    norm = np.hypot(du, dv) + 1e-6
    su = int(round(u + inset * du / norm))
    sv = int(round(v + inset * dv / norm))
    su = min(max(su, 0), depth_img.shape[1] - 1)
    sv = min(max(sv, 0), depth_img.shape[0] - 1)
    return get_median_depth(depth_img, su, sv, box), (su, sv)


def hsv_mask(hsv_image, color):
    mask = None
    for lo, hi in HSV_RANGES[color]:
        m = cv2.inRange(hsv_image, lo, hi)
        mask = m if mask is None else cv2.bitwise_or(mask, m)

    # close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
    # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel, iterations=2)

    open_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (4, 4))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel, iterations=2)

    # eroding_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (6, 6))
    # mask = cv2.erode(mask,eroding_kernel, iterations=1)

    dialate_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (4, 4))
    mask = cv2.dilate(mask,dialate_kernel, iterations=2)

    return mask


def approx_quad(contour):
    peri = cv2.arcLength(contour, True)
    for eps in np.linspace(0.01, 0.06, 6):
        approx = cv2.approxPolyDP(contour, eps * peri, True)
        if len(approx) == 4:
            return approx
    return None


def window_detection(mask, min_area=1500):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < min_area:
        return None
    hull = cv2.convexHull(largest)
    approx = approx_quad(hull)
    if approx is None or not cv2.isContourConvex(approx):
        return None
    return approx


def filter_depth(new_d, prev_d, alpha=0.3):
    if new_d is not None and new_d > 0:
        if prev_d is None or prev_d == 0:
            return new_d, new_d
        filtered = alpha * new_d + (1 - alpha) * prev_d
        return filtered, filtered
    return prev_d, prev_d


class WindowDetect(Node):

    # Topic defaults. These are the standard zed_wrapper (ROS 2) names for the
    # rectified left colour image and the depth map registered to it.
    IMAGE_TOPIC = '/zed/zed_node/rgb/image_rect_color'
    DEPTH_TOPIC = '/zed/zed_node/depth/depth_registered'

    # Debounce. A single frame's worth of green is not a window: one flash of
    # colour must not be able to stop a yaw sweep, and one dropped frame must
    # not unstick a vehicle that has already locked onto the real thing.
    DETECT_FRAMES = 3
    LOST_FRAMES = 5

    MIN_AREA = 1500             # px^2, smallest contour taken seriously
    PADDING = 5                 # px each corner is pulled inwards by
    CORNER_ALPHA = 0.4          # corner position smoothing
    DEPTH_ALPHA = 0.3           # corner depth smoothing
    DEPTH_MAX_AGE = 0.5         # s a depth frame stays usable for
    DEPTH_SCALE = 1.0           # multiplier on the raw depth. ROS depth is in
                                # metres; set 100.0 if you want the centimetres
                                # the standalone script printed.
    LOG_PERIOD = 1.0            # s between the routine status lines

    def __init__(self):
        super().__init__('window_detect')

        self.image_topic = str(self.declare_parameter('image_topic', self.IMAGE_TOPIC).value)
        self.depth_topic = str(self.declare_parameter('depth_topic', self.DEPTH_TOPIC).value)
        self.use_depth = bool(self.declare_parameter('use_depth', True).value)
        self.show_windows = bool(self.declare_parameter('show_windows', False).value)
        self.publish_image = bool(self.declare_parameter('publish_image', True).value)
        self.publish_mask = bool(self.declare_parameter('publish_mask', False).value)
        self.color = str(self.declare_parameter('color', 'green').value).strip().lower()
        self.min_area = float(self.declare_parameter('min_area', float(self.MIN_AREA)).value)
        self.detect_frames = int(self.declare_parameter('detect_frames', self.DETECT_FRAMES).value)
        self.lost_frames = int(self.declare_parameter('lost_frames', self.LOST_FRAMES).value)
        self.depth_scale = float(self.declare_parameter('depth_scale', self.DEPTH_SCALE).value)
        self.depth_units = 'cm' if abs(self.depth_scale - 100.0) < 1e-6 else 'm'

        if self.color not in HSV_RANGES:
            raise SystemExit(
                f"Unknown color '{self.color}'; expected one of {sorted(HSV_RANGES)}")

        self.bridge = CvBridge()

        # Sensor QoS (best effort, depth 1). A best-effort subscription is
        # compatible with a reliable publisher as well, so this works whichever
        # way zed_wrapper's qos_reliability is configured -- and on a frame we
        # are processing at camera rate, the newest one is the only one worth
        # having anyway.
        self.create_subscription(Image, self.image_topic,
                                 self.image_callback, qos_profile_sensor_data)
        if self.use_depth:
            self.create_subscription(Image, self.depth_topic,
                                     self.depth_callback, qos_profile_sensor_data)

        self.detected_pub = self.create_publisher(Bool, 'window_detected', 10)
        self.info_pub = self.create_publisher(String, 'window_info', 10)
        self.image_pub = (self.create_publisher(Image, 'window_detection/image', 1)
                          if self.publish_image else None)
        self.mask_pub = (self.create_publisher(Image, 'window_detection/mask', 1)
                         if self.publish_mask else None)

        # Detection state, carried between frames exactly as the loop in the
        # standalone script carried it between iterations.
        self.smoothed_corners = None
        self.prev_e1 = self.prev_e2 = self.prev_e3 = self.prev_e4 = 0

        self.depth_image = None
        self.depth_time = 0.0

        self.hit_streak = 0
        self.miss_streak = 0
        self.detected = False       # the debounced answer
        self.last_info = ''
        self.frames = 0
        self.first_frame_logged = False
        self._last_log = 0.0

        # Heartbeat, so a dead camera is loud rather than silent. The detection
        # itself is published from the image callback, at camera rate.
        self.create_timer(1.0, self.watchdog)
        self.last_image_time = None

        self.get_logger().info(
            f"Window detection up. image={self.image_topic} "
            f"depth={self.depth_topic if self.use_depth else 'disabled'} "
            f"color={self.color}. Publishing /window_detected, /window_info"
            + (", /window_detection/image" if self.publish_image else "") + ".")

    # ------------------------------------------------------------------ subs

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth_time = time.monotonic()
        except Exception as exc:
            self.get_logger().warning(f"Cannot convert depth frame: {exc}",
                                      throttle_duration_sec=5.0)

    def current_depth(self):
        """The newest depth frame, or None if it is missing or stale."""
        if self.depth_image is None:
            return None
        if time.monotonic() - self.depth_time > self.DEPTH_MAX_AGE:
            return None
        return self.depth_image

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f"Cannot convert image frame: {exc}",
                                    throttle_duration_sec=5.0)
            return

        self.last_image_time = time.monotonic()
        self.frames += 1
        if not self.first_frame_logged:
            self.first_frame_logged = True
            h, w = cv_image.shape[:2]
            self.get_logger().info(f"First frame from {self.image_topic}: {w}x{h}.")

        self.process(cv_image, msg.header)

    # ------------------------------------------------------------ detection

    def process(self, cv_image, header):
        """One frame. Identical pipeline to the standalone script."""
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        green_mask = hsv_mask(hsv_img, self.color)
        window_contour = window_detection(green_mask, self.min_area)

        depth_image = self.current_depth() if self.use_depth else None
        info = ''

        if window_contour is not None:
            window_contour = window_contour.reshape(-1, 2)
            s = np.sum(window_contour, axis=1)
            u1, v1 = window_contour[np.argmin(s)]
            u3, v3 = window_contour[np.argmax(s)]

            u1, v1 = max(u1 + self.PADDING, 0), max(v1 + self.PADDING, 0)
            u3, v3 = (min(u3 - self.PADDING, cv_image.shape[1] - 1),
                      min(v3 - self.PADDING, cv_image.shape[0] - 1))

            diff = np.diff(window_contour, axis=1)
            u2, v2 = window_contour[np.argmin(diff)]
            u4, v4 = window_contour[np.argmax(diff)]

            u2, v2 = max(u2 - self.PADDING, 0), max(v2 + self.PADDING, 0)
            u4, v4 = (min(u4 + self.PADDING, cv_image.shape[1] - 1),
                      min(v4 - self.PADDING, cv_image.shape[0] - 1))

            corners = np.array([[u1, v1], [u2, v2], [u3, v3], [u4, v4]], dtype=np.float32)

            if self.smoothed_corners is None:
                self.smoothed_corners = corners
            else:
                self.smoothed_corners = (self.CORNER_ALPHA * corners
                                         + (1 - self.CORNER_ALPHA) * self.smoothed_corners)

            u1, v1 = self.smoothed_corners[0].astype(int)
            u2, v2 = self.smoothed_corners[1].astype(int)
            u3, v3 = self.smoothed_corners[2].astype(int)
            u4, v4 = self.smoothed_corners[3].astype(int)

            center = self.smoothed_corners.mean(axis=0)

            d1 = d2 = d3 = d4 = 0.0
            if depth_image is not None:
                d1, s1 = sample_corner_depth(depth_image, u1, v1, center)
                d2, s2 = sample_corner_depth(depth_image, u2, v2, center)
                d3, s3 = sample_corner_depth(depth_image, u3, v3, center)
                d4, s4 = sample_corner_depth(depth_image, u4, v4, center)

                for sp in (s1, s2, s3, s4):
                    cv2.circle(cv_image, sp, 4, (0, 0, 255), -1)

                d1, self.prev_e1 = filter_depth(d1, self.prev_e1, self.DEPTH_ALPHA)
                d2, self.prev_e2 = filter_depth(d2, self.prev_e2, self.DEPTH_ALPHA)
                d3, self.prev_e3 = filter_depth(d3, self.prev_e3, self.DEPTH_ALPHA)
                d4, self.prev_e4 = filter_depth(d4, self.prev_e4, self.DEPTH_ALPHA)

                d1, d2, d3, d4 = (d * self.depth_scale for d in (d1, d2, d3, d4))

            cv2.drawContours(cv_image, [window_contour], -1, (255, 0, 255), 3)
            cv2.putText(cv_image, "Window Detected", (u1 - 10, v1 - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

            for pt, label, d in zip([(u1, v1), (u2, v2), (u3, v3), (u4, v4)],
                                    ['d1', 'd2', 'd3', 'd4'], [d1, d2, d3, d4]):
                cv2.circle(cv_image, pt, 6, (255, 0, 0), -1)
                cv2.putText(cv_image, f"{label}={d:.2f}", (pt[0] + 10, pt[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            info = self.frame_info(cv_image, center, [d1, d2, d3, d4], depth_image)
        else:
            # Nothing this frame: drop the smoothing state so a later detection
            # starts from its own corners instead of blending into wherever the
            # last one happened to be.
            self.smoothed_corners = None

        self.update_detection(window_contour is not None, info)
        self.annotate_banner(cv_image)
        self.publish_frames(cv_image, green_mask, header)

        if self.show_windows:
            cv2.imshow("ZED Image Processing", cv_image)
            cv2.imshow("Green Mask", green_mask)
            cv2.waitKey(1)

    def frame_info(self, cv_image, center, depths, depth_image):
        """Pipe-separated detail for /window_info.

        u|v|offset|area|d1|d2|d3|d4|dc

        offset is the horizontal position of the window centre as a fraction
        of half the image width: -1 hard left, 0 dead centre, +1 hard right.
        That is the number a centring controller wants, and it is independent
        of the resolution the wrapper happens to be publishing at.
        """
        h, w = cv_image.shape[:2]
        cu, cv_ = float(center[0]), float(center[1])
        offset = (cu - w / 2.0) / (w / 2.0)
        area = float(cv2.contourArea(self.smoothed_corners.astype(np.int32)))

        dc = 0.0
        if depth_image is not None:
            dc = float(get_median_depth(depth_image, int(round(cu)), int(round(cv_)), box=5))
            dc *= self.depth_scale

        return "|".join([
            f"{cu:.1f}", f"{cv_:.1f}", f"{offset:+.3f}", f"{area:.0f}",
            f"{depths[0]:.2f}", f"{depths[1]:.2f}",
            f"{depths[2]:.2f}", f"{depths[3]:.2f}", f"{dc:.2f}",
        ])

    def update_detection(self, hit, info):
        """Debounce, publish, and say out loud when the answer changes."""
        if hit:
            self.hit_streak += 1
            self.miss_streak = 0
            self.last_info = info
        else:
            self.miss_streak += 1
            self.hit_streak = 0

        if not self.detected and self.hit_streak >= self.detect_frames:
            self.detected = True
            self.get_logger().warning(
                f"WINDOW DETECTED  ({self.describe()})")
        elif self.detected and self.miss_streak >= self.lost_frames:
            self.detected = False
            self.get_logger().warning("Window LOST.")

        msg = Bool()
        msg.data = self.detected
        self.detected_pub.publish(msg)

        if hit:
            info_msg = String()
            info_msg.data = info
            self.info_pub.publish(info_msg)

        now = time.monotonic()
        if now - self._last_log >= self.LOG_PERIOD:
            self._last_log = now
            if self.detected:
                self.get_logger().info(f"window: YES  {self.describe()}")
            else:
                self.get_logger().info(
                    f"window: no   (streak {self.miss_streak} misses, "
                    f"{self.frames} frames seen)")

    def describe(self):
        """Human-readable version of the last good detection."""
        if not self.last_info:
            return ''
        f = self.last_info.split('|')
        try:
            return (f"centre=({float(f[0]):.0f},{float(f[1]):.0f}) "
                    f"offset={float(f[2]):+.2f} area={float(f[3]):.0f}px "
                    f"dist={float(f[8]):.2f}{self.depth_units}")
        except (IndexError, ValueError):
            return self.last_info

    # ------------------------------------------------------------- output

    def annotate_banner(self, cv_image):
        """Big top-left banner, so the state is readable in rqt_image_view."""
        text = "WINDOW LOCKED" if self.detected else "searching..."
        color = (0, 255, 0) if self.detected else (0, 165, 255)
        cv2.putText(cv_image, text, (12, 34), cv2.FONT_HERSHEY_SIMPLEX,
                    1.0, color, 2)
        if self.detected and self.last_info:
            cv2.putText(cv_image, self.describe(), (12, 62),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    def publish_frames(self, cv_image, mask, header):
        if self.image_pub is not None:
            out = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            out.header = header
            self.image_pub.publish(out)
        if self.mask_pub is not None:
            out = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
            out.header = header
            self.mask_pub.publish(out)

    def watchdog(self):
        """Complain if the camera stops, and keep /window_detected fresh.

        Without this a dead zed_wrapper looks exactly like "no window in
        sight" to anything downstream, which is the one confusion that could
        leave the vehicle yawing forever with a blind camera.
        """
        if self.last_image_time is None:
            self.get_logger().warning(
                f"No frames on {self.image_topic} yet. Is zed_wrapper running? "
                "Check `ros2 topic list | grep zed`.",
                throttle_duration_sec=5.0)
            return
        age = time.monotonic() - self.last_image_time
        if age > 2.0:
            self.get_logger().error(
                f"No camera frame for {age:.1f} s -- detection is stale.",
                throttle_duration_sec=5.0)
            if self.detected:
                self.detected = False
                self.hit_streak = 0
                self.get_logger().warning("Window detection dropped: camera went away.")
            msg = Bool()
            msg.data = False
            self.detected_pub.publish(msg)

    def destroy_node(self):
        if self.show_windows:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WindowDetect()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
