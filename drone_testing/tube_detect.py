"""
Find the VERTICAL tubes of the tube obstacle and publish where each one is, in
angles and depth.

This is tube_cross.py's eyes, and it is the same shape of node as bar_detect:
HSV mask -> shape test -> depth samples -> /tube_geometry, with a debounced
/tubes_detected for the flight node to gate on. Topics, intrinsics, the MJPEG
stream and the frame-rate cap are wired exactly as in window_detect and
bar_detect.

THE OBSTACLE
    Three uprights 0.5 m apart and 2 m tall, a horizontal tube joining them at
    0.461 m, and a diagonal from the top of the left upright down to the right
    one. Behind it, about 1 m back, one more free-standing upright.

    Only the UPRIGHTS are measured. They are what the flight node needs -- the
    ground position of each one fixes the plane of the obstacle and the gap
    between the left and middle tubes -- and they are the only part whose
    position does not depend on which height the camera happens to be at.

WHY THE MASK IS OPENED WITH A TALL KERNEL
    The whole obstacle is one colour and one connected piece: the uprights,
    the cross tube and the diagonal are a single contour in a colour mask, and
    no shape test on that contour finds three tubes in it.

    A morphological OPEN with a kernel one pixel wide and vertical_kernel_frac
    of the image tall keeps only what has a long vertical run in every column
    it occupies. An upright has that. The cross tube has a run of its own
    thickness. The diagonal, at 45 degrees, has a run of its thickness times
    root two. Both vanish, and what is left falls apart into one contour per
    upright.

THE FLOOR
    The arena floor is red too. A floor strip that survives the open is wedge-
    shaped and its depth changes along it by metres, so it fails the depth
    spread test here. Anything that gets past that is placed in NED by the
    flight node and has to stand on the floor and reach up past
    min_tube_top_height, which the floor cannot.

TOPICS
    /tubes_detected     Bool, debounced: at least min_tubes uprights in frame
    /tube_info          String, one line of human-readable state
    /tube_geometry      Float32MultiArray, one row of STRIDE values per tube:
                            depth_m      median depth along the centreline
                            az_deg       azimuth of the centreline, + RIGHT
                            el_deg       elevation of the depth sample, + UP
                            el_top_deg   elevation of the visible top end
                            el_bot_deg   elevation of the visible bottom end
                            trunc_top    1.0 if the top is cut by the frame
                            trunc_bot    1.0 if the bottom is cut by the frame
                            width_px     apparent thickness
                        An EMPTY array means "frame processed, no tubes".
    /tube_detection/image[/compressed]   the debug overlay

    q/k mean nothing here; this node does not fly anything.
"""

import math
import threading
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Bool, Float32MultiArray, MultiArrayDimension, String

from drone_testing.bar_detect import rect_axes
from drone_testing.window_detect import (HSV_RANGES, MjpegServer, array_to_imgmsg,
                                         get_median_depth, hsv_mask,
                                         imgmsg_to_bgr, imgmsg_to_depth)

STRIDE = 8


def vertical_tubes(mask, kernel_px, min_area, min_aspect, max_tilt_deg):
    """Every upright-shaped contour left after a vertical open, left to right.

    Returns (opened_mask, [(contour, centre, half, long_len, short_len,
    aspect, tilt), ...]).
    """
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1, max(3, int(kernel_px))))
    opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    contours, _ = cv2.findContours(opened, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    found = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue
        centre, half, long_len, short_len = rect_axes(cv2.minAreaRect(contour))
        if short_len < 1.0:
            continue
        aspect = long_len / short_len
        if aspect < min_aspect:
            continue
        # Angle of the long axis off VERTICAL in the image, folded to [0, 90].
        tilt = abs(math.degrees(math.atan2(half[0], half[1])))
        if tilt > 90.0:
            tilt = 180.0 - tilt
        if tilt > max_tilt_deg:
            continue
        found.append((contour, centre, half, long_len, short_len, aspect, tilt))
    found.sort(key=lambda f: f[1][0])
    return opened, found


class TubeDetect(Node):

    IMAGE_TOPIC = '/zed/zed_node/rgb/color/rect/image'
    DEPTH_TOPIC = '/zed/zed_node/depth/depth_registered'
    CAMERA_INFO_TOPIC = 'auto'

    FALLBACK_HFOV_DEG = 90.0
    MAX_FPS = 10.0

    DETECT_FRAMES = 3
    LOST_FRAMES = 5
    MIN_TUBES = 2               # uprights in frame for /tubes_detected

    MIN_AREA = 600
    MIN_ASPECT = 4.0
    MAX_TILT_DEG = 15.0         # of the long axis off vertical, in the image
    VERTICAL_KERNEL_FRAC = 0.10 # of the image height. Must be longer than the
                                # vertical run of the diagonal where it is
                                # thickest in the image (up close), and shorter
                                # than the visible length of an upright.
    SAMPLES_ALONG = 9
    SAMPLE_BOX = 2
    BORDER_MARGIN = 8
    DEPTH_MAX_AGE = 0.5
    DEPTH_MIN = 0.30
    DEPTH_MAX = 10.0
    LOG_PERIOD = 1.0
    JPEG_QUALITY = 60
    STREAM_PORT = 8082          # 8080 is window_detect, 8081 bar_detect

    def _num(self, name, default):
        """Numeric parameter that takes `3` and `3.0` alike (see bar_detect)."""
        from rcl_interfaces.msg import ParameterDescriptor
        return self.declare_parameter(
            name, default, ParameterDescriptor(dynamic_typing=True)).value

    def __init__(self):
        super().__init__('tube_detect')

        self.image_topic = str(self.declare_parameter('image_topic', self.IMAGE_TOPIC).value)
        self.depth_topic = str(self.declare_parameter('depth_topic', self.DEPTH_TOPIC).value)
        self.camera_info_topic = str(self.declare_parameter(
            'camera_info_topic', self.CAMERA_INFO_TOPIC).value).strip()
        if self.camera_info_topic in ('', 'auto'):
            self.camera_info_topic = self.image_topic.rsplit('/', 1)[0] + '/camera_info'
            self.get_logger().info(
                f"camera_info_topic derived from image_topic: {self.camera_info_topic}")

        self.color = str(self.declare_parameter('color', 'red').value).strip().lower()
        if self.color not in HSV_RANGES:
            raise SystemExit(
                f"Unknown color '{self.color}'; expected one of {sorted(HSV_RANGES)}")

        self.min_area = float(self._num('min_area', float(self.MIN_AREA)))
        self.min_aspect = float(self._num('min_aspect', self.MIN_ASPECT))
        self.max_tilt_deg = float(self._num('max_tilt_deg', self.MAX_TILT_DEG))
        self.kernel_frac = float(self._num('vertical_kernel_frac', self.VERTICAL_KERNEL_FRAC))
        self.samples_along = int(self._num('samples_along', self.SAMPLES_ALONG))
        self.border_margin = float(self._num('border_margin', float(self.BORDER_MARGIN)))
        self.min_tubes = int(self._num('min_tubes', self.MIN_TUBES))
        self.detect_frames = int(self._num('detect_frames', self.DETECT_FRAMES))
        self.lost_frames = int(self._num('lost_frames', self.LOST_FRAMES))
        self.depth_min = float(self._num('depth_min', self.DEPTH_MIN))
        self.depth_max = float(self._num('depth_max', self.DEPTH_MAX))
        self.fallback_hfov = math.radians(float(self._num(
            'fallback_hfov_deg', self.FALLBACK_HFOV_DEG)))
        self.max_fps = float(self._num('max_fps', self.MAX_FPS))
        self.min_frame_interval = (1.0 / self.max_fps) if self.max_fps > 0.0 else 0.0

        self.publish_image = bool(self.declare_parameter('publish_image', True).value)
        self.publish_compressed = bool(self.declare_parameter('publish_compressed', True).value)
        self.publish_mask = bool(self.declare_parameter('publish_mask', False).value)
        self.jpeg_quality = int(self._num('jpeg_quality', self.JPEG_QUALITY))
        self.stream_port = int(self._num('stream_port', self.STREAM_PORT))
        self.stream_scale = float(self._num('stream_scale', 0.5))

        self.create_subscription(Image, self.image_topic,
                                 self.image_callback, qos_profile_sensor_data)
        self.create_subscription(Image, self.depth_topic,
                                 self.depth_callback, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.camera_info_topic,
                                 self.camera_info_callback, qos_profile_sensor_data)

        self.detected_pub = self.create_publisher(Bool, 'tubes_detected', 10)
        self.info_pub = self.create_publisher(String, 'tube_info', 10)
        self.geometry_pub = self.create_publisher(Float32MultiArray, 'tube_geometry', 10)
        self.image_pub = (self.create_publisher(Image, 'tube_detection/image', 1)
                          if self.publish_image else None)
        self.mask_pub = (self.create_publisher(Image, 'tube_detection/mask', 1)
                         if self.publish_mask else None)
        self.compressed_pub = (self.create_publisher(
            CompressedImage, 'tube_detection/image/compressed', 1)
            if self.publish_compressed else None)

        self._jpeg = None
        self._jpeg_lock = threading.Lock()
        self.stream = MjpegServer(self, self.stream_port) if self.stream_port else None

        self.depth_image = None
        self.depth_time = 0.0
        self.intrinsics = None
        self.last_processed = 0.0
        self.frames = 0
        self.frames_skipped = 0
        self.first_frame_logged = False

        self.hit_streak = 0
        self.miss_streak = 0
        self.detected = False
        self.last_log = 0.0
        self.last_info = ''

        self.get_logger().warning(
            f"Tube detection up. colour={self.color} image={self.image_topic} "
            f"depth={self.depth_topic} -> /tubes_detected, /tube_geometry"
            + (f", MJPEG on :{self.stream_port}" if self.stream else ""))

    # ------------------------------------------------------------------ subs

    def camera_info_callback(self, msg):
        k = msg.k
        if k[0] > 0.0 and k[4] > 0.0 and self.intrinsics is None:
            self.intrinsics = (float(k[0]), float(k[4]), float(k[2]), float(k[5]))
            self.get_logger().info(
                f"CameraInfo from {self.camera_info_topic}: fx={k[0]:.1f} "
                f"fy={k[4]:.1f} cx={k[2]:.1f} cy={k[5]:.1f}.")

    def depth_callback(self, msg):
        try:
            self.depth_image = imgmsg_to_depth(msg)
            self.depth_time = time.monotonic()
        except Exception as exc:
            self.get_logger().error(f"Cannot convert depth frame: {exc}",
                                    throttle_duration_sec=5.0)

    def current_depth(self):
        if self.depth_image is None:
            return None
        if time.monotonic() - self.depth_time > self.DEPTH_MAX_AGE:
            return None
        return self.depth_image

    def _intrinsics(self, shape):
        if self.intrinsics is not None:
            return self.intrinsics
        h, w = shape[:2]
        fx = (w / 2.0) / math.tan(self.fallback_hfov / 2.0)
        self.get_logger().warning(
            f"No CameraInfo on {self.camera_info_topic} yet; guessing the "
            f"intrinsics from fallback_hfov_deg={math.degrees(self.fallback_hfov):.0f}. "
            "Every tube position is scaled wrong until this arrives.",
            throttle_duration_sec=5.0)
        return fx, fx, w / 2.0, h / 2.0

    def image_callback(self, msg):
        now = time.monotonic()
        if self.min_frame_interval and now - self.last_processed < self.min_frame_interval:
            self.frames_skipped += 1
            return
        self.last_processed = now
        try:
            cv_image = imgmsg_to_bgr(msg)
        except Exception as exc:
            self.get_logger().error(f"Cannot convert image frame: {exc}",
                                    throttle_duration_sec=5.0)
            return
        self.frames += 1
        if not self.first_frame_logged:
            self.first_frame_logged = True
            h, w = cv_image.shape[:2]
            self.get_logger().info(f"First frame from {self.image_topic}: {w}x{h}.")
        self.process(cv_image, msg.header)

    # ------------------------------------------------------------ detection

    def process(self, cv_image, header):
        h, w = cv_image.shape[:2]
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = hsv_mask(hsv, self.color)
        opened, candidates = vertical_tubes(
            mask, self.kernel_frac * h, self.min_area, self.min_aspect, self.max_tilt_deg)

        depth_img = self.current_depth()
        rows = []
        rejects = []
        if depth_img is None:
            rejects.append('no depth frame')
        else:
            fx, fy, cx, cy = self._intrinsics(cv_image.shape)
            for cand in candidates:
                row, why = self._measure(cv_image, depth_img, cand, fx, fy, cx, cy)
                if row is None:
                    rejects.append(why)
                else:
                    rows.append(row)

        if len(rows) >= self.min_tubes:
            self._hit()
        else:
            self._miss()

        self._publish_geometry(rows)
        info = (f"{len(rows)} tube(s): "
                + ", ".join(f"d={r[0]:.2f} az={r[1]:+.0f}" for r in rows))
        if rejects:
            info += f" | rejected: {'; '.join(rejects[:3])}"
        self.last_info = info
        self._publish_state(info)
        self._publish_images(cv_image, opened, header)
        self._log()

    def _measure(self, img, depth_img, cand, fx, fy, cx, cy):
        contour, centre, half, long_len, short_len, aspect, tilt = cand
        h, w = img.shape[:2]
        p_top = centre - half * 0.92
        p_bot = centre + half * 0.92
        if p_top[1] > p_bot[1]:
            p_top, p_bot = p_bot, p_top

        samples = []
        n = max(3, self.samples_along)
        for i in range(n):
            t = i / (n - 1.0)
            p = p_top + (p_bot - p_top) * t
            u = int(round(min(max(p[0], 0), w - 1)))
            v = int(round(min(max(p[1], 0), h - 1)))
            d = get_median_depth(depth_img, u, v, self.SAMPLE_BOX)
            if self.depth_min <= d <= self.depth_max:
                samples.append((d, (u, v)))
        if len(samples) < 3:
            return None, f'{len(samples)} depth samples'

        depths = np.array([s[0] for s in samples])
        median_depth = float(np.median(depths))
        # An upright seen from roughly level is at nearly one depth end to end.
        # A floor strip is not: that is what this catches.
        spread = float(np.max(np.abs(depths - median_depth)))
        allowed = max(0.30, 0.20 * median_depth)
        if spread > allowed:
            return None, f'depth spread {spread:.2f} m'

        # The sample nearest the median depth is the point that gets placed.
        k = int(np.argmin(np.abs(depths - median_depth)))
        su, sv = samples[k][1]

        az = math.degrees(math.atan2(float(su) - cx, fx))
        el = math.degrees(math.atan2(cy - float(sv), fy))
        el_top = math.degrees(math.atan2(cy - float(p_top[1]), fy))
        el_bot = math.degrees(math.atan2(cy - float(p_bot[1]), fy))
        trunc_top = p_top[1] < self.border_margin
        trunc_bot = p_bot[1] > h - 1 - self.border_margin

        self._draw(img, contour, p_top, p_bot, samples, median_depth, az)
        return [median_depth, az, el, el_top, el_bot,
                1.0 if trunc_top else 0.0, 1.0 if trunc_bot else 0.0,
                float(short_len)], ''

    def _publish_geometry(self, rows):
        msg = Float32MultiArray()
        msg.layout.dim = [
            MultiArrayDimension(label='tube', size=len(rows), stride=STRIDE * len(rows)),
            MultiArrayDimension(label='values', size=STRIDE, stride=STRIDE),
        ]
        msg.data = [float(v) for row in rows for v in row]
        self.geometry_pub.publish(msg)

    # ------------------------------------------------------------- debounce

    def _hit(self):
        self.hit_streak += 1
        self.miss_streak = 0
        if not self.detected and self.hit_streak >= self.detect_frames:
            self.detected = True
            self.get_logger().warning("TUBES DETECTED")

    def _miss(self):
        self.miss_streak += 1
        self.hit_streak = 0
        if self.detected and self.miss_streak >= self.lost_frames:
            self.detected = False
            self.get_logger().warning("Tubes LOST.")

    def _publish_state(self, info):
        msg = Bool()
        msg.data = self.detected
        self.detected_pub.publish(msg)
        text = String()
        text.data = info
        self.info_pub.publish(text)

    def _log(self):
        now = time.monotonic()
        if now - self.last_log < self.LOG_PERIOD:
            return
        self.last_log = now
        self.get_logger().info(
            f"tubes: {'YES' if self.detected else 'no '} ({self.last_info}; "
            f"{self.frames} frames, {self.frames_skipped} skipped)")

    # --------------------------------------------------------------- output

    def _draw(self, img, contour, p_top, p_bot, samples, depth, az):
        cv2.drawContours(img, [contour], -1, (255, 0, 255), 2)
        cv2.line(img, tuple(np.round(p_top).astype(int)),
                 tuple(np.round(p_bot).astype(int)), (0, 255, 0), 2)
        for _, pt in samples:
            cv2.circle(img, pt, 3, (0, 0, 255), -1)
        cv2.putText(img, f"d={depth:.2f} az={az:+.0f}",
                    (int(p_top[0]) - 40, max(20, int(p_top[1]) + 20)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

    def _publish_images(self, cv_image, mask, header):
        if self.image_pub is not None:
            self.image_pub.publish(array_to_imgmsg(cv_image, 'bgr8', header))
        if self.mask_pub is not None:
            self.mask_pub.publish(array_to_imgmsg(mask, 'mono8', header))
        if self.compressed_pub is None and self.stream is None:
            return
        frame = cv_image
        if self.stream_scale and self.stream_scale != 1.0:
            frame = cv2.resize(frame, None, fx=self.stream_scale,
                               fy=self.stream_scale, interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode('.jpg', frame,
                               [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            return
        payload = buf.tobytes()
        if self.compressed_pub is not None:
            out = CompressedImage()
            out.header = header
            out.format = 'jpeg'
            out.data = payload
            self.compressed_pub.publish(out)
        with self._jpeg_lock:
            self._jpeg = payload

    def latest_jpeg(self):
        with self._jpeg_lock:
            return self._jpeg

    def destroy_node(self):
        if self.stream is not None:
            self.stream.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TubeDetect()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
