"""
Does the DOWNWARD camera see a coloured bar underneath the aircraft?

The red bar's crossing confirmation for course_fsm's measured-bar mode
(bars_measured:=true). The front camera loses the red bar long before the
aircraft is over it -- it drops out of the bottom of the field of view -- so
the only camera that can say "we are passing over it now" is the one
looking down. This node answers exactly that and nothing more: no depth (the
down camera has none), no geometry, just a debounced Bool.

THE TEST
    HSV mask -> the same bar-shape test bar_detect uses (minAreaRect, long
    side min_aspect times the short one) -> the shape's long axis must span
    at least min_span_frac of the image and its centre must lie inside the
    central centre_band_frac of the frame along the direction of travel.
    The band is what turns "a red thing is visible" into "a red thing is
    UNDER us".

THE CAVEAT
    The arena floor has red strips. A floor strip seen from above is also a
    long red rectangle, so this node on its own cannot tell the bar from a
    strip -- course_fsm only listens to it during RED_CROSS, where the bar is
    by far the nearest (and so largest) red thing below, and treats a
    missing confirmation as a warning, not a reason to stop. Tune
    min_span_frac / min_area on the real floor before trusting it more.

TOPICS
    in   image_topic                 sensor_msgs/Image, the down camera
    out  bar_down/detected           Bool, debounced
         bar_down/info               String, one line of state
"""

import math
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from drone_testing.bar_detect import bar_detection
from drone_testing.window_detect import HSV_RANGES, hsv_mask, imgmsg_to_bgr


class BarDownCheck(Node):

    MIN_AREA = 1500.0           # px, of the bar contour
    MIN_ASPECT = 3.0            # long / short side of the minAreaRect
    MAX_TILT_DEG = 35.0         # of the long axis off the image's
                                # across-track axis
    MIN_SPAN_FRAC = 0.35        # long side / image size along it
    CENTRE_BAND_FRAC = 0.50     # central fraction of the frame, along
                                # track, the bar centre must be in
    DETECT_FRAMES = 2
    LOST_FRAMES = 3
    MAX_FPS = 15.0

    def _num(self, name, default):
        from rcl_interfaces.msg import ParameterDescriptor
        return self.declare_parameter(
            name, default, ParameterDescriptor(dynamic_typing=True)).value

    def __init__(self):
        super().__init__('bar_down_check')
        self.image_topic = str(self.declare_parameter(
            'image_topic', '/camera/down/image_raw').value)
        self.color = str(self.declare_parameter('color', 'red').value).strip().lower()
        if self.color not in HSV_RANGES:
            raise SystemExit(
                f"Unknown color '{self.color}'; expected one of {sorted(HSV_RANGES)}")
        self.min_area = float(self._num('min_area', self.MIN_AREA))
        self.min_aspect = float(self._num('min_aspect', self.MIN_ASPECT))
        self.max_tilt_deg = float(self._num('max_tilt_deg', self.MAX_TILT_DEG))
        self.min_span_frac = float(self._num('min_span_frac', self.MIN_SPAN_FRAC))
        self.centre_band_frac = float(self._num('centre_band_frac', self.CENTRE_BAND_FRAC))
        # Which image axis the aircraft travels along. 'v' = forward is up/down
        # in the image (the bar then lies left-right, i.e. horizontal), 'u' =
        # forward is left/right in the image (the bar lies vertical).
        self.travel_axis = str(self.declare_parameter('travel_axis', 'v').value).strip().lower()
        self.detect_frames = int(self._num('detect_frames', self.DETECT_FRAMES))
        self.lost_frames = int(self._num('lost_frames', self.LOST_FRAMES))
        max_fps = float(self._num('max_fps', self.MAX_FPS))
        self.min_interval = 1.0 / max_fps if max_fps > 0.0 else 0.0

        self.create_subscription(Image, self.image_topic, self.image_callback,
                                 qos_profile_sensor_data)
        self.detected_pub = self.create_publisher(Bool, 'bar_down/detected', 10)
        self.info_pub = self.create_publisher(String, 'bar_down/info', 10)

        self.hit_streak = 0
        self.miss_streak = 0
        self.detected = False
        self.last_processed = 0.0
        self.get_logger().warning(
            f"Down-camera {self.color} bar check on {self.image_topic}: "
            f"span >= {self.min_span_frac:.0%} of the frame, centre in the "
            f"middle {self.centre_band_frac:.0%} along track ({self.travel_axis}).")

    def image_callback(self, msg):
        now = time.monotonic()
        if self.min_interval and now - self.last_processed < self.min_interval:
            return
        self.last_processed = now
        try:
            img = imgmsg_to_bgr(msg)
        except Exception as exc:
            self.get_logger().error(f"Cannot convert image frame: {exc}",
                                    throttle_duration_sec=5.0)
            return
        hit, info = self.check(img)
        self.update(hit, info)

    def check(self, img):
        h, w = img.shape[:2]
        if self.travel_axis == 'u':
            # Rotate so the bar is always horizontal for bar_detection's
            # tilt test, and "along track" is always the image rows.
            img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
            h, w = w, h
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = hsv_mask(hsv, self.color)
        found = bar_detection(mask, self.min_area, self.min_aspect, self.max_tilt_deg)
        if found is None:
            return False, 'no bar-shaped contour'
        _, centre, _, long_len, _, aspect, tilt = found
        span = long_len / float(w)
        if span < self.min_span_frac:
            return False, f'span {span:.0%} < {self.min_span_frac:.0%}'
        off = abs(float(centre[1]) - h / 2.0) / (h / 2.0)
        if off > self.centre_band_frac:
            return False, f'bar {off:.0%} off centre along track'
        return True, f'span={span:.0%} off={off:.0%} aspect={aspect:.1f} tilt={tilt:.0f}'

    def update(self, hit, info):
        if hit:
            self.hit_streak += 1
            self.miss_streak = 0
            if not self.detected and self.hit_streak >= self.detect_frames:
                self.detected = True
                self.get_logger().warning(f"BAR BELOW ({info})")
        else:
            self.miss_streak += 1
            self.hit_streak = 0
            if self.detected and self.miss_streak >= self.lost_frames:
                self.detected = False
                self.get_logger().warning("Bar below: gone.")
        out = Bool()
        out.data = self.detected
        self.detected_pub.publish(out)
        s = String()
        s.data = info
        self.info_pub.publish(s)


def main(args=None):
    rclpy.init(args=args)
    node = BarDownCheck()
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
