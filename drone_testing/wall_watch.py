#!/usr/bin/env python3
"""HOW FAR IS THE THING IN FRONT? The ZED depth image, reduced to one number.

    /zed/zed_node/depth/depth_registered -> /wall/clearance, /wall/close

The thermal mission's outbound leg ends either with an ArUco marker under the
down camera or with the end of the corridor in front of the aircraft. This
node answers the second half: the distance to the nearest solid thing ahead,
and whether that is closer than the stop distance.

THE OBSTACLE DOES NOT FILL THE FRAME

    It is a box at the end of the run, not a wall from floor to ceiling, so
    "the minimum of the depth image" would be answering a different question
    and "the median" would be answering it about the room behind. What this
    does instead is count: it takes a low PERCENTILE of the valid depths
    inside a central region and requires at least min_pixels of them to be
    that close before it believes any of it. A few hundred pixels agreeing is
    an object; thirty pixels agreeing is a speckle on a shiny floor.

THE FLOOR IS THE TRAP, AND IT IS NOT A SMALL ONE

    A forward-looking camera on a multirotor is not looking forward. To hold
    0.8 m/s the aircraft pitches nose-down about 10 deg, and the ZED's vertical
    field of view is wide enough that the floor then fills the bottom of the
    frame -- at 1.8 m altitude and 10 deg of pitch the floor crosses the lower
    edge of the image about 2.5 m ahead. A naive "closest thing in the frame"
    reads that floor as an obstacle, decides the corridor has ended, and stops
    the aircraft in the middle of the arena. It would do it MORE the faster you
    fly, because pitch grows with speed, which is the worst possible failure
    to debug from a log.

    So every pixel is turned into a 3-D point in the camera frame, rotated by
    the aircraft's live attitude into a world-levelled frame, and any point
    lower than floor_margin above the floor is thrown away before anything is
    measured. That needs the attitude (VehicleAttitude) and the height
    (rangefinder); without either, the node says so and reports no clearance
    rather than guessing, because a guess here stops the mission.

WHAT IT PUBLISHES

    /wall/clearance     std_msgs/Float32    metres to the nearest thing ahead
                                            that is not the floor. NaN when it
                                            cannot tell -- NOT a large number,
                                            which would read as "all clear".
    /wall/close         std_msgs/Bool       debounced: true only after
                                            detect_frames consecutive frames
                                            inside stop_distance.
    /wall/info          std_msgs/String     one human-readable line.

CHECKING IT BEFORE IT STOPS A MISSION

    ros2 run drone_testing wall_watch --ros-args -p stop_distance:=1.5
    ros2 topic echo /wall/info

    Walk a box towards the aircraft and watch the clearance come down. Then
    PITCH THE AIRCRAFT NOSE-DOWN BY HAND over a bare floor with nothing in
    front of it and check the clearance does NOT collapse -- that is the floor
    rejection working, and it is the only part of this node that a static
    bench test will not exercise by itself.
"""

import math
import time

import numpy as np
import rclpy
from px4_msgs.msg import (EstimatorStatusFlags, VehicleAttitude,
                          VehicleLocalPosition)
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32, String

from drone_testing.window_detect import imgmsg_to_depth
from drone_testing import px4_height


class WallWatch(Node):

    def __init__(self):
        super().__init__('wall_watch')

        def p(name, default):
            return self.declare_parameter(name, default).value

        self.depth_topic = str(p('depth_topic',
                                 '/zed/zed_node/depth/depth_registered'))
        self.info_topic = str(p('camera_info_topic',
                                '/zed/zed_node/depth/camera_info'))

        self.STOP_DISTANCE = float(p('stop_distance', 1.50))
        self.MIN_PIXELS = int(p('min_pixels', 300))
        self.PERCENTILE = float(p('percentile', 2.0))
        self.ROI_WIDTH = float(p('roi_width', 0.7))
        self.ROI_HEIGHT = float(p('roi_height', 0.8))
        self.MIN_RANGE = float(p('min_range', 0.30))
        self.MAX_RANGE = float(p('max_range', 12.0))
        self.FLOOR_MARGIN = float(p('floor_margin', 0.35))
        self.CEILING_MARGIN = float(p('ceiling_margin', 3.0))
        self.DETECT_FRAMES = int(p('detect_frames', 3))
        self.CLEAR_FRAMES = int(p('clear_frames', 5))
        self.DOWNSAMPLE = int(p('downsample', 4))
        self.CAM_PITCH = math.radians(float(p('camera_pitch_deg', 0.0)))
        self.MAX_AGE = float(p('max_age', 0.5))
        self.REQUIRE_ATTITUDE = bool(p('require_attitude', True))

        self.clearance_pub = self.create_publisher(Float32, '/wall/clearance', 10)
        self.close_pub = self.create_publisher(Bool, '/wall/close', 10)
        self.info_pub = self.create_publisher(String, '/wall/info', 10)

        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST, depth=5)
        # BOTH NAMES, and this is not belt-and-braces. PX4 renamed its
        # published topics with message versioning: on the aircraft's firmware
        # the position arrives on ..._v1 and a subscriber to the bare name is
        # matched with NOTHING. It does not error, it does not warn, it simply
        # never receives -- which cost a bench session already, and is why
        # every flight node in this package subscribes to the pair.
        for topic in ('/fmu/out/vehicle_local_position',
                      '/fmu/out/vehicle_local_position_v1'):
            self.create_subscription(VehicleLocalPosition, topic,
                                     self._on_position, px4_qos)
        self.create_subscription(EstimatorStatusFlags,
                                 px4_height.RANGEFINDER_TOPIC,
                                 self._on_flags, px4_qos)
        for topic in ('/fmu/out/vehicle_attitude',
                      '/fmu/out/vehicle_attitude_v1'):
            self.create_subscription(VehicleAttitude, topic,
                                     self._on_attitude, px4_qos)
        self.create_subscription(Image, self.depth_topic, self._on_depth,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.info_topic, self._on_info,
                                 qos_profile_sensor_data)

        self.local_position = None
        self.estimator_flags = None
        self.attitude = None            # (roll, pitch, yaw), rad
        self.attitude_time = None
        self.intrinsics = None          # (fx, fy, cx, cy)
        self.close = False
        self.hits = 0
        self.misses = 0
        self.clearance = float('nan')

        self.get_logger().warning(
            f"WALL: depth from {self.depth_topic}, stop at "
            f"{self.STOP_DISTANCE:.2f} m, needs {self.MIN_PIXELS} pixels at the "
            f"{self.PERCENTILE:.0f}th percentile inside the central "
            f"{self.ROI_WIDTH * 100:.0f}%x{self.ROI_HEIGHT * 100:.0f}% of the "
            f"frame. Floor rejected below {self.FLOOR_MARGIN:.2f} m above it. "
            "-> /wall/clearance, /wall/close")

    # ------------------------------------------------------------- the inputs

    def _on_position(self, msg):
        self.local_position = msg

    def _on_flags(self, msg):
        self.estimator_flags = msg

    def _on_attitude(self, msg):
        # PX4 VehicleAttitude.q is (w, x, y, z), body FRD in NED.
        w, x, y, z = (float(msg.q[0]), float(msg.q[1]),
                      float(msg.q[2]), float(msg.q[3]))
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        s = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
        pitch = math.asin(s)
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        self.attitude = (roll, pitch, yaw)
        self.attitude_time = time.monotonic()

    def _on_info(self, msg):
        if msg.k[0] > 0.0:
            self.intrinsics = (float(msg.k[0]), float(msg.k[4]),
                               float(msg.k[2]), float(msg.k[5]))

    # ---------------------------------------------------------- the measuring

    def _on_depth(self, msg):
        try:
            depth = imgmsg_to_depth(msg)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=5.0)
            return

        clearance, why = self._clearance(depth)
        self.clearance = clearance

        if math.isnan(clearance):
            # Unknown is NOT clear. Do not let a dead depth topic read as a
            # corridor that goes on for ever, and do not let it read as a wall
            # either -- hold whatever the debounce last decided and say why.
            self.hits = 0
            self.misses = 0
            self.clearance_pub.publish(Float32(data=float('nan')))
            self.close_pub.publish(Bool(data=self.close))
            self._say(f"clearance UNKNOWN: {why}")
            return

        if clearance <= self.STOP_DISTANCE:
            self.misses = 0
            self.hits += 1
            if not self.close and self.hits >= self.DETECT_FRAMES:
                self.close = True
                self.get_logger().warning(
                    f"WALL at {clearance:.2f} m, inside the "
                    f"{self.STOP_DISTANCE:.2f} m stop distance, for "
                    f"{self.DETECT_FRAMES} frames.")
        else:
            self.hits = 0
            self.misses += 1
            if self.close and self.misses >= self.CLEAR_FRAMES:
                self.close = False
                self.get_logger().warning(f"WALL cleared: {clearance:.2f} m.")

        self.clearance_pub.publish(Float32(data=float(clearance)))
        self.close_pub.publish(Bool(data=self.close))
        self._say(f"{clearance:.2f} m ahead ({why})"
                  + (" -- CLOSE" if self.close else ""))

    def _clearance(self, depth):
        """(metres, why) to the nearest non-floor thing ahead. NaN if unknown."""
        if self.intrinsics is None:
            return float('nan'), f"no CameraInfo on {self.info_topic}"

        h, w = depth.shape[:2]
        step = max(1, self.DOWNSAMPLE)
        # The central region only: the corridor is what matters, and the frame
        # edges are where a doorway or a passing person lives.
        r0 = int(h * (1.0 - self.ROI_HEIGHT) / 2.0)
        r1 = int(h - r0)
        c0 = int(w * (1.0 - self.ROI_WIDTH) / 2.0)
        c1 = int(w - c0)
        roi = depth[r0:r1:step, c0:c1:step]
        rows = np.arange(r0, r1, step)[:, None]
        cols = np.arange(c0, c1, step)[None, :]

        z = roi.astype(np.float64)
        good = np.isfinite(z) & (z > self.MIN_RANGE) & (z < self.MAX_RANGE)
        if not np.any(good):
            return float('nan'), "no valid depth in the region of interest"

        height, _ = self._agl_or_none()
        attitude_ok = (self.attitude is not None and self.attitude_time is not None
                       and time.monotonic() - self.attitude_time <= self.MAX_AGE)
        if height is None or not attitude_ok:
            if self.REQUIRE_ATTITUDE:
                missing = []
                if height is None:
                    missing.append(px4_height.why_no_height(
                        self.estimator_flags, self.local_position))
                if not attitude_ok:
                    missing.append("no fresh /fmu/out/vehicle_attitude")
                return float('nan'), ("cannot reject the floor: "
                                      + "; ".join(missing))
            # Explicitly asked to fly blind to the floor. Only honest on a
            # bench where the camera is level and there is no floor in frame.
            valid = z[good]
            return float(np.percentile(valid, self.PERCENTILE)), \
                "floor rejection OFF"

        fx, fy, cx, cy = self.intrinsics
        # Camera optical frame: +x right, +y down, +z forward.
        x = (cols - cx) / fx * z
        y = (rows - cy) / fy * z
        # Into body FRD: forward = z, right = x, down = y, after the camera's
        # own fixed downward tilt on the airframe.
        cp, sp = math.cos(self.CAM_PITCH), math.sin(self.CAM_PITCH)
        fwd = z * cp + y * sp
        down_body = y * cp - z * sp
        right = x
        # Level it with the live attitude. Only the DOWN component is needed,
        # because the only test is "is this point near the floor".
        roll, pitch, _ = self.attitude
        sr, cr = math.sin(roll), math.cos(roll)
        sq, cq = math.sin(pitch), math.cos(pitch)
        down_world = -fwd * sq + right * cq * sr + down_body * cq * cr
        above_floor = height - down_world

        keep = (good
                & (above_floor > self.FLOOR_MARGIN)
                & (above_floor < height + self.CEILING_MARGIN)
                & (fwd > self.MIN_RANGE))
        count = int(np.count_nonzero(keep))
        if count < self.MIN_PIXELS:
            return float('inf'), (f"clear: only {count} non-floor pixels, "
                                  f"need {self.MIN_PIXELS}")
        ahead = fwd[keep]
        value = float(np.percentile(ahead, self.PERCENTILE))
        return value, f"{count} px, {self.PERCENTILE:.0f}th pct"

    def _agl_or_none(self):
        height = px4_height.agl(self.estimator_flags, self.local_position)
        return height, height is not None

    def _say(self, text):
        self.info_pub.publish(String(data=text))
        self.get_logger().info(text, throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = WallWatch()
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
