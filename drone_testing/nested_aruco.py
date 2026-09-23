#!/usr/bin/env python3
"""
aruco_pose for a NESTED landing pad: a small marker printed in the middle of a
big one. A drop-in replacement -- same topics, same frame, same debounce -- so
course_fsm and thermal_fsm land on it without a line of flight code changing.

Ported from the bench node ~/Downloads/cam/lend/nested_aruco_node.py. What
came across, and what deliberately did not:

  TAKEN
    * Both IDs belong to ONE pad. The big marker is what the camera finds from
      altitude; as the aircraft comes down it overflows the frame and the
      small one takes over. Either fix is published under ONE id (pad_id, the
      big marker's by default), so the flight node's "is this my pad" check
      sees one pad all the way down instead of the id changing under it at
      ~1.5 m.
    * The small marker is preferred whenever it decodes: it is the one still
      in frame at the bottom, and the one whose corners are largest there.
    * Measured on the bench: the two essentially never decode together -- the
      small marker covers the big one's central bits. So nothing here needs
      both at once; they simply hand over.
    * POSITION FROM THE RANGEFINDER, NOT FROM THE MARKER SIZE. The lateral
      offset is the camera ray to the marker's centre scaled by the lidar
      height. Only the intrinsics are involved: no marker size, no pose
      solve. With aruco_pose and ONE marker_size, a nested pad is wrong by
      exactly 4x on whichever marker is not that size -- which in a landing
      loop is a 4x gain error, i.e. an oscillation. solvePnP with the role's
      own size is the fallback when there is no fresh lidar height.

  LEFT BEHIND
    * The velocity command, the CENTER/DESCEND/LAND decision and the slew
      limiter. The FSMs already fly this on position setpoints, attitude-
      compensated, with their own centring gate, smoothing and PX4 land
      handoff; a second controller in the detector would fight them.
    * Auto dictionary scanning (five detector passes per frame until it locks)
      and role learning from enclosing squares / FCU height. Set the two IDs
      and the dictionary; the source node's own notes call explicit IDs the
      most reliable of its three methods.

The output frame is aruco_pose's: +x right in the image, +y up in the image,
+z up (so the marker is at z = -height). See aruco_pose.py.

    ros2 run drone_testing nested_aruco --ros-args \\
        -p big_marker_id:=1 -p small_marker_id:=2 \\
        -p big_marker_size:=0.80 -p small_marker_size:=0.20
"""


import cv2
import numpy as np

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, Float32, Header, String

from drone_testing.aruco_pose import ArucoPose, R_CF, body_words
from drone_testing.window_detect import array_to_imgmsg

try:
    from px4_msgs.msg import VehicleLocalPosition
    HAVE_PX4 = True
except ImportError:     # bench machine with no px4_msgs
    HAVE_PX4 = False


def square_points(size):
    """TL, TR, BR, BL -- the order SOLVEPNP_IPPE_SQUARE requires."""
    s = size / 2.0
    return np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]],
                    dtype=np.float64)


def quad_centre(q):
    """Where the diagonals cross: the true centre under perspective.

    The corner mean drifts toward the near edge when the marker is viewed
    obliquely -- i.e. exactly when the aircraft is tilted or off to one side.
    """
    p0, p1, p2, p3 = q
    d1, d2 = p2 - p0, p3 - p1
    den = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(den) < 1e-9:
        return q.mean(axis=0)
    t = ((p1[0] - p0[0]) * d2[1] - (p1[1] - p0[1]) * d2[0]) / den
    return p0 + t * d1


class NestedArucoPose(ArucoPose):

    BIG_MARKER_SIZE = 0.80
    SMALL_MARKER_SIZE = 0.20
    RANGE_TIMEOUT = 0.5         # s before the lidar height counts as stale
    MIN_RANGE = 0.05            # m; below this dist_bottom is not a height

    def __init__(self):
        super().__init__()
        p = self.declare_parameter
        self.big_id = int(p('big_marker_id', -1).value)
        self.small_id = int(p('small_marker_id', -1).value)
        self.sizes = {'big': float(p('big_marker_size', self.BIG_MARKER_SIZE).value),
                      'small': float(p('small_marker_size', self.SMALL_MARKER_SIZE).value)}
        pad_id = int(p('pad_id', -1).value)
        # m added to the lidar reading if the camera does not sit at the
        # rangefinder's height. Usually ~0.
        self.camera_height_offset = float(p('camera_height_offset', 0.0).value)
        self.use_rangefinder = bool(p('use_rangefinder', True).value)
        self.range_timeout = float(p('range_timeout', self.RANGE_TIMEOUT).value)

        if self.big_id < 0 or self.small_id < 0:
            self.get_logger().error(
                "big_marker_id / small_marker_id not set: the pad cannot be "
                "recognised as ONE pad and both markers are treated as plain "
                f"{self.marker_size:.2f} m markers.")
        self.roles = {}
        if self.big_id >= 0:
            self.roles[self.big_id] = 'big'
        if self.small_id >= 0:
            self.roles[self.small_id] = 'small'
        # The id every pad fix is published under.
        self.pad_id = pad_id if pad_id >= 0 else (
            self.big_id if self.big_id >= 0 else self.small_id)
        # The pad's markers always count, on top of whatever marker_ids names.
        self.marker_ids = list(dict.fromkeys(self.marker_ids + list(self.roles)))

        self.range_h = None
        self.range_time = None
        if HAVE_PX4 and self.use_rangefinder:
            qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST, depth=5)
            # Both names: see line_detect.py. The aircraft's firmware
            # publishes ..._v1 and the bare name silently receives nothing.
            for topic in ('/uav_1/fmu/out/vehicle_local_position',
                          '/uav_1/fmu/out/vehicle_local_position_v1'):
                self.create_subscription(VehicleLocalPosition, topic,
                                         self._on_position, qos)
        elif self.use_rangefinder:
            self.get_logger().error(
                "px4_msgs not importable: no lidar height, so every fix comes "
                "from solvePnP and is only as good as hfov_deg.")

        self.height_pub = self.create_publisher(Float32, '/aruco/height', 10)
        self.get_logger().warning(
            f"NESTED pad: big id {self.big_id} ({self.sizes['big']:.2f} m), "
            f"small id {self.small_id} ({self.sizes['small']:.2f} m), "
            f"published as id {self.pad_id}. Counting ids "
            f"{'/'.join(str(i) for i in self.marker_ids)}.")

    # ------------------------------------------------------------ lidar

    def _on_position(self, msg):
        # dist_bottom is the rangefinder when PX4 marks it valid; otherwise
        # -z, which on this airframe is rangefinder-referenced too.
        if msg.dist_bottom_valid:
            self.range_h = float(msg.dist_bottom)
        elif msg.z_valid:
            self.range_h = -float(msg.z)
        else:
            return
        self.range_time = self.get_clock().now().nanoseconds / 1e9

    def _lidar_height(self):
        if self.range_h is None or self.range_time is None:
            return None
        if self.get_clock().now().nanoseconds / 1e9 - self.range_time > self.range_timeout:
            return None
        h = self.range_h + self.camera_height_offset
        return h if h > self.MIN_RANGE else None

    # ------------------------------------------------------------ solve

    def _ray_fix(self, quad, height):
        """Camera ray to the marker centre, scaled to the lidar height."""
        c = quad_centre(quad).reshape(1, 1, 2).astype(np.float64)
        xn, yn = cv2.undistortPoints(c, self.K, self.D).reshape(2)
        return tuple(float(v) for v in R_CF @ np.array([xn * height, yn * height, height]))

    def _pnp_fix(self, quad, size):
        ok, _, tvec = cv2.solvePnP(square_points(size), quad, self.K, self.D,
                                   flags=cv2.SOLVEPNP_IPPE_SQUARE)
        return tuple(float(v) for v in R_CF @ tvec.reshape(3)) if ok else None

    def _choose(self, corners, seen):
        """Small marker if decoded, else the largest wanted marker."""
        best = None
        for i, mid in enumerate(seen):
            if mid not in self.marker_ids:
                continue
            q = corners[i].reshape(4, 2).astype(np.float64)
            rank = (self.roles.get(mid) == 'small',
                    cv2.contourArea(q.astype(np.float32)))
            if best is None or rank > best[0]:
                best = (rank, mid, q)
        return (None, None) if best is None else (best[1], best[2])

    # ------------------------------------------------------------ detect

    def detect_once(self):
        with self._frame_lock:
            if self._frame is None or self._frame_seq == self._processed_seq:
                return
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

        if self.frame_pub is not None:
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = 'down_camera'
            self.frame_pub.publish(array_to_imgmsg(frame, 'bgr8', header))

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detect(gray)
        seen = [int(i) for i in ids.flatten()] if ids is not None else []

        mid, quad = self._choose(corners, seen)
        pose, src = None, None
        if quad is not None:
            lidar = self._lidar_height()
            if lidar is not None:
                pose, src = self._ray_fix(quad, lidar), 'lidar'
            else:
                role = self.roles.get(mid)
                size = self.sizes[role] if role else self.marker_size
                pose, src = self._pnp_fix(quad, size), f'pnp {size:.2f} m'
            if pose is None:
                quad = None

        self._update_debounce(pose is not None)

        if pose is not None:
            out_id = self.pad_id if mid in self.roles else mid
            self.last_marker_id = out_id
            x, y, z = pose
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f'camera_body/{out_id}'
            msg.point.x, msg.point.y, msg.point.z = x, y, z
            self.point_pub.publish(msg)
            self.height_pub.publish(Float32(data=-z))
            role = self.roles.get(mid, 'plain')
            line = (f"id {mid} ({role}) -> pad {out_id}  [{src}]  "
                    f"height={-z:.2f} m  |  marker is {body_words(y, x)}")
        else:
            line = f"pad not visible (seen: {seen or '-'})"

        self.detected_pub.publish(Bool(data=self.detected))
        self.info_pub.publish(String(data=line))
        self.get_logger().info(line, throttle_duration_sec=1.0)
        self._render(frame, quad, line)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = NestedArucoPose()
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
