#!/usr/bin/env python3
"""
Find the hottest of three boxes with a downward MLX90640, put the DROP POINT
over it, descend to drop height and hover there. ARK Flow localisation.

    arm -> ground wait -> climb to survey_altitude -> hold -> SURVEY (map every
    warm blob in NED, from the centre and, if needed, a small ring of points)
    -> pick the hottest -> APPROACH (fly the drop point over it at survey
    altitude) -> DESCEND (step down only while centred) -> HOVER (confirm
    alignment at drop_altitude, publish thermal_drop/ready, hold) -> land.

    q -> abort into a controlled descent.   k -> force-disarm.

Everything that can hurt somebody -- arming, estimator health gates, the
climb and descent ramps, touchdown -- is inherited from OffboardSequence,
exactly as precision_land and bar_cross do.

THE SENSOR, AND WHAT A PIXEL MEANS
    32 x 24 pixels over 110 x 75 degrees: ~3.4 deg a pixel, ~9 cm at nadir
    from 1.5 m, and a lot more towards the edges. Every pixel centre is turned
    into a ray with a tangent (pinhole) model, rotated body -> NED with the
    FULL attitude quaternion (so the tilt the aircraft uses to move does not
    show up as a phantom offset), and intersected with the plane box_height
    above the arming plane. Single hottest pixels are noisy, so what is
    tracked is the temperature-weighted centroid of the blob around each peak.

    Mounting assumed: lens straight down, image TOP towards the NOSE, image
    RIGHT towards the aircraft's RIGHT. If yours differs use cam_yaw_deg /
    flip_lr / flip_ud -- and prove it with mode:=bench before flying: a sign
    error flies the aircraft AWAY from the box, accelerating.

WHY SURVEY FIRST
    At 1.5 m the footprint is ~4.3 x 2.3 m, so a 1 x 1 m box area is normally
    in one frame. But an edge-of-frame box reads colder (it covers fewer
    pixels and the lens rolls off), so a box seen once off to the side is not
    compared fairly. Every blob is therefore clustered in NED over many frames
    and scored on the 90th percentile of its peak temperature. If fewer than
    expected_boxes clusters show up at the centre, the aircraft visits a ring
    of survey_step offsets to get each box nearer nadir.

THE OFFSET PARAMETERS
    cam_from_drop_forward / cam_from_drop_right: where the CAMERA sits relative
    to the DROP POINT (the release point of the mechanism), metres, forward
    and right positive. This is what puts the payload -- not the camera --
    over the box. drop_from_cog_forward / _right: where the drop point sits
    relative to the flight controller's position (the CoG); leave 0 if the
    mechanism is under the middle.

HEIGHT, AND THE ONE THING TO WATCH
    drop_altitude is height above the ARMING PLANE (the floor), hard-floored
    at 0.50 m. Beware: the ARK Flow's rangefinder sees the TOP of the box once
    the aircraft is over it. With a 0.3 m box at 0.5 m that is 0.2 m of range
    -- below FLOW_MIN_AGL -- so flow and range fusion can drop out right at
    the moment it matters. The base class then lands. Check the box height
    against drop_altitude before the first flight (0.5 m + box height above
    the lidar's minimum range is what you actually want).
"""

import collections
import math
import threading
import time

import numpy as np
import rclpy
from px4_msgs.msg import VehicleAttitude, VehicleStatus
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from drone_testing.offboard_sequence import OffboardSequence, spin_node, wrap_pi

H, W = 24, 32


def quat_rotate(q, v):
    """Rotate a 3-vector by a (w, x, y, z) quaternion (body FRD -> NED)."""
    w, x, y, z = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return np.array([
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    ])


def find_blobs(grid, min_contrast, max_blobs=6):
    """Warm blobs in a 24x32 grid, hottest first.

    Each is {'peak', 'row', 'col', 'pixels'}; row/col are the temperature-
    weighted centroid (floats), which is far steadier than argmax.
    """
    ambient = float(np.median(grid))
    used = np.zeros(grid.shape, dtype=bool)
    blobs = []
    for _ in range(max_blobs):
        masked = np.where(used, -np.inf, grid)
        r0, c0 = np.unravel_index(int(np.argmax(masked)), grid.shape)
        peak = float(grid[r0, c0])
        if peak < ambient + min_contrast:
            break
        # Half-way between ambient and the peak: the blob's edge.
        thresh = max(ambient + 0.5 * min_contrast, ambient + 0.5 * (peak - ambient))
        stack = [(r0, c0)]
        pix = []
        seen = {(r0, c0)}
        while stack:
            r, c = stack.pop()
            if used[r, c] or grid[r, c] < thresh:
                continue
            pix.append((r, c))
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                rr, cc = r + dr, c + dc
                if 0 <= rr < H and 0 <= cc < W and (rr, cc) not in seen:
                    seen.add((rr, cc))
                    stack.append((rr, cc))
        if not pix:
            used[r0, c0] = True
            continue
        rows = np.array([p[0] for p in pix], dtype=float)
        cols = np.array([p[1] for p in pix], dtype=float)
        wts = np.array([grid[p] - thresh for p in pix], dtype=float) + 0.05
        blobs.append({
            'peak': peak,
            'row': float(np.sum(rows * wts) / np.sum(wts)),
            'col': float(np.sum(cols * wts) / np.sum(wts)),
            'pixels': len(pix),
        })
        # Blank the blob and a one-pixel ring so its shoulder is not a new blob.
        for r, c in pix:
            used[max(0, r - 1):r + 2, max(0, c - 1):c + 2] = True
    return blobs, ambient


class ThermalDrop(OffboardSequence):

    BENCH = "BENCH"
    SURVEY = "SURVEY"
    APPROACH = "APPROACH"
    DESCEND = "DESCEND"
    HOVER = "HOVER"
    DROP_STAGES = (SURVEY, APPROACH, DESCEND, HOVER)

    MIN_DROP_ALTITUDE = 0.50    # m. Hard floor, not a parameter.
    MAX_SURVEY_ALTITUDE = 1.60  # m. Above this the MLX readings are not usable.

    def __init__(self):
        super().__init__('thermal_drop')
        num = self._declare_number

        self.MODE = str(self.declare_parameter('mode', 'fly').value).strip().lower()
        if self.MODE not in ('bench', 'fly'):
            self.get_logger().error(f"Unknown mode '{self.MODE}'; using bench.")
            self.MODE = 'bench'

        # ---- the sensor ----
        self.HFOV = math.radians(float(num('hfov_deg', 110.0)))
        self.VFOV = math.radians(float(num('vfov_deg', 75.0)))
        self.CAM_YAW = math.radians(float(num('cam_yaw_deg', 0.0)))
        self.FLIP_LR = bool(self.declare_parameter('flip_lr', False).value)
        self.FLIP_UD = bool(self.declare_parameter('flip_ud', False).value)
        self.MIN_CONTRAST = float(num('min_contrast', 3.0))
        self.FRAME_LATENCY = float(num('frame_latency', 0.06))
        self.MAX_RAY_ANGLE = math.radians(float(num('max_ray_angle_deg', 60.0)))

        # ---- the offsets ----
        cam_f = float(num('cam_from_drop_forward', 0.0))
        cam_r = float(num('cam_from_drop_right', 0.0))
        self.drop_from_cog = np.array([float(num('drop_from_cog_forward', 0.0)),
                                       float(num('drop_from_cog_right', 0.0)), 0.0])
        self.cam_from_cog = self.drop_from_cog + np.array([cam_f, cam_r, 0.0])

        # ---- the arena ----
        self.BOX_HEIGHT = float(num('box_height', 0.0))
        self.EXPECTED_BOXES = int(num('expected_boxes', 3))
        self.SEARCH_RADIUS = float(num('search_radius', 1.5))
        self.CLUSTER_RADIUS = float(num('cluster_radius', 0.30))
        self.MIN_CLUSTER_FRAMES = int(num('min_cluster_frames', 3))
        self.MIN_HOT_MARGIN = float(num('min_hot_margin', 2.0))

        # ---- the flight ----
        survey_alt = float(num('survey_altitude', 1.5))
        if survey_alt > self.MAX_SURVEY_ALTITUDE:
            self.get_logger().error(
                f"survey_altitude {survey_alt:.2f} m is above the "
                f"{self.MAX_SURVEY_ALTITUDE:.2f} m the MLX is usable from; clamping.")
            survey_alt = self.MAX_SURVEY_ALTITUDE
        self.SURVEY_ALTITUDE = survey_alt
        self.TAKEOFF_ALTITUDE = survey_alt
        self.commanded_altitude = survey_alt

        drop_alt = float(num('drop_altitude', self.MIN_DROP_ALTITUDE))
        if drop_alt < self.MIN_DROP_ALTITUDE:
            self.get_logger().error(
                f"drop_altitude {drop_alt:.2f} m is below the "
                f"{self.MIN_DROP_ALTITUDE:.2f} m floor; using the floor.")
            drop_alt = self.MIN_DROP_ALTITUDE
        self.DROP_ALTITUDE = drop_alt

        self.SURVEY_DWELL = float(num('survey_dwell_seconds', 4.0))
        self.SURVEY_STEP = float(num('survey_step', 0.5))
        self.SURVEY_ALL_POINTS = bool(self.declare_parameter('survey_all_points', False).value)
        self.SURVEY_MOVE_TIMEOUT = 15.0

        self.APPROACH_TOLERANCE = float(num('approach_tolerance', 0.15))
        self.DESCEND_TOLERANCE = float(num('descend_tolerance', 0.12))
        self.DESCEND_SPEED = float(num('descend_speed', 0.12))
        self.ALIGN_TOLERANCE = float(num('align_tolerance', 0.08))
        self.ALIGN_SETTLE = float(num('align_settle_seconds', 2.0))
        self.HOVER_SECONDS = float(num('hover_seconds', 10.0))
        self.LAND_AFTER_HOVER = bool(self.declare_parameter('land_after_hover', True).value)
        self.STAGE_TIMEOUT = float(num('stage_timeout', 45.0))

        self.TRACK_GATE = float(num('track_gate', 0.40))
        self.TRACK_WINDOW = float(num('track_window', 1.5))
        self.TRACK_LOST_SECONDS = float(num('track_lost_seconds', 3.0))
        self.FLIGHT_SECONDS = float(num('flight_seconds', 150.0))

        # ---- state shared between the frame callback and the timer ----
        self._lock = threading.Lock()
        self.pose_buffer = collections.deque(maxlen=200)   # (wall t, q, xyz)
        self.attitude_q = None
        self.clusters = []          # survey map
        self.collecting = False
        self.track = collections.deque(maxlen=40)          # (monotonic t, xy, peak)
        self.chosen = None          # {'xy', 'score'}
        self.frames_seen = 0
        self.last_blobs = []
        self.last_ambient = None

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE,
                                history=HistoryPolicy.KEEP_LAST, depth=1)
        for topic in ('/fmu/out/vehicle_attitude', '/fmu/out/vehicle_attitude_v1'):
            self.create_subscription(VehicleAttitude, topic, self.attitude_callback,
                                     sensor_qos, callback_group=self.sensor_cbg)
        self.create_subscription(Image, str(self.declare_parameter(
            'image_topic', 'thermal/image').value), self.image_callback, 5,
            callback_group=self.sensor_cbg)

        self.ready_pub = self.create_publisher(Bool, 'thermal_drop/ready', 10)
        self.target_pub = self.create_publisher(String, 'thermal_drop/target', 10)

        self.flight_start = None
        self.survey_points = []
        self.survey_index = 0
        self.survey_arrived_since = None
        self.survey_centre = None
        self.desired_alt = None
        self.in_band_since = None
        self.track_lost_since = None
        self.drop_ready = False
        self.outcome = 'not attempted'
        self._xy_counter = None

        if self.MODE == 'bench':
            self.stream_setpoints = False
            self.current_stage = self.BENCH
            self.get_logger().warning(
                "BENCH MODE: nothing is armed or published to PX4. Hold a hot "
                "object under the camera and move it to the drone's RIGHT and "
                "FORWARD -- the log must say RIGHT and FORWARD.")
            return

        self.get_logger().warning(
            f"Thermal drop: climb {self.SURVEY_ALTITUDE:.2f} m, survey for "
            f"{self.EXPECTED_BOXES} boxes, fly the drop point over the hottest, "
            f"descend to {self.DROP_ALTITUDE:.2f} m, align to "
            f"{self.ALIGN_TOLERANCE * 100:.0f} cm, hover {self.HOVER_SECONDS:.0f} s, "
            f"then {'land' if self.LAND_AFTER_HOVER else 'stay up'}. Camera is "
            f"({cam_f:+.2f} fwd, {cam_r:+.2f} right) of the drop point. Hard "
            f"limit {self.FLIGHT_SECONDS:.0f} s. q = descend, k = force-disarm.")

    # ------------------------------------------------------------------ subs

    def attitude_callback(self, msg):
        q = [float(v) for v in msg.q]
        n = math.sqrt(sum(v * v for v in q))
        if n < 1e-6 or not all(math.isfinite(v) for v in q):
            return
        q = [v / n for v in q]
        self.attitude_q = q
        lp = self.local_position
        if lp is not None:
            with self._lock:
                self.pose_buffer.append((time.time(), q, (lp.x, lp.y, lp.z)))

    def local_position_callback(self, msg):
        # Keep our own NED points in the same frame as the base class's hold.
        if self._xy_counter is None:
            self._xy_counter = msg.xy_reset_counter
        elif msg.xy_reset_counter != self._xy_counter:
            self._xy_counter = msg.xy_reset_counter
            d = np.array([msg.delta_xy[0], msg.delta_xy[1]])
            with self._lock:
                for cl in self.clusters:
                    cl['xy'] = cl['xy'] + d
                self.track = collections.deque(
                    ((t, xy + d, p) for t, xy, p in self.track), maxlen=40)
                if self.chosen is not None:
                    self.chosen['xy'] = self.chosen['xy'] + d
                if self.survey_centre is not None:
                    self.survey_centre = self.survey_centre + d
                self.pose_buffer.clear()
        super().local_position_callback(msg)

    def _on_heading_reset(self, delta):
        lp = self.local_position
        if lp is None:
            return
        pivot = np.array([lp.x, lp.y])
        c, s = math.cos(delta), math.sin(delta)
        rot = np.array([[c, -s], [s, c]])

        def turn(xy):
            return pivot + rot @ (xy - pivot)

        with self._lock:
            for cl in self.clusters:
                cl['xy'] = turn(cl['xy'])
            self.track = collections.deque(
                ((t, turn(xy), p) for t, xy, p in self.track), maxlen=40)
            if self.chosen is not None:
                self.chosen['xy'] = turn(self.chosen['xy'])
            self.pose_buffer.clear()

    # --------------------------------------------------------- the geometry

    def pixel_ray_body(self, row, col):
        """Unit-less FRD ray (forward, right, down=1) through a pixel centre."""
        u = (col + 0.5 - W / 2.0) / (W / 2.0) * math.tan(self.HFOV / 2.0)
        v = (row + 0.5 - H / 2.0) / (H / 2.0) * math.tan(self.VFOV / 2.0)
        if self.FLIP_LR:
            u = -u
        if self.FLIP_UD:
            v = -v
        fwd, right = -v, u      # image top = nose, image right = right
        c, s = math.cos(self.CAM_YAW), math.sin(self.CAM_YAW)
        return np.array([fwd * c - right * s, fwd * s + right * c, 1.0])

    def _pose_at(self, wall_t):
        with self._lock:
            if not self.pose_buffer:
                return None
            best = min(self.pose_buffer, key=lambda p: abs(p[0] - wall_t))
        if abs(best[0] - wall_t) > 0.3:
            return None
        return best[1], np.array(best[2])

    def project(self, blob, q, p_ned):
        """Blob -> (north, east) of the box top in NED, or None."""
        if self.home_z is None:
            return None
        ray = quat_rotate(q, self.pixel_ray_body(blob['row'], blob['col']))
        norm = float(np.linalg.norm(ray))
        if ray[2] / norm < math.cos(self.MAX_RAY_ANGLE):
            return None     # too grazing to intersect reliably
        cam = p_ned + quat_rotate(q, self.cam_from_cog)
        above_box = (self.home_z - cam[2]) - self.BOX_HEIGHT
        if above_box < 0.05:
            return None
        t = above_box / ray[2]
        return np.array([cam[0] + t * ray[0], cam[1] + t * ray[1]])

    def vehicle_target_for(self, box_xy):
        """Where the FC position must be for the DROP POINT to be over box_xy."""
        lp = self.local_position
        heading = lp.heading if lp is not None else self.yaw_setpoint
        c, s = math.cos(heading), math.sin(heading)
        f, r = self.drop_from_cog[0], self.drop_from_cog[1]
        return box_xy - np.array([f * c - r * s, f * s + r * c])

    # ------------------------------------------------------------ the frames

    def image_callback(self, msg):
        if msg.encoding != '32FC1' or msg.height != H or msg.width != W:
            self.get_logger().warning(
                f"thermal image is {msg.width}x{msg.height} {msg.encoding}, "
                f"expected {W}x{H} 32FC1.", throttle_duration_sec=5.0)
            return
        grid = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(H, W)
        self.frames_seen += 1
        blobs, ambient = find_blobs(grid.astype(float), self.MIN_CONTRAST)
        self.last_blobs, self.last_ambient = blobs, ambient

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pose = self._pose_at(stamp - self.FRAME_LATENCY)
        if pose is None or not blobs:
            return
        q, p_ned = pose
        lp = self.local_position
        if lp is None or not lp.xy_valid or not lp.z_valid:
            return

        fixes = []
        for b in blobs:
            xy = self.project(b, q, p_ned)
            if xy is not None:
                fixes.append((xy, b))
        now = time.monotonic()

        with self._lock:
            if self.collecting:
                speed = math.hypot(lp.vx, lp.vy)
                if speed < 0.15:
                    for xy, b in fixes:
                        self._add_to_map(xy, b['peak'])
            if self.chosen is not None:
                ref = self._track_estimate_locked(now)
                if ref is None:
                    ref = self.chosen['xy']
                near = [(xy, b) for xy, b in fixes
                        if np.linalg.norm(xy - ref) <= self.TRACK_GATE]
                if near:
                    xy, b = max(near, key=lambda f: f[1]['peak'])
                    self.track.append((now, xy, b['peak']))

    def _add_to_map(self, xy, peak):
        if (self.survey_centre is not None
                and np.linalg.norm(xy - self.survey_centre) > self.SEARCH_RADIUS):
            return
        for cl in self.clusters:
            if np.linalg.norm(cl['xy'] - xy) <= self.CLUSTER_RADIUS:
                n = len(cl['peaks'])
                cl['xy'] = (cl['xy'] * n + xy) / (n + 1)
                cl['peaks'].append(peak)
                return
        self.clusters.append({'xy': xy.copy(), 'peaks': [peak]})

    def _real_clusters(self):
        with self._lock:
            out = [{'xy': cl['xy'].copy(),
                    'score': float(np.percentile(cl['peaks'], 90)),
                    'frames': len(cl['peaks'])}
                   for cl in self.clusters if len(cl['peaks']) >= self.MIN_CLUSTER_FRAMES]
        return sorted(out, key=lambda c: -c['score'])

    def _track_estimate_locked(self, now):
        fresh = [xy for t, xy, _ in self.track if now - t <= self.TRACK_WINDOW]
        if not fresh:
            return None
        return np.median(np.array(fresh), axis=0)

    def track_estimate(self):
        with self._lock:
            return self._track_estimate_locked(time.monotonic())

    # ------------------------------------------------------------ the clock

    def _check_flight_clock(self):
        if self.flight_start is None:
            return False
        if self.current_stage not in (self.TAKEOFF, self.HOLD) + self.DROP_STAGES:
            return False
        if time.monotonic() - self.flight_start < self.FLIGHT_SECONDS:
            return False
        self._finish(f"{self.FLIGHT_SECONDS:.0f} s airborne", land=True)
        return True

    # -------------------------------------------------------- state machine

    def timer_callback(self):
        if self.MODE == 'bench':
            self._handle_bench()
            return
        if self._check_flight_clock():
            return
        if self.current_stage not in self.DROP_STAGES:
            super().timer_callback()
            return

        if self.stream_setpoints:
            self.publish_offboard_control_mode()
            self.publish_position_setpoint()
        self.publish_status()
        self.ready_pub.publish(Bool(data=self.drop_ready))

        if self.kill_requested:
            self._enter_stage(self.KILLING)
            return
        if self.abort_requested:
            self.abort_requested = False
            self._finish("operator abort", land=True)
            return
        if not self._still_flyable():
            return
        self._try_latch_xy_hold()
        self.log_flight_state()

        {self.SURVEY: self._handle_survey,
         self.APPROACH: self._handle_approach,
         self.DESCEND: self._handle_descend,
         self.HOVER: self._handle_hover}[self.current_stage]()

    def _handle_takeoff(self):
        if self.flight_start is None:
            self.flight_start = time.monotonic()
        super()._handle_takeoff()

    def _handle_hold(self):
        if not self._still_flyable():
            return
        self._try_latch_xy_hold()
        if self._in_stage_for() < self.HOLD_SECONDS or not self.hold_xy:
            self.get_logger().info(
                "Holding before the survey"
                + ('' if self.hold_xy else ' (waiting for flow x/y latch)') + "...",
                throttle_duration_sec=1.0)
            return
        lp = self.local_position
        self.survey_centre = np.array([self.hold_x, self.hold_y])
        c, s = math.cos(self.home_yaw), math.sin(self.home_yaw)
        d = self.SURVEY_STEP
        offsets = [(0.0, 0.0), (d, 0.0), (0.0, d), (-d, 0.0), (0.0, -d)]
        self.survey_points = [self.survey_centre + np.array([f * c - r * s, f * s + r * c])
                              for f, r in offsets]
        self.survey_index = 0
        self.survey_arrived_since = None
        with self._lock:
            self.clusters = []
            self.collecting = True
        self._enter_stage(self.SURVEY)
        self.get_logger().warning(
            f"SURVEY from ({lp.x:+.2f}, {lp.y:+.2f}) at "
            f"{self.commanded_altitude:.2f} m. {self.frames_seen} thermal frames "
            "received so far.")

    # --------------------------------------------------------------- SURVEY

    def _handle_survey(self):
        lp = self.local_position
        point = self.survey_points[self.survey_index]
        self._move_to(point)
        now = time.monotonic()
        dist = math.hypot(point[0] - lp.x, point[1] - lp.y)

        if dist > 2.0 * self.APPROACH_TOLERANCE:
            self.survey_arrived_since = None
            if self._in_stage_for() > self.SURVEY_MOVE_TIMEOUT:
                self.get_logger().warning(
                    f"SURVEY: point {self.survey_index} not reached, skipping.")
                self._next_survey_point()
            return
        if self.survey_arrived_since is None:
            self.survey_arrived_since = now
        if now - self.survey_arrived_since < self.SURVEY_DWELL:
            self.get_logger().info(
                f"SURVEY point {self.survey_index + 1}/{len(self.survey_points)}: "
                f"{self._clusters_summary()}", throttle_duration_sec=1.0)
            return

        found = self._real_clusters()
        if (len(found) >= self.EXPECTED_BOXES and not self.SURVEY_ALL_POINTS) \
                or self.survey_index + 1 >= len(self.survey_points):
            self._decide(found)
            return
        self.get_logger().warning(
            f"SURVEY: {len(found)}/{self.EXPECTED_BOXES} boxes after point "
            f"{self.survey_index + 1}; moving {self.SURVEY_STEP:.2f} m to look again.")
        self._next_survey_point()

    def _next_survey_point(self):
        self.survey_index += 1
        self.survey_arrived_since = None
        self._restart_stage_clock()
        if self.survey_index >= len(self.survey_points):
            self._decide(self._real_clusters())

    def _decide(self, found):
        with self._lock:
            self.collecting = False
        if not found:
            self._finish("no warm box found in the survey", land=True)
            return
        for i, cl in enumerate(found):
            self.get_logger().warning(
                f"  box {i + 1}: ({cl['xy'][0]:+.2f}, {cl['xy'][1]:+.2f}) NED, "
                f"{cl['score']:.1f} C, {cl['frames']} frames")
        if len(found) < self.EXPECTED_BOXES:
            self.get_logger().warning(
                f"Only {len(found)} of {self.EXPECTED_BOXES} boxes seen; "
                "taking the hottest of those.")
        best = found[0]
        if len(found) > 1 and best['score'] - found[1]['score'] < self.MIN_HOT_MARGIN:
            self.get_logger().warning(
                f"The hottest box leads by only {best['score'] - found[1]['score']:.1f} C "
                f"(want {self.MIN_HOT_MARGIN:.1f}). Taking it anyway.")
        with self._lock:
            self.chosen = {'xy': best['xy'].copy(), 'score': best['score']}
            self.track.clear()
        self.desired_alt = self.SURVEY_ALTITUDE
        self.in_band_since = None
        self.track_lost_since = None
        self._enter_stage(self.APPROACH)
        self.get_logger().warning(
            f"APPROACH: hottest box at ({best['xy'][0]:+.2f}, {best['xy'][1]:+.2f}) "
            f"NED, {best['score']:.1f} C.")

    # ------------------------------------------------------ tracking helpers

    def _box_xy(self):
        """Live tracked estimate; falls back to the survey position."""
        est = self.track_estimate()
        now = time.monotonic()
        if est is not None:
            self.track_lost_since = None
            with self._lock:
                self.chosen['xy'] = est
            return est, True
        if self.track_lost_since is None:
            self.track_lost_since = now
        with self._lock:
            return self.chosen['xy'].copy(), False

    def _move_to(self, xy):
        self.move_target_x = float(xy[0])
        self.move_target_y = float(xy[1])
        self.moving = True

    def _set_altitude(self, alt):
        alt = min(max(alt, self.DROP_ALTITUDE), self.MAX_SURVEY_ALTITUDE)
        self.commanded_altitude = alt
        self.target_z = self.home_z - alt

    def _horizontal_error(self, target):
        lp = self.local_position
        return math.hypot(target[0] - lp.x, target[1] - lp.y)

    def _lost_for(self):
        if self.track_lost_since is None:
            return 0.0
        return time.monotonic() - self.track_lost_since

    def _publish_target(self, box, err):
        self.target_pub.publish(String(
            data=f"{box[0]:.3f}|{box[1]:.3f}|{err:.3f}|{self.current_stage}"))

    # ------------------------------------------------------------- APPROACH

    def _handle_approach(self):
        box, live = self._box_xy()
        target = self.vehicle_target_for(box)
        self._move_to(target)
        self._set_altitude(self.SURVEY_ALTITUDE)
        err = self._horizontal_error(target)
        self._publish_target(box, err)

        if err <= self.APPROACH_TOLERANCE and live:
            if self.in_band_since is None:
                self.in_band_since = time.monotonic()
            elif time.monotonic() - self.in_band_since >= 1.0:
                self.desired_alt = self.relative_altitude()
                self.in_band_since = None
                self._enter_stage(self.DESCEND)
                self.get_logger().warning(
                    f"DESCEND: over the box ({err * 100:.0f} cm), stepping down "
                    f"to {self.DROP_ALTITUDE:.2f} m.")
                return
        else:
            self.in_band_since = None

        if self._in_stage_for() > self.STAGE_TIMEOUT:
            self._finish("could not settle over the box at survey altitude", land=True)
            return
        self.get_logger().info(
            f"APPROACH: {err:.2f} m to go, track {'LIVE' if live else 'lost'}.",
            throttle_duration_sec=1.0)

    # -------------------------------------------------------------- DESCEND

    def _handle_descend(self):
        box, live = self._box_xy()
        target = self.vehicle_target_for(box)
        self._move_to(target)
        err = self._horizontal_error(target)
        self._publish_target(box, err)
        alt = self.relative_altitude()
        if alt is None:
            return
        if self.desired_alt is None:
            self.desired_alt = alt

        if self._lost_for() > self.TRACK_LOST_SECONDS:
            self.get_logger().warning(
                "DESCEND: lost the box; climbing back to survey altitude to reacquire.")
            with self._lock:
                self.track.clear()
            self.track_lost_since = None
            self._enter_stage(self.APPROACH)
            return

        # Only go down while centred. Off-centre, freeze the altitude where the
        # setpoint currently is -- lower means a smaller footprint to lose it in.
        if live and err <= self.DESCEND_TOLERANCE:
            self.desired_alt = max(self.DROP_ALTITUDE,
                                   self.desired_alt - self.DESCEND_SPEED * 0.05)
        else:
            self.desired_alt = min(self.desired_alt, alt + 0.05)
        self._set_altitude(self.desired_alt)

        if (self.desired_alt <= self.DROP_ALTITUDE + 1e-3
                and abs(alt - self.DROP_ALTITUDE) <= self.ALTITUDE_TOLERANCE):
            self.in_band_since = None
            self._enter_stage(self.HOVER)
            self.get_logger().warning(
                f"HOVER: at {alt:.2f} m, confirming alignment to "
                f"{self.ALIGN_TOLERANCE * 100:.0f} cm.")
            return

        if self._in_stage_for() > self.STAGE_TIMEOUT:
            self._finish("descent onto the box timed out", land=True)
            return
        self.get_logger().info(
            f"DESCEND: alt {alt:.2f} -> {self.desired_alt:.2f} m, err "
            f"{err * 100:.0f} cm, track {'LIVE' if live else 'lost'}.",
            throttle_duration_sec=0.5)

    # ---------------------------------------------------------------- HOVER

    def _handle_hover(self):
        box, live = self._box_xy()
        target = self.vehicle_target_for(box)
        self._move_to(target)
        self._set_altitude(self.DROP_ALTITUDE)
        err = self._horizontal_error(target)
        self._publish_target(box, err)
        now = time.monotonic()

        if not self.drop_ready:
            if self._lost_for() > self.TRACK_LOST_SECONDS:
                self.get_logger().warning("HOVER: lost the box; back to APPROACH.")
                with self._lock:
                    self.track.clear()
                self.track_lost_since = None
                self._enter_stage(self.APPROACH)
                return
            if live and err <= self.ALIGN_TOLERANCE:
                if self.in_band_since is None:
                    self.in_band_since = now
                elif now - self.in_band_since >= self.ALIGN_SETTLE:
                    self.drop_ready = True
                    self._restart_stage_clock()
                    self.outcome = (f"ALIGNED over the hot box to {err * 100:.0f} cm "
                                    f"at {self.relative_altitude():.2f} m")
                    self.get_logger().warning(
                        f"DROP POSITION CONFIRMED: {self.outcome}. Hovering "
                        f"{self.HOVER_SECONDS:.0f} s.")
                    self._release_payload()
            else:
                self.in_band_since = None
                if self._in_stage_for() > self.STAGE_TIMEOUT:
                    self._finish("could not confirm alignment at drop height", land=True)
                    return
            self.get_logger().info(
                f"HOVER: err {err * 100:.0f} cm (want "
                f"{self.ALIGN_TOLERANCE * 100:.0f}), track {'LIVE' if live else 'lost'}.",
                throttle_duration_sec=0.5)
            return

        remaining = self.HOVER_SECONDS - self._in_stage_for()
        if remaining <= 0.0:
            if self.LAND_AFTER_HOVER:
                self._finish("hover over the hot box complete", land=True)
            else:
                self.get_logger().info("Hovering over the box (land_after_hover false).",
                                       throttle_duration_sec=5.0)
            return
        self.get_logger().info(
            f"HOVER over the box: {remaining:.1f} s left, err {err * 100:.0f} cm.",
            throttle_duration_sec=1.0)

    def _release_payload(self):
        """Hook for the drop mechanism. Deliberately does nothing yet."""
        self.get_logger().warning("(drop mechanism not fitted -- release skipped)")

    def _finish(self, reason, land):
        if self.outcome == 'not attempted':
            self.outcome = f"ENDED: {reason}"
        self.drop_ready = False
        with self._lock:
            self.collecting = False
        self.moving = False
        if land:
            self._begin_landing(reason)

    # ---------------------------------------------------------------- bench

    def _handle_bench(self):
        blobs = self.last_blobs
        if self.frames_seen == 0:
            self.get_logger().info("BENCH: no thermal/image yet -- is thermal_sensor up?",
                                   throttle_duration_sec=2.0)
            return
        if not blobs:
            self.get_logger().info(
                f"BENCH: nothing {self.MIN_CONTRAST:.1f} C above ambient "
                f"({self.last_ambient:.1f} C).", throttle_duration_sec=1.0)
            return
        b = blobs[0]
        ray = self.pixel_ray_body(b['row'], b['col'])
        fwd_deg = math.degrees(math.atan(ray[0]))
        right_deg = math.degrees(math.atan(ray[1]))
        lp = self.local_position
        h = (lp.dist_bottom if lp is not None and lp.dist_bottom_valid else 1.0)
        h -= self.BOX_HEIGHT
        box_f = self.cam_from_cog[0] + h * ray[0]
        box_r = self.cam_from_cog[1] + h * ray[1]
        move_f = box_f - self.drop_from_cog[0]
        move_r = box_r - self.drop_from_cog[1]
        self.get_logger().info(
            f"BENCH: hottest {b['peak']:.1f} C (ambient {self.last_ambient:.1f}), "
            f"{len(blobs)} blobs | pixel r{b['row']:.1f} c{b['col']:.1f} = "
            f"{'FORWARD' if fwd_deg >= 0 else 'BACK'} {abs(fwd_deg):.0f} deg, "
            f"{'RIGHT' if right_deg >= 0 else 'LEFT'} {abs(right_deg):.0f} deg | "
            f"from {h:.2f} m the drop point must move "
            f"{'FORWARD' if move_f >= 0 else 'BACK'} {abs(move_f):.2f} m, "
            f"{'RIGHT' if move_r >= 0 else 'LEFT'} {abs(move_r):.2f} m",
            throttle_duration_sec=1.0)

    # --------------------------------------------------------------- status

    def _clusters_summary(self):
        found = self._real_clusters()
        if not found:
            return f"no boxes yet ({self.frames_seen} frames)"
        return ", ".join(f"{c['score']:.1f}C@({c['xy'][0]:+.2f},{c['xy'][1]:+.2f})"
                         for c in found)

    def publish_status(self):
        if self.current_stage not in self.DROP_STAGES:
            super().publish_status()
            return
        alt = self.relative_altitude()
        armed = self.arming_state == VehicleStatus.ARMING_STATE_ARMED
        if self.current_stage == self.SURVEY:
            detail = f"pt{self.survey_index + 1} n{len(self._real_clusters())}"
        elif self.current_stage == self.HOVER:
            detail = 'READY' if self.drop_ready else 'align'
        else:
            detail = f"{self.desired_alt:.2f}" if self.desired_alt else ''
        self.status_pub.publish(String(data="|".join([
            self.current_stage, 'ARM' if armed else 'DIS',
            f"{alt:.2f}" if alt is not None else 'nan',
            'POS' if self.hold_xy else ('FLO' if self.flow_is_healthy() else '---'),
            detail])))

    def destroy_node(self):
        self.get_logger().warning(
            f"Thermal drop outcome: {self.outcome}. {self.frames_seen} thermal "
            f"frames. Survey: {self._clusters_summary()}.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ThermalDrop()
        spin_node(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
