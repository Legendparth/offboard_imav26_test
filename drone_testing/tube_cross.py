"""
Takeoff -> find the tube obstacle -> fit it -> line up on the big gap -> fly
through it -> step left past the back tube -> fly clear -> land. ARK Flow
localisation, ZED as a camera only.

Flown as its own mission, one obstacle at a time, like bar_cross.

    arm -> climb -> hold -> SEARCH (stare ahead for the uprights) -> LOCK
    (stand still, fit the obstacle, choose the gap) -> ALIGN (fly to the
    entry point on the gap centreline at the gap altitude, refining the
    lateral offset while the uprights are still in view) -> PASS (committed,
    blind, through the gap) -> SHIFT (sideways, left by default) -> EXIT
    (straight on, past the back tube) -> CLEAR -> land.

    q -> abort into a controlled descent.   k -> force-disarm.

THE OBSTACLE (front view, as the aircraft approaches; mm)

        L         M         R           uprights 500 apart, 2000 tall
        |\\        |         |
        | \\       |         |           diagonal: top of L (2000) down to
        |  \\      |         |           R at 922, crossing M at about 1461
        |   \\     |         |
        |    \\    |         |
        | GAP \\   |         |           THE GAP: between L and M, above
        |      \\  |         |           the cross tube, below the diagonal
        |       \\ |         |
        |        \\|         |
        |         |\\        |
        |         |  \\      |
        |         |    \\    |
        |         |      \\  |
        |         |        \\|  922
        |=========|=========|  461      cross tube
        |         |         |
       ---------------------------      floor

    plus one free-standing upright about 1 m BEHIND the plane, off to the
    right. The largest hole is the L-M one: 500 mm between centrelines, less
    a tube, so about 450 mm for a 260 mm airframe -- under 10 cm a side.

WHY IT IS MEASURED AND NOT FLOWN BLIND
    Ten centimetres a side is less than ARK Flow drifts over a few metres, and
    less than the error in where the aircraft was put down. So the uprights
    are measured: each is placed on the ground plane in NED, exactly as
    bar_cross places the bar, points are clustered into tubes, and the known
    layout -- three uprights on a line, tube_spacing apart -- is fitted to
    them. The gap is the midpoint of the MEASURED left and middle uprights,
    not a template offset, so spacing errors in the build do not matter.

    Matching three tubes to the template is what decides which tube is the
    middle one. With only two in view the answer is ambiguous (L+M or M+R),
    so min_matched_tubes defaults to 3 and the node waits rather than guess.

    The back upright is 1 m behind the plane; plane_band keeps it out of the
    fit.

    assume_gap_distance > 0 skips all of this and flies the gap from the
    parameters, bar_cross style. With the margins above, only do that if you
    have put the aircraft down on the gap centreline with a tape measure.

THE ALTITUDE
    Solved for the airframe, across its whole width:

        floor  = cross_bar_height + tube_radius + clearance + body_below
        roof   = diagonal height at the inner edge of the airframe
                 - tube_radius - clearance - body_above

    The diagonal slopes, so its lowest point over the airframe is at the edge
    nearest the middle tube. The crossing altitude is the middle of
    [floor, roof]; if floor > roof the gap does not fit this aircraft with
    this clearance and the node refuses at start-up.

    With the launch defaults (cam_z -0.04, clearance 0.12): floor 0.77,
    roof 1.35, crossing at 1.06 m.

THE COMMIT
    Like the window traverse, PASS commits: the target is frozen at the entry
    and the camera stops steering, because the uprights leave the field of
    view in the last metre.

    If flow drops out BEFORE the gap, the aircraft lands where it is -- in
    front of the obstacle, straight down is clear. If it drops out at or past
    the plane, it pushes on open-loop for exactly the distance left to the
    pass exit and lands there: descending inside the gap lands on the cross
    tube. Losing flow during SHIFT or EXIT lands immediately.

THE LIDAR OVER THE CROSS TUBE
    The cross tube passes 0.6 m under the ARK Flow rangefinder. It is thin and
    crossed quickly, but it is the same step change in dist_bottom that
    tripped cs_rng_kin_consistent over the red bar (see course_fsm). During
    the tube stages horizontal hold is judged on flow fusion alone, as
    course_fsm does. If the height estimate itself jumps when crossing, the
    thing to look at is EKF2_RNG_K_GATE / EKF2_HGT_REF, not this node.
"""

import collections
import math
import threading
import time

import numpy as np

import rclpy
from px4_msgs.msg import TrajectorySetpoint, VehicleAttitude, VehicleStatus
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Bool, Float32MultiArray, String

from drone_testing.offboard_sequence import OffboardSequence, spin_node, wrap_pi
from drone_testing.tube_detect import STRIDE
from drone_testing.window_traverse import quat_rotate, rpy_to_matrix_frd


class TubeEstimator:
    """Camera-frame upright measurements -> clustered ground points in NED.

    Free of ROS and the flight node, like BarEstimator, so the rejection and
    clustering logic can be exercised on the bench.
    """

    def __init__(self, depth_min, depth_max, tube_radius, min_top_height,
                 max_bottom_height, buffer_seconds, buffer_max, cluster_radius,
                 min_samples):
        self.depth_min = depth_min
        self.depth_max = depth_max
        self.tube_radius = tube_radius
        self.min_top_height = min_top_height
        self.max_bottom_height = max_bottom_height
        self.buffer_seconds = buffer_seconds
        self.cluster_radius = cluster_radius
        self.min_samples = min_samples

        self._lock = threading.RLock()
        self.samples = collections.deque(maxlen=buffer_max)
        self.rejections = {}
        self.accepted_total = 0

    def add(self, row, q_att, p_ned, r_cam, t_cam, home_z, now):
        with self._lock:
            return self._add(row, q_att, p_ned, r_cam, t_cam, home_z, now)

    def _to_ned(self, depth, az_deg, el_deg, q_att, p_ned, r_cam, t_cam):
        az = math.radians(float(az_deg))
        el = math.radians(float(el_deg))
        cam = np.array([depth, depth * math.tan(az), -depth * math.tan(el)])
        body = r_cam @ cam + t_cam
        return np.asarray(quat_rotate(q_att, body)) + p_ned

    def _add(self, row, q_att, p_ned, r_cam, t_cam, home_z, now):
        depth, az, el, el_top, el_bot, trunc_top, trunc_bot, _ = row
        if not np.isfinite(depth) or not (self.depth_min <= depth <= self.depth_max):
            return self._reject('depth out of range')

        point = self._to_ned(depth, az, el, q_att, p_ned, r_cam, t_cam)
        top = self._to_ned(depth, az, el_top, q_att, p_ned, r_cam, t_cam)
        bottom = self._to_ned(depth, az, el_bot, q_att, p_ned, r_cam, t_cam)

        # Heights above the arming plane, positive up. An upright stands on
        # the floor and reaches up past the aircraft; a cut-off end only
        # tells us it goes at least that far.
        top_h = home_z - top[2]
        bottom_h = home_z - bottom[2]
        if top_h < self.min_top_height and trunc_top < 0.5:
            return self._reject('too short to be an upright')
        if bottom_h > self.max_bottom_height and trunc_bot < 0.5:
            return self._reject('does not reach the floor')

        # Depth is to the near SURFACE; the centreline is one radius further
        # along the horizontal ray.
        ray = point[:2] - p_ned[:2]
        norm = float(np.linalg.norm(ray))
        if norm < 1e-6:
            return self._reject('degenerate ray')
        xy = point[:2] + ray / norm * self.tube_radius

        self.accepted_total += 1
        self.samples.append({'t': now, 'xy': xy})
        return True, ''

    def _reject(self, reason):
        self.rejections[reason] = self.rejections.get(reason, 0) + 1
        return False, reason

    def clusters(self, now):
        """[(xy, count)] for every upright seen often enough, most-seen first."""
        with self._lock:
            cutoff = now - self.buffer_seconds
            pts = [s['xy'] for s in self.samples if s['t'] >= cutoff]
        groups = []
        for p in pts:
            for g in groups:
                if float(np.linalg.norm(p - g['mean'])) <= self.cluster_radius:
                    g['pts'].append(p)
                    g['mean'] = np.mean(g['pts'], axis=0)
                    break
            else:
                groups.append({'pts': [p], 'mean': p.copy()})
        out = [(np.median(np.array(g['pts']), axis=0), len(g['pts']))
               for g in groups if len(g['pts']) >= self.min_samples]
        out.sort(key=lambda c: -c[1])
        return out

    def rotate(self, pivot, delta):
        c, s = math.cos(delta), math.sin(delta)
        with self._lock:
            for sample in self.samples:
                dx = sample['xy'][0] - pivot[0]
                dy = sample['xy'][1] - pivot[1]
                sample['xy'] = np.array([pivot[0] + c * dx - s * dy,
                                         pivot[1] + s * dx + c * dy])

    def rejection_summary(self, limit=3):
        with self._lock:
            if not self.rejections:
                return 'none'
            worst = sorted(self.rejections.items(), key=lambda kv: -kv[1])[:limit]
        return ', '.join(f"{name} x{count}" for name, count in worst)


def solve_gap(clusters, vehicle_xy, heading, spacing, plane_band, match_tol,
              min_matched, max_plane_yaw, gap_side):
    """Fit the three-upright layout to tube clusters and return the gap.

    Returns (solution, reason). solution is a dict with 'point' (gap centre,
    NED xy on the tube plane), 'normal' (unit, pointing through the obstacle),
    'left' (unit, the aircraft's left when flying along normal), 'heading',
    'width' (measured between the gap's two uprights), 'matched'.
    """
    v = np.asarray(vehicle_xy, dtype=float)
    fwd = np.array([math.cos(heading), math.sin(heading)])
    left = np.array([math.sin(heading), -math.cos(heading)])

    ahead = []
    for xy, count in clusters:
        rel = xy - v
        along = float(np.dot(rel, fwd))
        if along > 0.3:
            ahead.append((xy, along))
    if not ahead:
        return None, 'no uprights ahead'

    nearest = min(a for _, a in ahead)
    front = np.array([xy for xy, a in ahead if a <= nearest + plane_band])

    # The line through the front uprights. With one tube there is no line,
    # and the layout match below will refuse it anyway.
    u = left.copy()
    if len(front) >= 2:
        centred = front - front.mean(axis=0)
        _, _, vt = np.linalg.svd(centred, full_matrices=False)
        u = vt[0] / np.linalg.norm(vt[0])
        if float(np.dot(u, left)) < 0.0:
            u = -u
        if math.acos(min(1.0, abs(float(np.dot(u, left))))) > max_plane_yaw:
            return None, 'tube plane is too far off square to the heading'
    normal = np.array([-u[1], u[0]])
    if float(np.dot(normal, fwd)) < 0.0:
        normal = -normal

    lat = [float(np.dot(p - v, u)) for p in front]
    slots = (1, 0, -1)        # left, middle, right: + is LEFT

    best = None
    for li in lat:
        for k in slots:
            m = li - k * spacing
            used = {}
            residual = 0.0
            for lj in lat:
                errs = [(abs(lj - (m + s * spacing)), s) for s in slots]
                err, s = min(errs)
                if err <= match_tol and (s not in used or err < used[s][0]):
                    used[s] = (err, lj)
            residual = sum(e for e, _ in used.values())
            key = (len(used), -residual)
            if best is None or key > best[0]:
                best = (key, m, used)
            elif key[0] == best[0][0] and abs(m - best[1]) > 0.5 * spacing:
                # Equally good fit with a different middle tube: ambiguous,
                # unless something later beats both.
                best = (key, m, used, 'ambiguous')

    if best is None:
        return None, 'no layout fit'
    if len(best) == 4:
        return None, 'two layouts fit equally well (which tube is the middle?)'
    _, m, used = best
    if len(used) < min_matched:
        return None, f'only {len(used)}/{min_matched} uprights fit the layout'

    pair = (1, 0) if gap_side == 'left' else (0, -1)
    if pair[0] not in used or pair[1] not in used:
        return None, 'the uprights either side of the gap are not both matched'
    l_a, l_b = used[pair[0]][1], used[pair[1]][1]

    along = float(np.mean([np.dot(p - v, normal) for p in front]))
    point = v + u * (0.5 * (l_a + l_b)) + normal * along
    return {
        'point': point,
        'normal': normal,
        'left': u,
        'heading': math.atan2(normal[1], normal[0]),
        'width': abs(l_a - l_b),
        'middle_lateral': m,
        'matched': len(used),
        'residual': sum(e for e, _ in used.values()),
    }, ''


class TubeCross(OffboardSequence):

    SEARCH = "SEARCH"
    LOCK = "LOCK"
    ALIGN = "ALIGN"
    PASS = "PASS"
    SHIFT = "SHIFT"
    EXIT = "EXIT"
    CLEAR = "CLEAR"

    TUBE_STAGES = (SEARCH, LOCK, ALIGN, PASS, SHIFT, EXIT, CLEAR)
    FLOW_ONLY_STAGES = (PASS, SHIFT, EXIT, CLEAR)

    # ---- the obstacle (rules drawing, metres) -----------------------------
    TUBE_SPACING = 0.50
    TUBE_RADIUS = 0.025
    CROSS_BAR_HEIGHT = 0.461
    DIAGONAL_LEFT_HEIGHT = 2.000    # where the diagonal meets the LEFT upright
    DIAGONAL_RIGHT_HEIGHT = 0.922   # ... and the RIGHT one
    GAP_SIDE = 'left'               # left = between L and M (the big one)

    # ---- the path ---------------------------------------------------------
    STANDOFF_DISTANCE = 1.20    # m before the plane the pass starts from
    PASS_EXIT_DISTANCE = 0.50   # m past the plane before stepping sideways
    SHIFT_LEFT = 0.40           # m, + = left. Clears the back upright.
    EXIT_DISTANCE = 1.20        # m on from the shift point; the back upright
                                # is 1 m behind the plane
    CLEARANCE = 0.12            # m wanted between airframe and tube, vertically

    # ---- the airframe (same numbers as window_traverse / bar_cross) -------
    GEAR_BELOW_CAMERA = 0.120
    DRONE_HEIGHT = 0.260
    DRONE_WIDTH = 0.260

    APPROACH_SPEED = 0.30
    PASS_SPEED = 0.30
    SHIFT_SPEED = 0.25

    ALIGN_CROSS_TOLERANCE = 0.05    # m off the gap centreline. Tight on
                                    # purpose: there is under 10 cm a side.
    ALIGN_ALONG_TOLERANCE = 0.15
    ALIGN_YAW_TOLERANCE = math.radians(5.0)
    ALT_TOLERANCE = 0.06
    SETTLE_SECONDS = 1.5
    ARRIVE_TOLERANCE = 0.10
    ARRIVE_SETTLE_SECONDS = 0.5
    REFINE_MIN_DISTANCE = 1.00      # m to the plane below which the camera
                                    # stops steering the entry point
    REFINE_MAX_JUMP = 0.20          # m a refinement may move the gap by
    MIN_GAP_WIDTH = 0.40
    MAX_GAP_WIDTH = 0.60

    SEARCH_TIMEOUT = 45.0
    LOCK_SECONDS = 2.0
    LOCK_TIMEOUT = 20.0
    ALIGN_TIMEOUT = 40.0
    PASS_TIMEOUT = 20.0
    MOVE_STAGE_TIMEOUT = 20.0
    CLEAR_SECONDS = 2.0
    SOLUTION_LOST_TIMEOUT = 6.0

    YAW_CONE_DEG = 30.0

    # ---- the estimate -----------------------------------------------------
    GEOMETRY_TOPIC = 'tube_geometry'
    DETECT_TOPIC = 'tubes_detected'
    ATTITUDE_MAX_HZ = 30.0
    DEPTH_MIN = 0.40
    DEPTH_MAX = 6.00
    MIN_TOP_HEIGHT = 1.20
    MAX_BOTTOM_HEIGHT = 0.40
    BUFFER_SECONDS = 2.5
    BUFFER_MAX = 400
    CLUSTER_RADIUS = 0.15
    MIN_SAMPLES = 6
    PLANE_BAND = 0.40
    MATCH_TOLERANCE = 0.12
    MIN_MATCHED_TUBES = 3
    MAX_PLANE_YAW_DEG = 30.0

    TAKEOFF_ALTITUDE = 1.00
    MAX_ALTITUDE = 2.00
    FLIGHT_SECONDS = 150.0

    def __init__(self, node_name='tube_cross'):
        super().__init__(node_name)
        n = self._declare_number

        self.TUBE_SPACING = float(n('tube_spacing', self.TUBE_SPACING))
        self.TUBE_RADIUS = float(n('tube_radius', self.TUBE_RADIUS))
        self.CROSS_BAR_HEIGHT = float(n('cross_bar_height', self.CROSS_BAR_HEIGHT))
        self.DIAGONAL_LEFT_HEIGHT = float(n('diagonal_left_height', self.DIAGONAL_LEFT_HEIGHT))
        self.DIAGONAL_RIGHT_HEIGHT = float(n('diagonal_right_height', self.DIAGONAL_RIGHT_HEIGHT))
        side = str(self.declare_parameter('gap_side', self.GAP_SIDE).value).strip().lower()
        self.gap_side = side if side in ('left', 'right') else self.GAP_SIDE

        self.STANDOFF_DISTANCE = float(n('standoff_distance', self.STANDOFF_DISTANCE))
        self.PASS_EXIT_DISTANCE = float(n('pass_exit_distance', self.PASS_EXIT_DISTANCE))
        self.SHIFT_LEFT = float(n('shift_left', self.SHIFT_LEFT))
        self.EXIT_DISTANCE = float(n('exit_distance', self.EXIT_DISTANCE))
        self.CLEARANCE = float(n('clearance', self.CLEARANCE))
        self.cross_altitude_override = float(n('cross_altitude', 0.0))

        self.GEAR_BELOW_CAMERA = float(n('gear_below_camera', self.GEAR_BELOW_CAMERA))
        self.DRONE_HEIGHT = float(n('drone_height', self.DRONE_HEIGHT))
        self.DRONE_WIDTH = float(n('drone_width', self.DRONE_WIDTH))

        self.APPROACH_SPEED = float(n('approach_speed', self.APPROACH_SPEED))
        self.PASS_SPEED = float(n('pass_speed', self.PASS_SPEED))
        self.SHIFT_SPEED = float(n('shift_speed', self.SHIFT_SPEED))
        self.ALIGN_CROSS_TOLERANCE = float(n('align_cross_tolerance', self.ALIGN_CROSS_TOLERANCE))
        self.ALIGN_ALONG_TOLERANCE = float(n('align_along_tolerance', self.ALIGN_ALONG_TOLERANCE))
        self.ALIGN_YAW_TOLERANCE = math.radians(float(n(
            'align_yaw_tolerance_deg', math.degrees(self.ALIGN_YAW_TOLERANCE))))
        self.ALT_TOLERANCE = float(n('alt_tolerance', self.ALT_TOLERANCE))
        self.SETTLE_SECONDS = float(n('settle_seconds', self.SETTLE_SECONDS))
        self.REFINE_MIN_DISTANCE = float(n('refine_min_distance', self.REFINE_MIN_DISTANCE))
        self.REFINE_MAX_JUMP = float(n('refine_max_jump', self.REFINE_MAX_JUMP))
        self.SEARCH_TIMEOUT = float(n('search_timeout', self.SEARCH_TIMEOUT))
        self.LOCK_SECONDS = float(n('lock_seconds', self.LOCK_SECONDS))
        self.FLIGHT_SECONDS = float(n('flight_seconds', self.FLIGHT_SECONDS))
        self.YAW_CONE = math.radians(float(n('yaw_cone_deg', self.YAW_CONE_DEG)))

        self.PLANE_BAND = float(n('plane_band', self.PLANE_BAND))
        self.MATCH_TOLERANCE = float(n('match_tolerance', self.MATCH_TOLERANCE))
        self.MIN_MATCHED_TUBES = int(n('min_matched_tubes', self.MIN_MATCHED_TUBES))
        self.MAX_PLANE_YAW = math.radians(float(n('max_plane_yaw_deg', self.MAX_PLANE_YAW_DEG)))

        # Blind: gap assumed this far ahead of where the aircraft is at the
        # end of the hold, offset sideways by assume_gap_left.
        self.ASSUME_GAP_DISTANCE = float(n('assume_gap_distance', 0.0))
        self.ASSUME_GAP_LEFT = float(n('assume_gap_left', 0.0))
        self.flying_blind = self.ASSUME_GAP_DISTANCE > 0.0

        cam_x = float(n('cam_x', 0.0))
        cam_y = float(n('cam_y', 0.0))
        cam_z = float(n('cam_z', 0.0))
        self.r_cam = rpy_to_matrix_frd(float(n('cam_roll', 0.0)),
                                       float(n('cam_pitch', 0.0)),
                                       float(n('cam_yaw', 0.0)))
        self.t_cam = np.array([cam_x, -cam_y, -cam_z])
        self.body_below = self.GEAR_BELOW_CAMERA - cam_z
        self.body_above = self.DRONE_HEIGHT - self.body_below

        self.estimator = TubeEstimator(
            depth_min=float(n('depth_min', self.DEPTH_MIN)),
            depth_max=float(n('depth_max', self.DEPTH_MAX)),
            tube_radius=self.TUBE_RADIUS,
            min_top_height=float(n('min_tube_top_height', self.MIN_TOP_HEIGHT)),
            max_bottom_height=float(n('max_tube_bottom_height', self.MAX_BOTTOM_HEIGHT)),
            buffer_seconds=float(n('buffer_seconds', self.BUFFER_SECONDS)),
            buffer_max=self.BUFFER_MAX,
            cluster_radius=float(n('cluster_radius', self.CLUSTER_RADIUS)),
            min_samples=int(n('pose_min_samples', self.MIN_SAMPLES)),
        )

        self.cross_altitude, self.gap_floor, self.gap_roof = self._solve_altitude()
        self.problems = self._check_geometry()
        for problem in self.problems:
            self.get_logger().error(f"TUBES NOT FLYABLE: {problem}")

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE,
                                history=HistoryPolicy.KEEP_LAST, depth=1)
        self.attitude = None
        self.attitude_time = None
        self._attitude_min_interval = 1.0 / self.ATTITUDE_MAX_HZ
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude',
                                 self.attitude_callback, sensor_qos,
                                 callback_group=self.sensor_cbg)

        self.geometry_topic = str(self.declare_parameter(
            'geometry_topic', self.GEOMETRY_TOPIC).value)
        detect_topic = str(self.declare_parameter('detect_topic', self.DETECT_TOPIC).value)
        self.create_subscription(Float32MultiArray, self.geometry_topic,
                                 self.geometry_callback, 10,
                                 callback_group=self.sensor_cbg)
        self.create_subscription(Bool, detect_topic, self.detected_callback, 10,
                                 callback_group=self.sensor_cbg)
        self.gap_pub = self.create_publisher(String, 'tube_gap', 10)

        self.tubes_flag = False
        self.geometry_seen = 0
        self.solution = None
        self.solution_reason = 'no data yet'
        self.solution_ok_since = None
        self.solution_lost_since = None

        self.gap_point = None
        self.gap_normal = None
        self.gap_left = None
        self.gap_heading = None
        self.entry = None
        self.pass_exit = None
        self.shift_point = None
        self.final_point = None
        self.stage_target = None
        self.settle_since = None
        self.push_since = None
        self.push_seconds = 0.0
        self.flight_start = None
        self.outcome = 'not attempted'

        self.get_logger().warning(
            f"Tube crossing on ARK FLOW{', FLYING BLIND' if self.flying_blind else ''}: "
            f"through the {self.gap_side.upper()} gap at {self.cross_altitude:.2f} m "
            f"(fits {self.gap_floor:.2f}-{self.gap_roof:.2f} m), then "
            f"{abs(self.SHIFT_LEFT):.2f} m {'left' if self.SHIFT_LEFT >= 0 else 'right'} "
            f"and {self.EXIT_DISTANCE:.2f} m on. Point the aircraft at the "
            "obstacle before you arm. Press q to abort, k to force-disarm. "
            + ("READY." if not self.problems else "WILL NOT ATTEMPT -- see errors."))

    # ------------------------------------------------------------ geometry

    def _diagonal_height(self, lateral):
        """Diagonal height at `lateral` m LEFT of the middle upright."""
        mid = 0.5 * (self.DIAGONAL_LEFT_HEIGHT + self.DIAGONAL_RIGHT_HEIGHT)
        slope = ((self.DIAGONAL_LEFT_HEIGHT - self.DIAGONAL_RIGHT_HEIGHT)
                 / (2.0 * self.TUBE_SPACING))
        return mid + slope * lateral

    def _solve_altitude(self):
        centre = (0.5 if self.gap_side == 'left' else -0.5) * self.TUBE_SPACING
        inner_edges = (centre - 0.5 * self.DRONE_WIDTH, centre + 0.5 * self.DRONE_WIDTH)
        roof_tube = min(self._diagonal_height(e) for e in inner_edges)
        floor = (self.CROSS_BAR_HEIGHT + self.TUBE_RADIUS + self.CLEARANCE
                 + self.body_below)
        roof = roof_tube - self.TUBE_RADIUS - self.CLEARANCE - self.body_above
        altitude = 0.5 * (floor + roof)
        if self.cross_altitude_override > 0.0:
            altitude = self.cross_altitude_override
        return altitude, floor, roof

    def _check_geometry(self):
        problems = []
        if self.gap_floor > self.gap_roof:
            problems.append(
                f"the gap leaves no altitude band with {self.CLEARANCE:.2f} m "
                f"clearance (floor {self.gap_floor:.2f} > roof {self.gap_roof:.2f})")
        elif not (self.gap_floor <= self.cross_altitude <= self.gap_roof):
            problems.append(
                f"cross_altitude {self.cross_altitude:.2f} m is outside the "
                f"band {self.gap_floor:.2f}-{self.gap_roof:.2f} m")
        side = 0.5 * (self.TUBE_SPACING - 2.0 * self.TUBE_RADIUS - self.DRONE_WIDTH)
        if side < 0.05:
            problems.append(f"only {side:.3f} m a side between airframe and uprights")
        if not (self.MIN_ALTITUDE <= self.cross_altitude <= self.MAX_ALTITUDE):
            problems.append(
                f"cross altitude {self.cross_altitude:.2f} m outside "
                f"[{self.MIN_ALTITUDE:.2f}, {self.MAX_ALTITUDE:.2f}]")
        if self.cross_altitude - self.body_below < self.FLOW_MIN_AGL + 0.2:
            problems.append("cross altitude is too close to the optical-flow floor")
        return problems

    # ------------------------------------------------------------------ subs

    def attitude_callback(self, msg):
        now = time.monotonic()
        if (self.attitude_time is not None
                and now - self.attitude_time < self._attitude_min_interval):
            return
        self.attitude = msg
        self.attitude_time = now

    def detected_callback(self, msg):
        self.tubes_flag = bool(msg.data)

    def geometry_callback(self, msg):
        self.geometry_seen += 1
        data = np.asarray(msg.data, dtype=float)
        if data.size == 0 or data.size % STRIDE != 0:
            return
        lp = self.local_position
        if (lp is None or not lp.xy_valid or not lp.z_valid
                or self.home_z is None or self.attitude is None):
            return
        q = np.asarray(self.attitude.q, dtype=float)
        p = np.array([lp.x, lp.y, lp.z])
        now = time.monotonic()
        for row in data.reshape(-1, STRIDE):
            self.estimator.add(row, q, p, self.r_cam, self.t_cam, self.home_z, now)

    # --------------------------------------------------------------- the gap

    def _update_solution(self):
        lp = self.local_position
        now = time.monotonic()
        if lp is None or self.home_z is None:
            self.solution = None
            return
        heading = self.gap_heading if self.gap_heading is not None else self.home_yaw
        sol, reason = solve_gap(
            self.estimator.clusters(now), (lp.x, lp.y), heading,
            self.TUBE_SPACING, self.PLANE_BAND, self.MATCH_TOLERANCE,
            self.MIN_MATCHED_TUBES, self.MAX_PLANE_YAW, self.gap_side)
        if sol is not None and not (self.MIN_GAP_WIDTH <= sol['width'] <= self.MAX_GAP_WIDTH):
            sol, reason = None, f"measured gap {sol['width']:.2f} m is implausible"
        self.solution = sol
        self.solution_reason = reason
        if sol is not None:
            self.solution_lost_since = None
            if self.solution_ok_since is None:
                self.solution_ok_since = now
        else:
            self.solution_ok_since = None
            if self.solution_lost_since is None:
                self.solution_lost_since = now

    def gap_summary(self):
        if self.solution is None:
            if self.geometry_seen == 0:
                return f"nothing on /{self.geometry_topic} -- is tube_detect running?"
            clusters = self.estimator.clusters(time.monotonic())
            return (f"no gap: {self.solution_reason} ({len(clusters)} tube "
                    f"clusters, {self.estimator.accepted_total} samples accepted; "
                    f"rejections: {self.estimator.rejection_summary()})")
        s = self.solution
        return (f"gap at ({s['point'][0]:+.2f}, {s['point'][1]:+.2f}), "
                f"{s['width']:.2f} m wide, heading {math.degrees(s['heading']):+.0f} deg, "
                f"{s['matched']} uprights matched, residual {s['residual']:.3f} m")

    def publish_gap(self):
        msg = String()
        s = self.solution
        msg.data = '' if s is None else "|".join([
            f"{s['point'][0]:.3f}", f"{s['point'][1]:.3f}",
            f"{math.degrees(s['heading']):.1f}", f"{s['width']:.3f}",
            f"{s['matched']}", f"{s['residual']:.3f}"])
        self.gap_pub.publish(msg)

    def assumed_solution(self):
        lp = self.local_position
        if lp is None:
            return None
        h = self.home_yaw
        fwd = np.array([math.cos(h), math.sin(h)])
        left = np.array([math.sin(h), -math.cos(h)])
        point = (np.array([lp.x, lp.y]) + fwd * self.ASSUME_GAP_DISTANCE
                 + left * self.ASSUME_GAP_LEFT)
        return {'point': point, 'normal': fwd, 'left': left, 'heading': h,
                'width': self.TUBE_SPACING, 'matched': 0, 'residual': 0.0}

    def _freeze_path(self, sol):
        """Every waypoint of the crossing, from one gap solution."""
        self.gap_point = np.array(sol['point'], dtype=float)
        self.gap_normal = np.array(sol['normal'], dtype=float)
        self.gap_left = np.array(sol['left'], dtype=float)
        self.gap_heading = sol['heading']
        self.entry = self.gap_point - self.gap_normal * self.STANDOFF_DISTANCE
        self.pass_exit = self.gap_point + self.gap_normal * self.PASS_EXIT_DISTANCE
        self.shift_point = self.pass_exit + self.gap_left * self.SHIFT_LEFT
        self.final_point = self.shift_point + self.gap_normal * self.EXIT_DISTANCE

    def _path_frame(self, target):
        """(along, cross) from the aircraft to target, in the gap frame."""
        lp = self.local_position
        if lp is None or target is None or self.gap_normal is None:
            return None, None
        e = np.asarray(target) - np.array([lp.x, lp.y])
        return float(np.dot(e, self.gap_normal)), float(np.dot(e, self.gap_left))

    def _distance_past_plane(self):
        lp = self.local_position
        if lp is None or self.gap_point is None:
            return -1e9
        return float(np.dot(np.array([lp.x, lp.y]) - self.gap_point, self.gap_normal))

    # ---------------------------------------------------------------- moves

    def _set_target(self, xy, altitude=None):
        self.stage_target = np.array(xy, dtype=float)
        self.move_target_x = float(xy[0])
        self.move_target_y = float(xy[1])
        self.moving = True
        if altitude is not None:
            self.commanded_altitude = float(altitude)
            self.target_z = self.home_z - float(altitude)

    def _clamp_to_cone(self, heading):
        if self.YAW_CONE <= 0.0 or self.home_z is None:
            return heading
        off = wrap_pi(heading - self.home_yaw)
        if abs(off) <= self.YAW_CONE:
            return heading
        return wrap_pi(self.home_yaw + math.copysign(self.YAW_CONE, off))

    def _aim_yaw_at(self, heading):
        self.yaw_remaining = wrap_pi(self._clamp_to_cone(heading) - self.yaw_setpoint)

    def _heading_error(self, heading):
        lp = self.local_position
        return math.pi if lp is None else abs(wrap_pi(heading - lp.heading))

    def _arrived(self, target):
        """True once within ARRIVE_TOLERANCE for ARRIVE_SETTLE_SECONDS."""
        along, cross = self._path_frame(target)
        if (along is not None and abs(along) <= self.ARRIVE_TOLERANCE
                and abs(cross) <= self.ARRIVE_TOLERANCE):
            now = time.monotonic()
            if self.settle_since is None:
                self.settle_since = now
            return now - self.settle_since >= self.ARRIVE_SETTLE_SECONDS
        self.settle_since = None
        return False

    def _enter_tube_stage(self, stage):
        self._enter_stage(stage)
        self.settle_since = None
        self.push_since = None

    # ---------------------------------------------------------- EKF2 resets

    def _on_heading_reset(self, delta):
        super()._on_heading_reset(delta)
        lp = self.local_position
        if lp is None:
            self.estimator.samples.clear()
            return
        pivot = (lp.x, lp.y)
        self.estimator.rotate(pivot, delta)
        c, s = math.cos(delta), math.sin(delta)

        def turn(p):
            dx, dy = p[0] - pivot[0], p[1] - pivot[1]
            return np.array([pivot[0] + c * dx - s * dy, pivot[1] + s * dx + c * dy])

        def spin(v):
            return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])

        for name in ('gap_point', 'entry', 'pass_exit', 'shift_point',
                     'final_point', 'stage_target'):
            if getattr(self, name, None) is not None:
                setattr(self, name, turn(getattr(self, name)))
        for name in ('gap_normal', 'gap_left'):
            if getattr(self, name, None) is not None:
                setattr(self, name, spin(getattr(self, name)))
        if self.gap_heading is not None:
            self.gap_heading = wrap_pi(self.gap_heading + delta)
        if self.move_target_x is not None:
            self.move_target_x, self.move_target_y = turn(
                (self.move_target_x, self.move_target_y))
            self.move_start_x, self.move_start_y = turn(
                (self.move_start_x, self.move_start_y))

    # ------------------------------------------------------------ health

    def flow_is_healthy(self):
        """Flow fusion alone during and after the pass (see course_fsm)."""
        if self.current_stage not in self.FLOW_ONLY_STAGES:
            return super().flow_is_healthy()
        lp = self.local_position
        f = self.estimator_flags
        if f is None:
            return super().flow_is_healthy()
        return (lp is not None and lp.xy_valid and lp.v_xy_valid
                and lp.dist_bottom > self.FLOW_MIN_AGL
                and f.cs_opt_flow and not f.cs_inertial_dead_reckoning)

    def flight_time(self):
        return 0.0 if self.flight_start is None else time.monotonic() - self.flight_start

    def _check_flight_clock(self):
        """Land at flight_seconds -- never from inside the gap."""
        if self.flight_start is None or self.current_stage == self.PASS:
            return False
        if self.current_stage not in (self.TAKEOFF, self.HOLD) + self.TUBE_STAGES:
            return False
        if self.flight_time() < self.FLIGHT_SECONDS:
            return False
        self._begin_landing(f"{self.FLIGHT_SECONDS:.0f} s airborne")
        return True

    # ------------------------------------------------------- state machine

    def timer_callback(self):
        self._update_solution()
        self.publish_gap()

        if self.current_stage not in self.TUBE_STAGES:
            super().timer_callback()
            return

        if self._check_flight_clock():
            return

        if self.stream_setpoints:
            self.publish_offboard_control_mode()
            self.publish_position_setpoint()
        self.publish_status()

        if self.kill_requested:
            self._enter_stage(self.KILLING)
            return
        if self.abort_requested:
            self.abort_requested = False
            self._begin_landing("operator abort")
            return

        if not self._still_flyable():
            return
        self._try_latch_xy_hold()
        self.log_flight_state()

        {
            self.SEARCH: self._handle_search,
            self.LOCK: self._handle_lock,
            self.ALIGN: self._handle_align,
            self.PASS: self._handle_pass,
            self.SHIFT: self._handle_shift,
            self.EXIT: self._handle_exit,
            self.CLEAR: self._handle_clear,
        }[self.current_stage]()

    def _handle_takeoff(self):
        if self.flight_start is None:
            self.flight_start = time.monotonic()
        super()._handle_takeoff()

    def _handle_hold(self):
        if not self._still_flyable():
            return
        self._try_latch_xy_hold()
        remaining = self.HOLD_SECONDS - self._in_stage_for()
        if remaining > 0.0:
            self.get_logger().info(f"Holding, {remaining:.1f} s...",
                                   throttle_duration_sec=1.0)
            self.log_flight_state()
            return
        if self.problems:
            self.outcome = 'REFUSED: ' + '; '.join(self.problems)
            self._begin_landing("tube geometry not flyable")
            return
        if not self.hold_xy:
            self.get_logger().warning("Waiting for a healthy lateral estimate.",
                                      throttle_duration_sec=2.0)
            return
        if self.flying_blind:
            sol = self.assumed_solution()
            if sol is not None:
                self._begin_align(sol)
            return
        self._enter_tube_stage(self.SEARCH)
        self.get_logger().warning("SEARCH: holding, looking for the uprights.")

    def _handle_search(self):
        if self.solution is not None and self.hold_xy:
            self._enter_tube_stage(self.LOCK)
            self.get_logger().warning(f"LOCK: {self.gap_summary()}.")
            return
        if self._in_stage_for() > self.SEARCH_TIMEOUT:
            self._abandon(f"no gap found in {self.SEARCH_TIMEOUT:.0f} s. {self.gap_summary()}")
            return
        self.get_logger().info(f"SEARCH: {self.gap_summary()}", throttle_duration_sec=1.0)

    def _handle_lock(self):
        if self.solution is None:
            if (self.solution_lost_since is not None and
                    time.monotonic() - self.solution_lost_since > self.SOLUTION_LOST_TIMEOUT):
                self._abandon(f"lost the gap during LOCK. {self.gap_summary()}")
            return
        if self._in_stage_for() > self.LOCK_TIMEOUT:
            self._abandon("the gap estimate never held steady")
            return
        if (self.solution_ok_since is None
                or time.monotonic() - self.solution_ok_since < self.LOCK_SECONDS):
            self.get_logger().info(f"LOCK: settling. {self.gap_summary()}",
                                   throttle_duration_sec=1.0)
            return
        self._begin_align(self.solution)

    def _begin_align(self, sol):
        self._freeze_path(sol)
        self.MOVE_SPEED = self.APPROACH_SPEED
        self._enter_tube_stage(self.ALIGN)
        self._set_target(self.entry, self.cross_altitude)
        self._aim_yaw_at(self.gap_heading)
        self.get_logger().warning(
            f"ALIGN: {'ASSUMED' if sol['matched'] == 0 else 'measured'} gap at "
            f"({self.gap_point[0]:+.2f}, {self.gap_point[1]:+.2f}), heading "
            f"{math.degrees(self.gap_heading):+.0f} deg. Flying to the entry "
            f"{self.STANDOFF_DISTANCE:.2f} m short of it at {self.cross_altitude:.2f} m.")

    def _handle_align(self):
        self._aim_yaw_at(self.gap_heading)

        # Refine while the uprights are still well in view. The heading is
        # kept; only the gap point may move, and not by much.
        if (not self.flying_blind and self.solution is not None
                and -self._distance_past_plane() > self.REFINE_MIN_DISTANCE):
            jump = float(np.linalg.norm(self.solution['point'] - self.gap_point))
            if jump <= self.REFINE_MAX_JUMP:
                sol = dict(self.solution)
                sol['normal'], sol['left'], sol['heading'] = (
                    self.gap_normal, self.gap_left, self.gap_heading)
                self._freeze_path(sol)
                self.move_target_x, self.move_target_y = map(float, self.entry)
                self.stage_target = self.entry.copy()
            else:
                self.get_logger().warning(
                    f"ALIGN: ignoring a {jump:.2f} m jump in the gap estimate.",
                    throttle_duration_sec=2.0)

        if not self.hold_xy:
            self.moving = False
            self.settle_since = None
            if self._in_stage_for() > self.ALIGN_TIMEOUT:
                self._abandon("lateral estimate never recovered before the pass")
            return
        self.moving = True

        along, cross = self._path_frame(self.entry)
        alt = self.relative_altitude()
        ready = (along is not None
                 and abs(along) <= self.ALIGN_ALONG_TOLERANCE
                 and abs(cross) <= self.ALIGN_CROSS_TOLERANCE
                 and alt is not None
                 and abs(alt - self.cross_altitude) <= self.ALT_TOLERANCE
                 and self._heading_error(self.gap_heading) <= self.ALIGN_YAW_TOLERANCE)
        if ready:
            now = time.monotonic()
            if self.settle_since is None:
                self.settle_since = now
            elif now - self.settle_since >= self.SETTLE_SECONDS:
                self._begin_pass()
            return
        self.settle_since = None

        if self._in_stage_for() > self.ALIGN_TIMEOUT:
            self._abandon(f"could not settle on the entry in {self.ALIGN_TIMEOUT:.0f} s")
            return
        self.get_logger().info(
            f"ALIGN: {0.0 if along is None else along:+.2f} along / "
            f"{0.0 if cross is None else cross:+.2f} across (tol "
            f"{self.ALIGN_CROSS_TOLERANCE:.2f}), alt "
            f"{'n/a' if alt is None else f'{alt:.2f}'}/{self.cross_altitude:.2f}, yaw err "
            f"{math.degrees(self._heading_error(self.gap_heading)):.0f} deg. "
            f"{self.gap_summary()}", throttle_duration_sec=1.0)

    def _begin_pass(self):
        self.MOVE_SPEED = self.PASS_SPEED
        self._enter_tube_stage(self.PASS)
        self._set_target(self.pass_exit, self.cross_altitude)
        self.get_logger().warning(
            f"PASS: committed. Through the gap to "
            f"({self.pass_exit[0]:+.2f}, {self.pass_exit[1]:+.2f}) at "
            f"{self.PASS_SPEED:.2f} m/s. The camera is no longer steering.")

    def _handle_pass(self):
        self.yaw_remaining = wrap_pi(self.gap_heading - self.yaw_setpoint)
        now = time.monotonic()

        if not self.hold_xy:
            past = self._distance_past_plane()
            if past < -0.35 and self.push_since is None:
                self._abandon("flow lost before the gap; landing in front of it")
                return
            if self.push_since is None:
                remaining = max(0.0, self.PASS_EXIT_DISTANCE - past)
                self.push_seconds = remaining / max(self.PASS_SPEED, 1e-3)
                self.push_since = now
                self.get_logger().error(
                    f"PASS: flow lost in the gap. Pushing on open-loop "
                    f"{remaining:.2f} m ({self.push_seconds:.1f} s) before landing.")
            elif now - self.push_since >= self.push_seconds:
                self._abandon("flow lost in the gap; pushed through open-loop")
            return
        if self.push_since is not None:
            self.get_logger().warning("PASS: flow is back; resuming.")
            self.push_since = None

        if self._arrived(self.pass_exit):
            self.MOVE_SPEED = self.SHIFT_SPEED
            self._enter_tube_stage(self.SHIFT)
            self._set_target(self.shift_point, self.cross_altitude)
            self.get_logger().warning(
                f"SHIFT: through. Stepping {self.SHIFT_LEFT:+.2f} m left of the "
                "gap line to clear the back upright.")
            return
        if self._in_stage_for() > self.PASS_TIMEOUT:
            self._abandon("the pass timed out")
            return
        self.get_logger().info(
            f"PASS: {self._distance_past_plane():+.2f} m past the plane.",
            throttle_duration_sec=0.5)

    def _handle_shift(self):
        self._aim_yaw_at(self.gap_heading)
        if not self.hold_xy:
            self._abandon("flow lost during the shift; landing straight down")
            return
        if self._arrived(self.shift_point):
            self.MOVE_SPEED = self.APPROACH_SPEED
            self._enter_tube_stage(self.EXIT)
            self._set_target(self.final_point, self.cross_altitude)
            self.get_logger().warning(
                f"EXIT: {self.EXIT_DISTANCE:.2f} m on, past the back upright.")
            return
        if self._in_stage_for() > self.MOVE_STAGE_TIMEOUT:
            self._abandon("the shift timed out")

    def _handle_exit(self):
        self._aim_yaw_at(self.gap_heading)
        if not self.hold_xy:
            self._abandon("flow lost during the exit; landing straight down")
            return
        if self._arrived(self.final_point):
            self.outcome = f"TUBES CROSSED ({self.gap_side} gap at {self.cross_altitude:.2f} m)"
            self._enter_tube_stage(self.CLEAR)
            return
        if self._in_stage_for() > self.MOVE_STAGE_TIMEOUT:
            self._abandon("the exit timed out")

    def _handle_clear(self):
        if self._in_stage_for() >= self.CLEAR_SECONDS:
            self._begin_landing("tubes crossed")

    def _abandon(self, reason):
        self.outcome = f"ABANDONED: {reason}"
        self.moving = False
        self.yaw_remaining = 0.0
        self.MOVE_SPEED = self.APPROACH_SPEED
        self.get_logger().error(f"Tube crossing abandoned: {reason}.")
        self._begin_landing(f"tube crossing abandoned -- {reason}")

    # --------------------------------------------------------------- output

    def publish_position_setpoint(self):
        """The inherited setpoint, except for the open-loop push in the gap."""
        if not (self.current_stage == self.PASS and self.push_since is not None
                and not self.hold_xy and self.home_z is not None):
            super().publish_position_setpoint()
            return
        nan = float('nan')
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self._step_setpoint_ramp()
        self._step_yaw_ramp()
        msg.position = [nan, nan, self.setpoint_z]
        msg.velocity = [self.PASS_SPEED * float(self.gap_normal[0]),
                        self.PASS_SPEED * float(self.gap_normal[1]), nan]
        msg.yaw = self.yaw_setpoint
        self.trajectory_setpoint_pub.publish(msg)

    def publish_status(self):
        if self.current_stage not in self.TUBE_STAGES:
            super().publish_status()
            return
        alt = self.relative_altitude()
        armed = self.arming_state == VehicleStatus.ARMING_STATE_ARMED
        if self.current_stage in (self.SEARCH, self.LOCK):
            detail = 'gap' if self.solution is not None else 'look'
        elif self.current_stage in (self.ALIGN, self.PASS):
            detail = f"p{self._distance_past_plane():+.1f}"
        else:
            detail = self.current_stage.lower()[:4]
        msg = String()
        msg.data = "|".join([
            self.current_stage, 'ARM' if armed else 'DIS',
            f"{alt:.2f}" if alt is not None else 'nan',
            'POS' if self.hold_xy else ('FLO' if self.flow_is_healthy() else '---'),
            detail])
        self.status_pub.publish(msg)

    def log_flight_state(self):
        super().log_flight_state()
        self.get_logger().info(f"tubes: {self.gap_summary()}", throttle_duration_sec=2.0)

    def destroy_node(self):
        self.get_logger().warning(
            f"Tube crossing outcome: {self.outcome}. "
            f"{self.estimator.accepted_total} samples accepted from "
            f"{self.geometry_seen} frames; rejections: "
            f"{self.estimator.rejection_summary(limit=5)}.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TubeCross()
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
