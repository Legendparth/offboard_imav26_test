"""
Takeoff -> sweep for the window -> estimate where it is -> line up on it ->
fly through it -> land. Localisation is ZED visual odometry throughout.

    arm -> sit on the ground -> climb -> hold -> SCAN (yaw sweep) ->
    LOCK (stop, face it, build a pose estimate) -> AIM (yaw onto the window
    normal) -> ALIGN (fly to a point standoff_distance in front of the
    window, on its axis, at its height) -> TRAVERSE (commit and fly through)
    -> CLEAR (hold beyond it) -> land

    q -> abort into a controlled descent.   k -> force-disarm.

WHAT IS INHERITED AND WHY
-------------------------
Everything up to and including the lock is somebody else's already-flown
code, reached by inheriting from both halves of it:

    WindowScan          the yaw sweep, the debounced detection, the lock,
                        the flight clock
    OffboardSequenceVio the vision health predicate, the on-the-ground x/y
                        latch, the yaw-alignment arming gate
    OffboardSequence    arming, the climb, the ramps and leashes, the
                        estimator-reset bookkeeping, the descent, the
                        touchdown detection, the keyboard aborts

so the only new flight code in this file is the four stages after the lock,
and the only new sensing code is the window pose estimator. Nothing about
how the aircraft climbs or lands is duplicated here, which is the point:
those are the parts that hurt people, and they should exist once.

WHICH FRAME THE SETPOINTS ARE IN
--------------------------------
This is the question that has to be answered before any of the rest makes
sense, so: every setpoint this node sends PX4 is an ABSOLUTE POINT IN THE
PX4 LOCAL NED FRAME -- the same frame /fmu/out/vehicle_local_position
reports x, y and z in, with its origin wherever EKF2 datumed itself and z
POSITIVE DOWN. Not body-relative, not relative to the arming point.

The inherited sequence node hides that behind direction words ("forward
1.0" resolves against a reference yaw into an NED target), but underneath
it is walking `hold_x` / `hold_y` -- an NED point -- towards `move_target_x`
/ `move_target_y`, also an NED point, and publishing that point. This node
skips the direction words and computes the NED targets directly, because
the window's position is naturally an absolute point and converting it into
"forward 1.4, right 0.3" and back again would only lose precision.

The one axis that is relative is altitude, and only in the bookkeeping:
`commanded_altitude` is metres above the ARMING POINT, and it is turned into
an NED z as `home_z - commanded_altitude` before it goes anywhere near PX4.

WHERE THE WINDOW'S POSITION COMES FROM
--------------------------------------
window_detect publishes /window_geometry every frame it sees a quadrilateral:
four corners and the centre, each as (depth, azimuth, elevation) in the
CAMERA frame. Turning that into a point in NED is three transforms:

    1. rays to camera-frame points   x=d, y=d*tan(az), z=-d*tan(el)
       This is exact rather than approximate because the ZED's depth is the
       distance along the optical axis, not the slant range.
    2. camera frame to body FRD      the fixed mounting rotation and lever
       arm (cam_x/cam_y/cam_z, cam_roll/cam_pitch/cam_yaw -- the SAME
       numbers, in the same ROS convention, that zed_localization takes)
    3. body FRD to NED               rotate by the vehicle attitude
       quaternion, then add the vehicle position

Step 3 uses the full attitude, not just the heading. That matters more than
it looks: a vehicle translating at 0.4 m/s sits at 5-10 degrees of pitch,
and at 3 m range a 10 degree pitch error puts the window half a metre off
vertically. Using the heading alone would make the aircraft fly at a window
that appears to move up and down as it accelerates.

THE OUTLIERS, WHICH ARE THE ACTUAL PROBLEM
------------------------------------------
Stereo depth on a thin green frame is not a well-behaved measurement. It
fails in one specific way: a sample box that lands a few pixels off the
frame reads the WALL BEHIND (metres too far) or nothing at all (zero), and
one such corner drags a naive four-corner average metres out of position.
So no single frame is ever trusted. Five independent filters stand between
a depth pixel and a setpoint:

    per corner      a non-positive depth, or one outside
                    [depth_min, depth_max], voids that corner
    per sample      the four corner depths must agree with their own median
                    to within corner_spread; the reconstructed quad must be
                    planar to within plane_tolerance, must have sides
                    between window_min_size and window_max_size, must have
                    opposite sides matching to within side_mismatch, and its
                    normal must be within max_tilt_deg of horizontal
    innovation      once an estimate exists, a sample whose centre is more
                    than gate_metres from it, or whose normal is more than
                    gate_yaw_deg off it, is rejected outright
    temporal        the estimate is the component-wise MEDIAN of every
                    accepted sample in the last buffer_seconds, not a mean
                    and not an EMA of one -- a median is the filter that
                    ignores an outlier instead of averaging it in
    quorum          nothing flies anywhere until min_samples accepted
                    samples exist and the newest is younger than pose_max_age

A median over a rolling window is deliberately chosen over the exponential
average in drone_imav_obs_course: an EMA with alpha=0.25 still moves 25 cm
towards a sample that is a metre wrong, and it does so on the first frame.
The median moves not at all until half the buffer agrees.

If the gate rejects gate_reset_count samples in a row, the buffer is thrown
away and rebuilt from scratch -- that is the case where the estimate itself
is the thing that is wrong, and refusing every sample forever is worse than
starting again.

DOES IT KEEP UPDATING WHILE IT FLIES?
-------------------------------------
Yes, through SCAN, LOCK, AIM and ALIGN: every frame goes into the estimator
and the approach target is recomputed from the current estimate on every
tick, so the aircraft is chasing the window's best-known position, not the
one it had when it first saw it.

At the start of TRAVERSE it COMMITS: the target is frozen and the estimator
is ignored. That is not laziness, it is the only safe reading of the
geometry. Passing through a window means the window leaves the field of
view, fills it, and finally is behind the camera; the last few metres of
depth on a frame edge are the least trustworthy data the camera produces,
and the aircraft is at its least able to act on a correction. The estimate
that lined the aircraft up from a metre and a half away, with the whole
window in frame, is better than anything measurable from inside it.

If vision itself dies during the traverse, the aircraft does NOT stop in the
window frame. It flies the remaining distance as a fixed velocity along the
committed heading for up to blind_traverse_seconds and then lands. Stopping
halfway through an aperture is the one outcome worth spending open-loop
seconds to avoid.

YAW
---
The nose is kept pointing along the direction of travel -- i.e. at the
window, and then through it -- for the whole approach. Two reasons, neither
of them cosmetic: the camera has to keep seeing the window for the estimate
to keep updating, and a vehicle crabbing sideways through an aperture needs
the aperture to be wider than its diagonal rather than its width. The yaw is
walked by the inherited ramp at yaw_rate, leashed to the measured heading,
and AIM does the bulk of the turn standing still, before any translation, so
the two never happen fast at the same time -- simultaneous yaw and
translation is the single most reliable way to make an IMU-less stereo
camera lose tracking.
"""

import math
import time
from collections import deque

import numpy as np

import rclpy
from px4_msgs.msg import TrajectorySetpoint, VehicleAttitude, VehicleStatus
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray, String

from drone_testing.offboard_sequence import wrap_pi
from drone_testing.offboard_sequence_vio import OffboardSequenceVio
from drone_testing.window_scan import WindowScan


# ----------------------------------------------------------------- geometry

def quat_rotate(q, v):
    """Rotate v by the Hamiltonian quaternion q = (w, x, y, z).

    px4_msgs/VehicleAttitude.q is the rotation that takes a vector expressed
    in the BODY (FRD) frame to the same vector expressed in the local NED
    frame, so this is exactly the body -> NED step and needs no inverse.
    """
    w = q[0]
    u = np.asarray(q[1:4], dtype=float)
    t = 2.0 * np.cross(u, v)
    return v + w * t + np.cross(u, t)


def rpy_to_matrix_frd(roll, pitch, yaw):
    """Mounting rotation as an FRD matrix, from angles given ROS-style.

    The angles are the pose of the camera in the body frame in the ROS
    convention -- x forward, y LEFT, z UP, applied yaw then pitch then roll --
    because those are the same three numbers zed_localization already takes
    for the same camera, and having the two nodes disagree about the sign of
    cam_pitch is a bug nobody would find in the air.

    FLU and FRD differ by flipping y and z, and D = diag(1, -1, -1) is its own
    inverse, so R_frd = D R_flu D.
    """
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    d = np.diag([1.0, -1.0, -1.0])
    return d @ (rz @ ry @ rx) @ d


# ------------------------------------------------------------- the estimator

class WindowEstimator:
    """Turns a stream of camera-frame quadrilaterals into one NED window pose.

    Deliberately free of ROS and of the flight node: it takes numbers in and
    gives numbers out, which is what makes the rejection logic -- the part
    that actually decides whether the aircraft flies at the right place --
    testable on the bench without a vehicle.

    add() returns (accepted, reason). Every rejection carries the reason it
    was rejected, and the node logs a rolling tally of them, because "the
    window estimate is not converging" is useless and "37 of the last 40
    samples failed the planarity test" tells you the sample box is landing on
    the wall behind the frame.
    """

    def __init__(self, *, depth_min, depth_max, corner_spread, corner_spread_frac,
                 plane_tolerance, min_size, max_size, side_mismatch, max_tilt,
                 buffer_seconds, buffer_max, min_samples, gate_metres, gate_yaw,
                 gate_reset_count):
        self.depth_min = depth_min
        self.depth_max = depth_max
        self.corner_spread = corner_spread
        self.corner_spread_frac = corner_spread_frac
        self.plane_tolerance = plane_tolerance
        self.min_size = min_size
        self.max_size = max_size
        self.side_mismatch = side_mismatch
        self.max_tilt = max_tilt            # rad, off horizontal
        self.buffer_seconds = buffer_seconds
        self.min_samples = min_samples
        self.gate_metres = gate_metres
        self.gate_yaw = gate_yaw            # rad
        self.gate_reset_count = gate_reset_count

        self.samples = deque(maxlen=buffer_max)
        self.rejections = {}
        self.accepted_total = 0
        self.consecutive_gated = 0
        self.last_reason = ''

    # ------------------------------------------------------------ ingestion

    def add(self, geometry, q_att, p_ned, r_cam, t_cam, now):
        """One /window_geometry message, with the vehicle pose it belongs to.

        geometry is the 5x3 (depth_m, az_deg, el_deg) array; only the four
        corner rows are used. The centre row is redundant here -- the centroid
        of four validated corners is a better centre than one depth sample at
        the middle of an OPEN window, where the depth pixel is looking at
        whatever is on the far side of the room.
        """
        corners_cam = []
        for depth, az_deg, el_deg in geometry[:4]:
            if not np.isfinite(depth) or depth <= 0.0:
                return self._reject('corner depth missing')
            if depth < self.depth_min or depth > self.depth_max:
                return self._reject('corner depth out of range')
            az = math.radians(float(az_deg))
            el = math.radians(float(el_deg))
            corners_cam.append([float(depth),
                                float(depth) * math.tan(az),
                                -float(depth) * math.tan(el)])
        corners_cam = np.array(corners_cam)

        # The four depths must agree with each other. A window seen at an
        # angle genuinely has a depth spread, so the allowance is the larger of
        # an absolute and a proportional one -- a 20 cm disagreement at 1.5 m
        # is a squint, the same 20 cm at 6 m is a corner on the far wall.
        depths = corners_cam[:, 0]
        median_depth = float(np.median(depths))
        allowed = max(self.corner_spread, self.corner_spread_frac * median_depth)
        if float(np.max(np.abs(depths - median_depth))) > allowed:
            return self._reject('corner depths disagree')

        # Camera -> body FRD -> NED. Done before the shape tests so those are
        # applied to the same points the setpoint will be derived from.
        corners_body = corners_cam @ r_cam.T + t_cam
        corners_ned = np.array([quat_rotate(q_att, p) for p in corners_body]) + p_ned

        centre = corners_ned.mean(axis=0)
        rel = corners_ned - centre

        # Sides, in the detector's corner order: TL, TR, BR, BL walking round
        # the quad, so 0-1 and 3-2 are the horizontals and 0-3 and 1-2 the
        # verticals.
        top = np.linalg.norm(corners_ned[1] - corners_ned[0])
        bottom = np.linalg.norm(corners_ned[2] - corners_ned[3])
        left = np.linalg.norm(corners_ned[3] - corners_ned[0])
        right = np.linalg.norm(corners_ned[2] - corners_ned[1])
        width = 0.5 * (top + bottom)
        height = 0.5 * (left + right)

        if not (self.min_size <= width <= self.max_size
                and self.min_size <= height <= self.max_size):
            return self._reject('implausible window size')
        if (abs(top - bottom) > self.side_mismatch * max(top, bottom)
                or abs(left - right) > self.side_mismatch * max(left, right)):
            return self._reject('opposite sides disagree')

        # Planarity. The smallest right-singular vector of the centred corners
        # is the plane normal, and the residual along it is how far from a
        # plane those four points are. A corner that has grabbed the wall
        # behind fails this even when it passed the depth-spread test, because
        # it is off the plane in the direction the spread test cannot see.
        try:
            _, _, vt = np.linalg.svd(rel)
        except np.linalg.LinAlgError:
            return self._reject('degenerate quad')
        normal = vt[2]
        if float(np.max(np.abs(rel @ normal))) > self.plane_tolerance:
            return self._reject('corners not coplanar')

        # Point the normal back at the aircraft, so "in front of the window"
        # is unambiguously +normal for everything downstream.
        if float(np.dot(normal, p_ned - centre)) < 0.0:
            normal = -normal

        # A window is vertical. A normal that is not horizontal means the quad
        # is the floor, a ceiling light or a badly-cornered detection, and
        # flying along it would fly the aircraft into the ground.
        horizontal = math.hypot(normal[0], normal[1])
        if horizontal < math.cos(self.max_tilt):
            return self._reject('window is not vertical')
        normal_h = np.array([normal[0], normal[1], 0.0]) / horizontal

        # Innovation gate against the estimate we already believe.
        est = self.estimate(now)
        if est is not None:
            if float(np.linalg.norm(centre - est['centre'])) > self.gate_metres:
                return self._gated('centre jumped')
            if abs(wrap_pi(math.atan2(normal_h[1], normal_h[0])
                           - math.atan2(est['normal'][1], est['normal'][0]))) > self.gate_yaw:
                return self._gated('normal swung')

        self.consecutive_gated = 0
        self.accepted_total += 1
        self.last_reason = ''
        self.samples.append({
            't': now,
            'centre': centre,
            'normal': normal_h,
            'width': width,
            'height': height,
        })
        return True, ''

    def _reject(self, reason):
        self.rejections[reason] = self.rejections.get(reason, 0) + 1
        self.last_reason = reason
        return False, reason

    def _gated(self, reason):
        self.consecutive_gated += 1
        if self.consecutive_gated >= self.gate_reset_count:
            # Every sample disagrees with the estimate. At that point the
            # estimate is the minority opinion and keeping it is how a vehicle
            # ends up flying confidently at nothing.
            self.samples.clear()
            self.consecutive_gated = 0
            return self._reject(reason + ' -- estimate discarded, rebuilding')
        return self._reject(reason)

    # -------------------------------------------------------------- output

    def _fresh(self, now):
        cutoff = now - self.buffer_seconds
        return [s for s in self.samples if s['t'] >= cutoff]

    def estimate(self, now):
        """The current robust pose, or None if there is not enough evidence.

        centre and normal are component-wise medians over the buffer. The
        median of a set of unit vectors is not a unit vector, so the normal is
        re-normalised; with samples that have already passed the innovation
        gate they are all within gate_yaw of each other and the renormalised
        median is a sane direction.
        """
        fresh = self._fresh(now)
        if len(fresh) < self.min_samples:
            return None

        centre = np.median(np.array([s['centre'] for s in fresh]), axis=0)
        normal = np.median(np.array([s['normal'] for s in fresh]), axis=0)
        norm = float(np.linalg.norm(normal[:2]))
        if norm < 1e-6:
            return None
        normal = np.array([normal[0] / norm, normal[1] / norm, 0.0])

        return {
            'centre': centre,
            'normal': normal,
            'width': float(np.median([s['width'] for s in fresh])),
            'height': float(np.median([s['height'] for s in fresh])),
            'samples': len(fresh),
            'age': now - fresh[-1]['t'],
        }

    def fresh_count(self, now):
        """How many accepted samples are inside the buffer window."""
        return len(self._fresh(now))

    def rejection_summary(self, limit=3):
        if not self.rejections:
            return 'none'
        worst = sorted(self.rejections.items(), key=lambda kv: -kv[1])[:limit]
        return ', '.join(f"{name} x{count}" for name, count in worst)


# ---------------------------------------------------------------- the flight

class WindowTraverse(WindowScan, OffboardSequenceVio):
    """Sweep, lock, line up, fly through.

    The base list is the whole design. WindowScan brings the sweep and the
    lock; OffboardSequenceVio brings the vision health predicate that the
    inherited horizontal control is gated on. Python's MRO puts them in that
    order over the one shared OffboardSequence, so `flow_is_healthy` resolves
    to the vision version and `_handle_scan` to the sweep, with no copy of
    either living here.
    """

    AIM = "AIM"
    ALIGN = "ALIGN"
    TRAVERSE = "TRAVERSE"
    CLEAR = "CLEAR"

    TRAVERSE_STAGES = (AIM, ALIGN, TRAVERSE, CLEAR)

    # ---- the approach -----------------------------------------------------
    STANDOFF_DISTANCE = 1.60    # m in front of the window plane the approach
                                # aims for. Far enough that the whole window is
                                # still in frame (at 90 deg HFOV a 1 m window
                                # subtends ~35 deg here) and close enough that
                                # the run through it is short.
    EXIT_DISTANCE = 1.50        # m beyond the window plane the traverse ends
    ALTITUDE_OFFSET = 0.0       # m added to the window centre height. Positive
                                # is higher. Leave at 0 unless the detected
                                # quad is known to sit off-centre on the frame.

    APPROACH_SPEED = 0.30       # m/s the carrot is walked at during ALIGN
    TRAVERSE_SPEED = 0.45       # m/s during the run through. Faster than the
                                # approach: less time in the aperture, and by
                                # then the estimate is frozen so there is
                                # nothing left to track.

    ALIGN_TOLERANCE = 0.18      # m radius around the standoff point
    ALIGN_ALT_TOLERANCE = 0.15  # m
    ALIGN_YAW_TOLERANCE = math.radians(8.0)
    ALIGN_SETTLE_SECONDS = 1.5  # all three held simultaneously for this long
    AIM_YAW_TOLERANCE = math.radians(12.0)

    AIM_TIMEOUT = 25.0
    ALIGN_TIMEOUT = 60.0
    TRAVERSE_TIMEOUT = 25.0
    CLEAR_SECONDS = 4.0

    # ---- the estimate -----------------------------------------------------
    GEOMETRY_TOPIC = 'window_geometry'
    POSE_MAX_AGE = 1.5          # s. Older than this and the estimate is not
                                # evidence about where the window is now.
    POSE_LOST_TIMEOUT = 6.0     # s without a usable estimate during AIM/ALIGN
                                # before the attempt is given up on
    BLIND_TRAVERSE_SECONDS = 3.0

    DEPTH_MIN = 0.35
    DEPTH_MAX = 8.00
    CORNER_SPREAD = 0.25        # m
    CORNER_SPREAD_FRAC = 0.15   # of the median corner depth
    PLANE_TOLERANCE = 0.15      # m. Note the factor of four: a best-fit plane
                                # through four points splits a single bad
                                # corner's error between all of them, so a
                                # corner that is X out of plane only shows a
                                # residual of X/4 here. This test alone would
                                # pass a 60 cm outlier, which is why the depth
                                # spread test above it is the primary defence
                                # and this one is the backstop for the case the
                                # spread test cannot see -- a corner displaced
                                # ACROSS the frame rather than along the ray.
    WINDOW_MIN_SIZE = 0.35      # m
    WINDOW_MAX_SIZE = 3.00      # m
    SIDE_MISMATCH = 0.40        # fraction
    MAX_TILT_DEG = 35.0         # of the normal, off horizontal
    BUFFER_SECONDS = 2.5
    BUFFER_MAX = 60
    MIN_SAMPLES = 6
    GATE_METRES = 1.00
    GATE_YAW_DEG = 40.0
    GATE_RESET_COUNT = 25

    # A traversal is a longer flight than a scan, so the inherited 40 s clock
    # would land the aircraft in the middle of the approach.
    FLIGHT_SECONDS = 150.0

    def __init__(self):
        super().__init__('window_traverse')

        self.STANDOFF_DISTANCE = float(self._declare_number(
            'standoff_distance', self.STANDOFF_DISTANCE))
        self.EXIT_DISTANCE = float(self._declare_number(
            'exit_distance', self.EXIT_DISTANCE))
        self.ALTITUDE_OFFSET = float(self._declare_number(
            'altitude_offset', self.ALTITUDE_OFFSET))
        self.APPROACH_SPEED = float(self._declare_number(
            'approach_speed', self.APPROACH_SPEED))
        self.TRAVERSE_SPEED = float(self._declare_number(
            'traverse_speed', self.TRAVERSE_SPEED))
        self.ALIGN_TOLERANCE = float(self._declare_number(
            'align_tolerance', self.ALIGN_TOLERANCE))
        self.ALIGN_SETTLE_SECONDS = float(self._declare_number(
            'align_settle_seconds', self.ALIGN_SETTLE_SECONDS))
        self.ALIGN_YAW_TOLERANCE = math.radians(float(self._declare_number(
            'align_yaw_tolerance_deg', math.degrees(self.ALIGN_YAW_TOLERANCE))))
        self.ALIGN_TIMEOUT = float(self._declare_number(
            'align_timeout', self.ALIGN_TIMEOUT))
        self.TRAVERSE_TIMEOUT = float(self._declare_number(
            'traverse_timeout', self.TRAVERSE_TIMEOUT))
        self.CLEAR_SECONDS = float(self._declare_number(
            'clear_seconds', self.CLEAR_SECONDS))
        self.POSE_MAX_AGE = float(self._declare_number(
            'pose_max_age', self.POSE_MAX_AGE))
        self.POSE_LOST_TIMEOUT = float(self._declare_number(
            'pose_lost_timeout', self.POSE_LOST_TIMEOUT))
        self.BLIND_TRAVERSE_SECONDS = float(self._declare_number(
            'blind_traverse_seconds', self.BLIND_TRAVERSE_SECONDS))
        self.MIN_SAMPLES = int(self._declare_number('pose_min_samples', self.MIN_SAMPLES))

        # The approach is flown by the inherited carrot, whose speed is
        # MOVE_SPEED. Setting it here rather than threading a second speed
        # through _step_xy_ramp keeps that ramp -- and its leash -- the only
        # thing that ever moves the horizontal setpoint.
        self.MOVE_SPEED = self.APPROACH_SPEED

        # Camera mounting: the pose of the camera in the body frame, ROS
        # convention (x fwd, y LEFT, z UP), exactly as zed_localization takes
        # it. Give both nodes the same numbers.
        cam_x = float(self._declare_number('cam_x', 0.0))
        cam_y = float(self._declare_number('cam_y', 0.0))
        cam_z = float(self._declare_number('cam_z', 0.0))
        cam_roll = float(self._declare_number('cam_roll', 0.0))
        cam_pitch = float(self._declare_number('cam_pitch', 0.0))
        cam_yaw = float(self._declare_number('cam_yaw', 0.0))
        self.r_cam = rpy_to_matrix_frd(cam_roll, cam_pitch, cam_yaw)
        self.t_cam = np.array([cam_x, -cam_y, -cam_z])   # ROS FLU -> body FRD

        self.estimator = WindowEstimator(
            depth_min=float(self._declare_number('depth_min', self.DEPTH_MIN)),
            depth_max=float(self._declare_number('depth_max', self.DEPTH_MAX)),
            corner_spread=float(self._declare_number('corner_spread', self.CORNER_SPREAD)),
            corner_spread_frac=float(self._declare_number(
                'corner_spread_frac', self.CORNER_SPREAD_FRAC)),
            plane_tolerance=float(self._declare_number(
                'plane_tolerance', self.PLANE_TOLERANCE)),
            min_size=float(self._declare_number('window_min_size', self.WINDOW_MIN_SIZE)),
            max_size=float(self._declare_number('window_max_size', self.WINDOW_MAX_SIZE)),
            side_mismatch=float(self._declare_number('side_mismatch', self.SIDE_MISMATCH)),
            max_tilt=math.radians(float(self._declare_number(
                'max_tilt_deg', self.MAX_TILT_DEG))),
            buffer_seconds=float(self._declare_number('buffer_seconds', self.BUFFER_SECONDS)),
            buffer_max=self.BUFFER_MAX,
            min_samples=self.MIN_SAMPLES,
            gate_metres=float(self._declare_number('gate_metres', self.GATE_METRES)),
            gate_yaw=math.radians(float(self._declare_number(
                'gate_yaw_deg', self.GATE_YAW_DEG))),
            gate_reset_count=int(self._declare_number(
                'gate_reset_count', self.GATE_RESET_COUNT)),
        )

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # The attitude is a separate topic from the local position; the local
        # position message carries only the heading, and the heading alone is
        # not enough to place a point that is 3 m in front of a pitching
        # vehicle. See the header.
        self.attitude = None
        self.attitude_time = None
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude',
                                 self.attitude_callback, qos_profile=sensor_qos)

        geometry_topic = str(self.declare_parameter(
            'geometry_topic', self.GEOMETRY_TOPIC).value)
        self.create_subscription(Float32MultiArray, geometry_topic,
                                 self.geometry_callback, 10)

        # What the estimator currently believes, for anyone watching from the
        # ground: x|y|z|yaw_deg|width|height|samples|age.
        self.window_pose_pub = self.create_publisher(String, 'window_pose', 10)

        self.geometry_seen = 0
        self.pose_ok_since = None       # monotonic time the estimate went usable
        self.pose_lost_since = None     # ... and when it stopped being usable

        # The committed traverse, frozen at the start of TRAVERSE.
        self.traverse_entry = None      # np(3) NED, the standoff point
        self.traverse_exit = None       # np(3) NED, beyond the window
        self.traverse_heading = None    # rad, NED
        self.traverse_window = None     # the estimate it was committed from
        self.blind_traverse_since = None

        self.align_in_band_since = None
        self.outcome = 'not attempted'

        self.get_logger().warning(
            f"Window traversal on ZED VISION: climb {self.TAKEOFF_ALTITUDE:.2f} m, "
            f"hold {self.HOLD_SECONDS:.0f} s, sweep +/-"
            f"{math.degrees(self.SCAN_SPAN) / 2:.0f} deg for the window, lock, "
            f"line up {self.STANDOFF_DISTANCE:.2f} m in front of it and fly "
            f"through to {self.EXIT_DISTANCE:.2f} m beyond, then land. "
            f"Approach {self.APPROACH_SPEED:.2f} m/s, traverse "
            f"{self.TRAVERSE_SPEED:.2f} m/s. Hard limit "
            f"{self.FLIGHT_SECONDS:.0f} s from the start of the climb. "
            "Press q to abort into a descent, k to force-disarm.")

    # ------------------------------------------------------------------ subs

    def attitude_callback(self, msg):
        self.attitude = msg
        self.attitude_time = time.monotonic()

    def geometry_callback(self, msg):
        """One frame's worth of window geometry, paired with where we are.

        The pairing is by ARRIVAL, not by timestamp: Float32MultiArray has no
        header to carry one. The lag between the frame being grabbed and this
        callback is a frame time plus transport, call it 60-100 ms, which at
        the 0.3-0.45 m/s this node flies is 2-5 cm of position error on the
        sample. That is well inside the tolerances everything downstream is
        built on, and it is systematic rather than random, so the median does
        not remove it -- worth knowing before you chase the last 5 cm of
        alignment accuracy.
        """
        self.geometry_seen += 1

        data = np.asarray(msg.data, dtype=float)
        if data.size < 15:
            self.get_logger().warning(
                f"/window_geometry has {data.size} values, expected at least 15. "
                "Is window_detect up to date?", throttle_duration_sec=5.0)
            return
        geometry = data[:15].reshape(5, 3)

        lp = self.local_position
        if lp is None or not lp.xy_valid or not lp.z_valid:
            return
        if self.attitude is None:
            self.get_logger().warning(
                "No /fmu/out/vehicle_attitude yet; cannot place the window.",
                throttle_duration_sec=5.0)
            return

        p_ned = np.array([lp.x, lp.y, lp.z])
        self.estimator.add(geometry, np.asarray(self.attitude.q, dtype=float),
                           p_ned, self.r_cam, self.t_cam, time.monotonic())

    # -------------------------------------------------------------- estimate

    def window_estimate(self):
        """The estimate, or None if it is missing, thin or stale."""
        est = self.estimator.estimate(time.monotonic())
        if est is None or est['age'] > self.POSE_MAX_AGE:
            return None
        return est

    def _track_pose_health(self):
        """Bookkeeping for how long the estimate has been usable, or not."""
        now = time.monotonic()
        if self.window_estimate() is not None:
            self.pose_lost_since = None
            if self.pose_ok_since is None:
                self.pose_ok_since = now
        else:
            self.pose_ok_since = None
            if self.pose_lost_since is None:
                self.pose_lost_since = now

    def pose_summary(self):
        est = self.window_estimate()
        if est is None:
            fresh = self.estimator.fresh_count(time.monotonic())
            if self.geometry_seen == 0:
                return ("no /window_geometry at all -- is window_detect new "
                        "enough to publish it?")
            return (f"no usable pose ({fresh}/{self.MIN_SAMPLES} fresh samples, "
                    f"{self.estimator.accepted_total} accepted ever; "
                    f"rejections: {self.estimator.rejection_summary()})")
        return (f"window at ({est['centre'][0]:+.2f}, {est['centre'][1]:+.2f}, "
                f"{est['centre'][2]:+.2f}) NED, facing "
                f"{math.degrees(math.atan2(est['normal'][1], est['normal'][0])):+.0f} deg, "
                f"{est['width']:.2f}x{est['height']:.2f} m, {est['samples']} samples, "
                f"{est['age'] * 1000:.0f} ms old")

    def publish_window_pose(self):
        est = self.window_estimate()
        msg = String()
        if est is None:
            msg.data = ''
        else:
            msg.data = "|".join([
                f"{est['centre'][0]:.3f}", f"{est['centre'][1]:.3f}",
                f"{est['centre'][2]:.3f}",
                f"{math.degrees(math.atan2(est['normal'][1], est['normal'][0])):.1f}",
                f"{est['width']:.3f}", f"{est['height']:.3f}",
                f"{est['samples']}", f"{est['age']:.3f}",
            ])
        self.window_pose_pub.publish(msg)

    # --------------------------------------------------------- the geometry

    def approach_points(self, est):
        """(entry, exit, heading) for an estimate, all in NED.

        entry is standoff_distance in front of the window ON ITS AXIS -- along
        the normal, which is what makes the approach square to the aperture
        rather than merely near it. exit is exit_distance past the window on
        the same line. heading points from entry to exit, i.e. at the window
        and then through it.

        The altitude of both is the window centre's, clamped into the flight
        envelope by the caller before it becomes a setpoint.
        """
        centre = est['centre']
        normal = est['normal']
        entry = centre + normal * self.STANDOFF_DISTANCE
        exit_point = centre - normal * self.EXIT_DISTANCE
        heading = math.atan2(-normal[1], -normal[0])
        return entry, exit_point, heading

    def window_altitude(self, est):
        """Height above the arming point to fly the traverse at, clamped.

        Returns None before home_z exists, which cannot happen from any stage
        that calls it -- home is captured at arming -- but the flight envelope
        clamp is real: a window estimate that has gone wrong vertically must
        not be able to command a climb past max_altitude or a descent into the
        floor. When the clamp bites it is reported, because a window whose
        estimated centre is outside the envelope is usually a bad estimate
        rather than a high window.
        """
        if self.home_z is None:
            return None
        wanted = self.home_z - est['centre'][2] + self.ALTITUDE_OFFSET
        clamped = min(max(wanted, self.MIN_ALTITUDE), self.MAX_ALTITUDE)
        if abs(clamped - wanted) > 1e-3:
            self.get_logger().warning(
                f"Window centre is at {wanted:.2f} m above the arming point, "
                f"outside the {self.MIN_ALTITUDE:.2f}-{self.MAX_ALTITUDE:.2f} m "
                f"envelope. Flying the traverse at {clamped:.2f} m instead.",
                throttle_duration_sec=5.0)
        return clamped

    def _set_target(self, x, y, altitude=None):
        """Point the inherited ramps at an NED point and an altitude.

        This is the whole of "how a setpoint is given" in this node: the x/y
        carrot target and the z ramp target. Everything about how they are
        walked -- the speed, the leash to the measured position, the fact that
        what is published is an absolute NED point -- is the base class's, and
        is not re-implemented here.
        """
        self.move_target_x = float(x)
        self.move_target_y = float(y)
        self.moving = True
        if altitude is not None:
            self.commanded_altitude = float(altitude)
            self.target_z = self.home_z - float(altitude)

    def _aim_yaw_at(self, heading):
        """Walk the commanded yaw towards an absolute NED heading.

        Rewritten every tick rather than latched once, so a target that moves
        as the estimate refines is followed instead of being flown to once and
        forgotten. yaw_remaining is what the inherited ramp consumes, and the
        ramp is still what limits the rate and holds the leash.
        """
        self.yaw_remaining = wrap_pi(heading - self.yaw_setpoint)

    def _heading_error(self, heading):
        lp = self.local_position
        if lp is None:
            return math.pi
        return abs(wrap_pi(heading - lp.heading))

    # ---------------------------------------------------------- state machine

    def _clock_stages(self):
        """The inherited hard clock, extended to the new stages.

        TRAVERSE is deliberately NOT on the list. Once committed, the aircraft
        is somewhere between a metre and a half in front of an aperture and a
        metre and a half past it, and starting a descent from inside a window
        frame because a stopwatch ran out is not a safety behaviour. The
        traverse has its own, much shorter, timeout; the clock catches the
        aircraft again in CLEAR immediately afterwards.
        """
        return super()._clock_stages() + (self.AIM, self.ALIGN, self.CLEAR)

    def timer_callback(self):
        self.publish_window_pose()

        if self.current_stage not in self.TRAVERSE_STAGES:
            # SCAN and LOCK are WindowScan's; everything else is the base
            # class's. Both are reached through the same call.
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

        self._track_pose_health()

        {
            self.AIM: self._handle_aim,
            self.ALIGN: self._handle_align,
            self.TRAVERSE: self._handle_traverse,
            self.CLEAR: self._handle_clear,
        }[self.current_stage]()

    # ------------------------------------------------------------- the lock

    def _handle_lock(self):
        """Hold facing the window until the pose estimate is good enough.

        WindowScan's version of this stage parks and waits for the flight
        clock. Here it is the sampling stage: the aircraft is stationary and
        pointed at the window, which is the best possible geometry for stereo
        depth on a thin frame, and it stays there until min_samples of the
        estimator's accepted samples exist.
        """
        super()._handle_lock()
        if self.current_stage != self.LOCK:
            # The parent went back to scanning, or started a landing.
            return

        self._track_pose_health()

        est = self.window_estimate()
        if est is None:
            self.get_logger().info(
                f"Locked, building the window pose: {self.pose_summary()}",
                throttle_duration_sec=1.0)
            return

        if not self.hold_xy:
            self.get_logger().warning(
                "Window pose is ready but the vision estimate is not healthy "
                "enough to fly to a point. Waiting.", throttle_duration_sec=2.0)
            return

        self._begin_aim(est)

    # ---------------------------------------------------------------- AIM

    def _begin_aim(self, est):
        _, _, heading = self.approach_points(est)
        self._enter_stage(self.AIM)
        self.align_in_band_since = None
        self.get_logger().warning(
            f"AIM: turning to {math.degrees(heading):+.0f} deg to face the "
            f"window square-on, holding position. {self.pose_summary()}.")

    def _handle_aim(self):
        """Yaw onto the window normal while standing still.

        Doing the turn before the translation, rather than during it, is the
        one concession this whole approach makes to the camera: an IMU-less
        stereo camera loses tracking on a fast yaw, and it loses it much more
        readily when the scene is also translating. After this the remaining
        yaw corrections are a few degrees and can be flown alongside the
        approach without noticing.
        """
        if not self._still_flyable():
            return

        self._try_latch_xy_hold()
        self.log_flight_state()

        est = self.window_estimate()
        if est is None:
            if self._give_up_on_pose('AIM'):
                return
            self.get_logger().info(
                f"AIM: waiting for the window pose. {self.pose_summary()}",
                throttle_duration_sec=1.0)
            return

        _, _, heading = self.approach_points(est)
        self._aim_yaw_at(heading)

        error = self._heading_error(heading)
        if error <= self.AIM_YAW_TOLERANCE and abs(self.yaw_remaining) < math.radians(2.0):
            self._begin_align(est)
            return

        if self._in_stage_for() > self.AIM_TIMEOUT:
            self.get_logger().warning(
                f"AIM timed out {math.degrees(error):.0f} deg short. Starting "
                "the approach anyway -- the alignment gate still has to pass "
                "before anything is flown through.")
            self._begin_align(est)
            return

        self.get_logger().info(
            f"AIM: {math.degrees(error):.0f} deg to turn. {self.pose_summary()}",
            throttle_duration_sec=1.0)

    # -------------------------------------------------------------- ALIGN

    def _begin_align(self, est):
        entry, _, heading = self.approach_points(est)
        self._enter_stage(self.ALIGN)
        self.align_in_band_since = None
        self.MOVE_SPEED = self.APPROACH_SPEED
        # Set the target HERE rather than leaving it to the first tick of the
        # stage. A single-frame pose dropout on that tick would otherwise leave
        # move_target_x as None while the stage is already running, and every
        # distance-to-target in ALIGN would be arithmetic on it.
        self._set_target(entry[0], entry[1], self.window_altitude(est))
        self.get_logger().warning(
            f"ALIGN: flying to ({entry[0]:+.2f}, {entry[1]:+.2f}) NED, "
            f"{self.STANDOFF_DISTANCE:.2f} m in front of the window on its "
            f"axis, at {self.APPROACH_SPEED:.2f} m/s. {self.pose_summary()}.")

    def _handle_align(self):
        """Fly to the standoff point, re-deriving it from the live estimate.

        The target is recomputed every tick. The carrot the base class walks
        towards it does the smoothing: the setpoint moves at MOVE_SPEED and is
        leashed to the measured position, so an estimate that shifts 10 cm
        does not produce a 10 cm step in what PX4 is asked for -- it produces a
        slightly different direction of travel for the next tick.

        The gate into TRAVERSE is three simultaneous conditions held for
        align_settle_seconds: inside align_tolerance of the standoff point,
        within align_alt_tolerance of the traverse altitude, and within
        align_yaw_tolerance of the window normal. Position alone is not
        alignment -- being in the right place pointing 20 degrees off puts the
        aircraft through the frame sideways.
        """
        if not self._still_flyable():
            return

        self._try_latch_xy_hold()
        self.log_flight_state()

        est = self.window_estimate()
        if est is None:
            if self._give_up_on_pose('ALIGN'):
                return
            # Keep the last target rather than stopping dead: a one-second
            # dropout in the middle of an approach is normal, and freezing the
            # carrot on every one of them makes the approach jerky.
            self.get_logger().info(
                f"ALIGN: pose stale, holding the last target. {self.pose_summary()}",
                throttle_duration_sec=1.0)
        else:
            entry, _, heading = self.approach_points(est)
            altitude = self.window_altitude(est)
            self._set_target(entry[0], entry[1], altitude)
            self._aim_yaw_at(heading)

        if not self.hold_xy:
            # No lateral estimate -> the inherited setpoint publisher has
            # already dropped back to "do not translate". Stop walking a
            # carrot the vehicle is not chasing.
            self.moving = False
            self.align_in_band_since = None
            self.get_logger().warning(
                "ALIGN: vision unhealthy, holding still until it comes back.",
                throttle_duration_sec=2.0)
            if self._in_stage_for() > self.ALIGN_TIMEOUT:
                self._abandon("vision never recovered during the approach")
            return

        if self._aligned():
            if self.align_in_band_since is None:
                self.align_in_band_since = time.monotonic()
            elif time.monotonic() - self.align_in_band_since >= self.ALIGN_SETTLE_SECONDS:
                self._begin_traverse()
            return

        self.align_in_band_since = None

        if self._in_stage_for() > self.ALIGN_TIMEOUT:
            self._abandon(
                f"could not settle on the approach point in "
                f"{self.ALIGN_TIMEOUT:.0f} s")
            return

        lp = self.local_position
        remaining = math.hypot(self.move_target_x - lp.x, self.move_target_y - lp.y)
        alt = self.relative_altitude()
        self.get_logger().info(
            f"ALIGN: {remaining:.2f} m to the approach point, "
            f"{math.degrees(self._heading_error(self._target_heading())):.0f} deg "
            f"off the normal, alt {'n/a' if alt is None else f'{alt:+.2f}'}/"
            f"{self.commanded_altitude:.2f} m. {self.pose_summary()}",
            throttle_duration_sec=1.0)

    def _target_heading(self):
        est = self.window_estimate()
        if est is not None:
            return self.approach_points(est)[2]
        return self.yaw_setpoint

    def _aligned(self):
        """All three axes of "lined up", simultaneously."""
        lp = self.local_position
        if lp is None or self.move_target_x is None:
            return False
        if math.hypot(self.move_target_x - lp.x,
                      self.move_target_y - lp.y) > self.ALIGN_TOLERANCE:
            return False
        alt = self.relative_altitude()
        if alt is None or abs(alt - self.commanded_altitude) > self.ALIGN_ALT_TOLERANCE:
            return False
        return self._heading_error(self._target_heading()) <= self.ALIGN_YAW_TOLERANCE

    # ----------------------------------------------------------- TRAVERSE

    def _begin_traverse(self):
        """Commit: freeze the target and stop listening to the camera.

        Everything about the run through the window is decided here, from the
        one place in the flight where the geometry is best known: stationary,
        square to the aperture, standoff_distance away with the whole window in
        frame. Nothing measured after this point can improve on that, and the
        things that could be measured -- a frame filling the field of view,
        depth on an image edge -- are the camera's worst cases.
        """
        est = self.window_estimate()
        if est is None:
            # Only reachable if the estimate died in the settle window. The
            # aircraft is in the right place pointing the right way; use the
            # geometry it settled onto.
            heading = self.yaw_setpoint
            entry = np.array([self.move_target_x, self.move_target_y, 0.0])
            exit_point = entry + np.array([
                math.cos(heading) * (self.STANDOFF_DISTANCE + self.EXIT_DISTANCE),
                math.sin(heading) * (self.STANDOFF_DISTANCE + self.EXIT_DISTANCE),
                0.0])
            self.get_logger().warning(
                "Committing to the traverse on the settled heading: the pose "
                "estimate went stale during the alignment settle.")
        else:
            entry, exit_point, heading = self.approach_points(est)
            self.traverse_window = est

        self.traverse_entry = entry
        self.traverse_exit = exit_point
        self.traverse_heading = heading
        self.blind_traverse_since = None
        self.MOVE_SPEED = self.TRAVERSE_SPEED
        self._set_target(exit_point[0], exit_point[1])
        self.yaw_remaining = wrap_pi(heading - self.yaw_setpoint)
        self._enter_stage(self.TRAVERSE)

        size = ('unknown size' if self.traverse_window is None
                else f"{self.traverse_window['width']:.2f}x"
                     f"{self.traverse_window['height']:.2f} m")
        self.get_logger().warning(
            f"TRAVERSE: committed. Flying through to ({exit_point[0]:+.2f}, "
            f"{exit_point[1]:+.2f}) NED at {self.TRAVERSE_SPEED:.2f} m/s on a "
            f"heading of {math.degrees(heading):+.0f} deg, altitude "
            f"{self.commanded_altitude:.2f} m. Window {size}. The camera is "
            "no longer steering: this target is frozen.")

    def _handle_traverse(self):
        if not self._still_flyable():
            return

        # This is what notices vision dying mid-run, and it is safe to call
        # here: the latching branch only fires when hold_xy is already False,
        # so it cannot disturb the carrot while the run is going well. When it
        # does fire -- vision coming back after a blind push -- re-anchoring on
        # the current position is exactly what should happen, and the carrot
        # then walks on to the frozen exit point from there.
        self._try_latch_xy_hold()
        self.log_flight_state()

        if not self.hold_xy:
            self._handle_blind_traverse()
            return

        self.blind_traverse_since = None
        self.yaw_remaining = wrap_pi(self.traverse_heading - self.yaw_setpoint)

        # Progress is measured ALONG the traverse line, not as a distance to
        # the exit point: a metre of crosstrack error would otherwise read as
        # "not there yet" forever and burn the timeout.
        along = self._distance_along_traverse()
        total = self.STANDOFF_DISTANCE + self.EXIT_DISTANCE
        if along >= total - self.ALIGN_TOLERANCE:
            self._begin_clear(f"through, {along:.2f} m flown of {total:.2f} m")
            return

        if self._in_stage_for() > self.TRAVERSE_TIMEOUT:
            self._abandon(
                f"traverse timed out {total - along:.2f} m short of the far side")
            return

        self.get_logger().info(
            f"TRAVERSE: {along:.2f}/{total:.2f} m, "
            f"{math.degrees(self._heading_error(self.traverse_heading)):.0f} deg "
            f"off the committed heading.", throttle_duration_sec=0.5)

    def _distance_along_traverse(self):
        """How far down the committed line the vehicle is, from the entry."""
        lp = self.local_position
        if lp is None or self.traverse_entry is None:
            return 0.0
        direction = np.array([math.cos(self.traverse_heading),
                              math.sin(self.traverse_heading)])
        offset = np.array([lp.x - self.traverse_entry[0],
                           lp.y - self.traverse_entry[1]])
        return float(np.dot(offset, direction))

    def _handle_blind_traverse(self):
        """Vision died mid-run: push on for a few seconds, then land.

        A position setpoint against a dead lateral estimate is a setpoint
        against a number that means nothing, so the inherited publisher has
        already stopped sending one. What replaces it here is an open-loop
        velocity along the committed heading -- open loop is exactly what the
        base class refuses to do anywhere else, and it is right to, but the
        alternative in this one place is stopping inside an aperture with no
        way to tell which side of it the aircraft is on.

        Bounded hard: blind_traverse_seconds of it, then a landing. It is a way
        out of the frame, not a way to complete the mission.
        """
        now = time.monotonic()
        if self.blind_traverse_since is None:
            self.blind_traverse_since = now
            self.get_logger().error(
                "TRAVERSE: vision lost mid-run. Pushing on open-loop along the "
                f"committed heading for up to {self.BLIND_TRAVERSE_SECONDS:.1f} s "
                "rather than stopping in the window.")

        elapsed = now - self.blind_traverse_since
        if elapsed >= self.BLIND_TRAVERSE_SECONDS:
            along = self._distance_along_traverse()
            self.outcome = (
                f"BLIND: vision lost mid-traverse, pushed on for "
                f"{self.BLIND_TRAVERSE_SECONDS:.1f} s and reached {along:.2f} m of "
                f"{self.STANDOFF_DISTANCE + self.EXIT_DISTANCE:.2f} m")
            self._begin_landing(
                f"vision did not come back within {self.BLIND_TRAVERSE_SECONDS:.1f} s "
                "of the blind push")
            return

        self.get_logger().warning(
            f"TRAVERSE: blind, {self.BLIND_TRAVERSE_SECONDS - elapsed:.1f} s left.",
            throttle_duration_sec=0.5)

    def _blind_traverse_active(self):
        return (self.current_stage == self.TRAVERSE
                and self.blind_traverse_since is not None
                and not self.hold_xy)

    # -------------------------------------------------------------- CLEAR

    def _begin_clear(self, reason):
        self.outcome = reason
        self.moving = False
        self.MOVE_SPEED = self.APPROACH_SPEED
        lp = self.local_position
        if lp is not None and self.hold_xy:
            self.hold_x = lp.x
            self.hold_y = lp.y
        self._enter_stage(self.CLEAR)
        self.get_logger().warning(
            f"WINDOW TRAVERSED ({reason}). Holding {self.CLEAR_SECONDS:.0f} s "
            "on the far side, then landing.")

    def _handle_clear(self):
        if not self._still_flyable():
            return

        self._try_latch_xy_hold()
        self.log_flight_state()

        remaining = self.CLEAR_SECONDS - self._in_stage_for()
        if remaining <= 0.0:
            self._begin_landing("traverse complete")
            return

        self.get_logger().info(
            f"CLEAR: holding, {remaining:.1f} s to the descent.",
            throttle_duration_sec=1.0)

    # ------------------------------------------------------------ giving up

    def _give_up_on_pose(self, stage):
        """True if the estimate has been gone long enough to abandon the run."""
        if self.pose_lost_since is None:
            return False
        if time.monotonic() - self.pose_lost_since < self.POSE_LOST_TIMEOUT:
            return False
        self._abandon(
            f"no usable window pose for {self.POSE_LOST_TIMEOUT:.0f} s during {stage}")
        return True

    def _abandon(self, reason):
        """Stop the attempt and land from wherever we are.

        Not a retry. Going back to SCAN after a failed approach means a vehicle
        that is now somewhere other than where it swept from, with an unknown
        amount of battery left, starting the same attempt with the same
        conditions that just failed. Land, look at the log, fly it again.
        """
        self.outcome = f"ABANDONED: {reason}"
        self.moving = False
        self.yaw_remaining = 0.0
        self.MOVE_SPEED = self.APPROACH_SPEED
        self.get_logger().error(f"Traverse abandoned: {reason}.")
        self._begin_landing(f"traverse abandoned -- {reason}")

    # ------------------------------------------------------------ publishers

    def publish_position_setpoint(self):
        """The inherited setpoint, except during a blind traverse.

        Everything about this node except those few seconds publishes an
        absolute NED point through the base class. The blind push is the one
        exception, and it is written out here rather than bolted into the base
        class because it is a behaviour that only makes sense inside a window.
        """
        if not self._blind_traverse_active() or self.home_z is None:
            super().publish_position_setpoint()
            return

        nan = float('nan')
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # Altitude stays a position setpoint on the ramp: the height estimate
        # is the lidar's and is unaffected by whatever happened to vision.
        self._step_setpoint_ramp()
        self._step_yaw_ramp()

        speed = self.TRAVERSE_SPEED
        msg.position = [nan, nan, self.setpoint_z]
        msg.velocity = [speed * math.cos(self.traverse_heading),
                        speed * math.sin(self.traverse_heading), nan]
        msg.yaw = self.yaw_setpoint
        self.trajectory_setpoint_pub.publish(msg)

    def publish_status(self):
        """stage|armed|altitude|xy|detail, the format the LCD node reads."""
        if self.current_stage not in self.TRAVERSE_STAGES:
            super().publish_status()
            return

        alt = self.relative_altitude()
        armed = self.arming_state == VehicleStatus.ARMING_STATE_ARMED
        lp = self.local_position

        if self.current_stage == self.AIM:
            detail = f"aim{math.degrees(abs(self.yaw_remaining)):.0f}"
        elif self.current_stage == self.ALIGN:
            left = (0.0 if lp is None or self.move_target_x is None
                    else math.hypot(self.move_target_x - lp.x,
                                    self.move_target_y - lp.y))
            detail = f"algn{left:.2f}"
        elif self.current_stage == self.TRAVERSE:
            total = self.STANDOFF_DISTANCE + self.EXIT_DISTANCE
            detail = f"thru{self._distance_along_traverse():.1f}/{total:.1f}"
        else:
            detail = f"{max(0.0, self.CLEAR_SECONDS - self._in_stage_for()):.0f}s"

        msg = String()
        msg.data = "|".join([
            self.current_stage,
            'ARM' if armed else 'DIS',
            f"{alt:.2f}" if alt is not None else 'nan',
            'POS' if self.hold_xy else ('VIO' if self.flow_is_healthy() else '---'),
            detail,
        ])
        self.status_pub.publish(msg)

    # --------------------------------------------------------------- logging

    def log_flight_state(self):
        super().log_flight_state()
        self.get_logger().info(
            f"window: {self.pose_summary()}", throttle_duration_sec=2.0)

    def destroy_node(self):
        self.get_logger().warning(
            f"Traverse outcome: {self.outcome}. "
            f"{self.estimator.accepted_total} geometry samples accepted of "
            f"{self.geometry_seen} received; rejections: "
            f"{self.estimator.rejection_summary(limit=5)}.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WindowTraverse()
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
