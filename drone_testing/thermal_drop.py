#!/usr/bin/env python3
"""
Find the hottest of three boxes with a downward MLX90640, put the DROP POINT
over it, descend to drop height and hover there. ARK Flow localisation.

    arm -> ground wait -> climb to survey_altitude -> hold -> SURVEY (map every
    warm blob in NED, from the centre and, if needed, a small ring of points)
    -> pick the hottest -> APPROACH (fly the drop point over it, and confirm it
    is still the hottest thing in the frame) -> DESCEND (step down only while
    centred, LED BLINKING RED the whole way) -> HOVER (confirm alignment at
    drop_altitude, RELEASE the servo, LED solid red, then off) -> RETREAT (climb back to
    retreat_altitude, step retreat_right to the right) -> land.

    q -> abort into a controlled descent.   k -> force-disarm.

THREE MODES -- WALK UP THE LADDER

    mode:=bench     Nothing armed, nothing published to PX4. Logs where the
                    hottest blob is in vehicle terms. This is the axis sign
                    check; run it first, props off.

    mode:=dryrun    THE WHOLE MISSION ON A SIMULATED VEHICLE. No arming, no
                    Offboard request, no heartbeat, no setpoint -- the motors
                    cannot be commanded. Everything else is real: the camera,
                    the blob detection, the survey, the hottest-box check, the
                    LED blinking red through the descent, and the SERVO
                    firing at the simulated drop height.

                    The simulated vehicle cannot change what the camera sees,
                    so the error it measures is just where the hot object is
                    relative to the drop point at the simulated height: YOU
                    close the loop by moving the hot object (or the airframe)
                    until the green box on the :8082 stream sits on the
                    crosshair. It then descends, blinks and drops exactly as
                    it would in the air. Props off for this too -- it is a
                    rehearsal, not a proof the aircraft is safe.

    mode:=fly       The real thing.

MAKING SURE IT IS THE HOTTEST BOX, NOT THE HOTTEST PIXEL
    Three independent filters, because a single noisy pixel reading 20 C high
    is the failure that puts the payload on the wrong box:

      1. min_blob_pixels -- a blob of one pixel is discarded outright.
      2. min_cluster_frames -- a survey cluster only counts once it has been
         seen in that many frames, and it is scored on the 90th percentile of
         its peak, not on its single best reading.
      3. the hottest-in-frame check -- APPROACH will not hand over to DESCEND
         until the tracked box has been the hottest blob in the frame for
         verify_ratio of the last verify_window seconds. If something else is
         hotter by retarget_margin, consistently and in a fixed place, the
         aircraft RETARGETS onto it instead.

THE SERVO, AND WHY A DRY RUN MAY NOT MOVE IT
    Two commands, and they are not interchangeable:

      servo_command:=set_actuator (default)
          MAV_CMD_DO_SET_ACTUATOR. The one for FLIGHT. It needs the output
          assigned to "Offboard Actuator Set <servo_index>" in QGC's
          Actuators tab -- an output left on "RC AUX 1" is an RC passthrough
          and ignores it. PX4 applies offboard actuator values to the outputs
          only while ARMED, so on a disarmed bench this can be accepted and
          still move nothing.

      servo_command:=actuator_test
          MAV_CMD_ACTUATOR_TEST, which is what the QGC Actuators sliders
          send, and the one that works DISARMED. It addresses the output by
          PX4 FUNCTION number, so servo_function must be set too (Servo 1-8
          are 201-208).

    So: actuator_test for dry runs on the bench, set_actuator in the air --
    unless the bench proves otherwise.

    PX4's verdict on either is logged from /fmu/out/vehicle_command_ack, and
    `ros2 run drone_testing servo_test` exercises both on their own.
    release_enabled:=false flies the whole mission and logs the release
    instead of commanding it.

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
from px4_msgs.msg import (VehicleAttitude, VehicleCommand, VehicleCommandAck,
                          VehicleStatus)
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from drone_testing.offboard_sequence import OffboardSequence, spin_node
from drone_testing.thermal_common import H, W, find_blobs


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


class _SimPose:
    """Everything the flight code reads off VehicleLocalPosition, simulated.

    Used only by mode:=dryrun. It is a stand-in for the estimator, not for the
    aircraft: horizontal position integrates towards whatever the mission
    commands, altitude follows the commanded altitude, and that is all.
    """

    def __init__(self, x, y, alt, heading=0.0, vz=0.0):
        self.x, self.y, self.z = float(x), float(y), -float(alt)
        self.vx = self.vy = 0.0
        self.vz = float(vz)
        self.heading = float(heading)
        self.xy_valid = self.z_valid = True
        self.v_xy_valid = self.v_z_valid = True
        self.dist_bottom = float(alt)
        self.dist_bottom_valid = True
        self.xy_reset_counter = 0
        self.z_reset_counter = 0
        self.heading_reset_counter = 0
        self.delta_xy = [0.0, 0.0]
        self.delta_z = 0.0
        self.delta_heading = 0.0


class ThermalDrop(OffboardSequence):

    BENCH = "BENCH"
    SURVEY = "SURVEY"
    APPROACH = "APPROACH"
    DESCEND = "DESCEND"
    HOVER = "HOVER"
    RETREAT = "RETREAT"
    DROP_STAGES = (SURVEY, APPROACH, DESCEND, HOVER, RETREAT)

    MIN_DROP_ALTITUDE = 0.50    # m. Hard floor, not a parameter.
    MAX_SURVEY_ALTITUDE = 2.60  # m. The clamp on survey_altitude.
                                #
                                # It was 1.60 when the survey was a RING
                                # flown 1.5 m up: low and close, five points,
                                # dwelling at each. That mission is gone. The
                                # survey is now a single observation from
                                # 2.50 m -- high enough that all three boxes
                                # are in one frame and no pattern has to be
                                # flown to find them -- so the clamp has to
                                # clear 2.50 m or it would silently undo the
                                # thing it is there to protect.
                                #
                                # 2.50 m IS NEAR THE MLX90640's LIMIT. At
                                # that height a 30 cm box is about 4 pixels
                                # across in a 32x24 frame, so min_blob_pixels
                                # and min_contrast are doing real work; if
                                # the boxes come back as one blob or as
                                # nothing, the survey is too high, not too
                                # fussy. Lower survey_altitude before
                                # loosening either filter.

    def __init__(self):
        super().__init__('thermal_drop')
        num = self._declare_number

        self.MODE = str(self.declare_parameter('mode', 'fly').value).strip().lower()
        if self.MODE not in ('bench', 'dryrun', 'fly'):
            self.get_logger().error(
                f"Unknown mode '{self.MODE}'; expected bench | dryrun | fly. "
                "Using bench, which commands nothing.")
            self.MODE = 'bench'

        # ---- the sensor ----
        self.HFOV = math.radians(float(num('hfov_deg', 110.0)))
        self.VFOV = math.radians(float(num('vfov_deg', 75.0)))
        self.CAM_YAW = math.radians(float(num('cam_yaw_deg', 0.0)))
        self.FLIP_LR = bool(self.declare_parameter('flip_lr', False).value)
        self.FLIP_UD = bool(self.declare_parameter('flip_ud', False).value)
        self.MIN_CONTRAST = float(num('min_contrast', 3.0))
        self.MIN_BLOB_PIXELS = int(num('min_blob_pixels', 2))
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
        # MAX_SURVEY_ALTITUDE is the height above which the real MLX90640's
        # readings stop being usable, so on the aircraft it is a hard ceiling
        # and the default below is the only value that should ever be used.
        # It is a PARAMETER because a scaled simulation is a different sensor
        # over a different-sized arena: imav2026_scaled is the real course
        # multiplied by 2.2, so every box is 2.2x further away and the survey
        # has to be flown 2.2x higher to see the same thing. Raising it for
        # HARDWARE does not buy a wider survey, it buys a blurrier one.
        self.MAX_SURVEY_ALTITUDE = float(num('max_survey_altitude',
                                             self.MAX_SURVEY_ALTITUDE))
        survey_alt = float(num('survey_altitude', 2.5))
        if survey_alt > self.MAX_SURVEY_ALTITUDE:
            self.get_logger().error(
                f"survey_altitude {survey_alt:.2f} m is above the "
                f"{self.MAX_SURVEY_ALTITUDE:.2f} m the MLX is usable from; clamping.")
            survey_alt = self.MAX_SURVEY_ALTITUDE
        self.SURVEY_ALTITUDE = survey_alt

        # ---- THE CRUISE HEIGHT, AND WHY IT IS NOT THE SURVEY HEIGHT ----
        #
        # The aircraft used to take off straight to survey_altitude and fly
        # the whole outbound mission there -- the marker creep, the hover on
        # the marker, the sidestep -- because that was the height the survey
        # needed and nothing had asked for another one.
        #
        # It is the wrong height for all three of those. Everything before
        # the survey is FINDING A MARKER ON THE FLOOR with a downward
        # camera, and low is better for that in every way that matters: the
        # marker is more pixels across, so the solver's pose is better; the
        # camera's footprint is smaller, so a marker in frame is a marker
        # nearly underneath rather than one 3 m off to the side; and the
        # aircraft is not carrying a cone around the arena at head height.
        # The survey is the ONE part that wants altitude, because it wants
        # all three boxes in one thermal frame, and it is also the one part
        # that is flown stationary -- so it can simply climb when it gets
        # there and pay the four seconds once.
        #
        # So: take off to cruise_altitude, fly everything up to and
        # including BOX_OFFSET there, climb to survey_altitude over the
        # boxes, and descend to drop_altitude from there. _begin_survey()
        # commands the climb and _handle_survey() waits for it.
        cruise_alt = float(num('cruise_altitude', 1.2))
        if cruise_alt < self.MIN_DROP_ALTITUDE:
            self.get_logger().error(
                f"cruise_altitude {cruise_alt:.2f} m is below the "
                f"{self.MIN_DROP_ALTITUDE:.2f} m floor; using the floor.")
            cruise_alt = self.MIN_DROP_ALTITUDE
        if cruise_alt > self.SURVEY_ALTITUDE:
            # Not fatal, just pointless: the survey would then be a DESCENT
            # and the outbound legs would be flown at the higher height,
            # which is the arrangement this parameter exists to undo.
            self.get_logger().warning(
                f"cruise_altitude {cruise_alt:.2f} m is above "
                f"survey_altitude {self.SURVEY_ALTITUDE:.2f} m, so the "
                "outbound legs are the HIGHEST part of the flight and the "
                "survey climb is a descent. That is backwards; check both.")
        self.CRUISE_ALTITUDE = cruise_alt
        self.TAKEOFF_ALTITUDE = cruise_alt
        self.commanded_altitude = cruise_alt

        drop_alt = float(num('drop_altitude', self.MIN_DROP_ALTITUDE))
        if drop_alt < self.MIN_DROP_ALTITUDE:
            self.get_logger().error(
                f"drop_altitude {drop_alt:.2f} m is below the "
                f"{self.MIN_DROP_ALTITUDE:.2f} m floor; using the floor.")
            drop_alt = self.MIN_DROP_ALTITUDE
        self.DROP_ALTITUDE = drop_alt

        self.SURVEY_DWELL = float(num('survey_dwell_seconds', 4.0))
        # 0.0 = NO SEARCH PATTERN. The survey is one observation, taken from
        # where BOX_OFFSET left the aircraft, at survey_altitude. See
        # _begin_survey() for why the ring went away. A positive value puts
        # the old four-point ring back, at that step, for an arena where the
        # boxes genuinely do not fit in one frame.
        self.SURVEY_STEP = float(num('survey_step', 0.0))
        self.SURVEY_ALL_POINTS = bool(self.declare_parameter('survey_all_points', False).value)
        self.SURVEY_MOVE_TIMEOUT = 15.0

        self.APPROACH_TOLERANCE = float(num('approach_tolerance', 0.15))
        self.DESCEND_TOLERANCE = float(num('descend_tolerance', 0.12))
        self.DESCEND_SPEED = float(num('descend_speed', 0.12))
        self.ALIGN_TOLERANCE = float(num('align_tolerance', 0.08))
        self.ALIGN_SETTLE = float(num('align_settle_seconds', 2.0))
        self.HOVER_SECONDS = float(num('hover_seconds', 2.0))
        self.LAND_AFTER_DROP = bool(self.declare_parameter('land_after_drop', True).value)
        self.STAGE_TIMEOUT = float(num('stage_timeout', 45.0))

        # ---- confirming it really is THE hot box ----
        self.VERIFY_FRAMES = int(num('verify_frames', 5))
        self.VERIFY_WINDOW = float(num('verify_window', 4.0))
        self.VERIFY_RATIO = float(num('verify_ratio', 0.7))
        self.RETARGET_MARGIN = float(num('retarget_margin', 1.5))

        # ---- the drop, and what happens after it ----
        self.SERVO_INDEX = int(num('servo_index', 1))
        self.SERVO_DROP_VALUE = float(num('servo_drop_value', 1.0))
        self.SERVO_NEUTRAL_VALUE = float(num('servo_neutral_value', -1.0))
        self.SERVO_HOLD_SECONDS = float(num('servo_hold_seconds', 2.0))
        self.RELEASE_ENABLED = bool(self.declare_parameter('release_enabled', True).value)
        self.SERVO_COMMAND = str(self.declare_parameter(
            'servo_command', 'set_actuator').value).strip().lower()
        if self.SERVO_COMMAND not in ('set_actuator', 'actuator_test'):
            self.get_logger().error(
                f"servo_command '{self.SERVO_COMMAND}' is not set_actuator or "
                "actuator_test; using set_actuator.")
            self.SERVO_COMMAND = 'set_actuator'
        self.SERVO_FUNCTION = int(num('servo_function', 0))
        if self.SERVO_COMMAND == 'actuator_test' and self.SERVO_FUNCTION <= 0:
            self.get_logger().error(
                "servo_command is actuator_test but servo_function is 0. That "
                "is the PX4 OUTPUT FUNCTION number of the servo (the same "
                "function QGC's Actuators tab shows for that output), not the "
                "offboard set index. Nothing will move until it is set.")
        self.SERVO_TEST_ON_START = bool(self.declare_parameter(
            'servo_test_on_start', False).value)
        # WHO ACTUALLY MOVES THE SERVO.
        #
        #   false: this node does, with MAV_CMD_DO_SET_ACTUATOR, exactly as
        #   it always has. That is what the SITL runs use -- there is no
        #   servo in Gazebo, the command is the only evidence there is, and
        #   an extra process to produce it would prove nothing.
        #
        #   true: servo_controller.py does. This node still decides WHEN --
        #   it publishes True on drop_trigger_topic the instant HOVER
        #   commits, and False when the hold is over -- and stops sending
        #   actuator commands itself, so the two are never fighting for the
        #   same output. That is the hardware arrangement: the release lives
        #   in one small node that can be run, watched and bench-tested on
        #   its own, and this node's job ends at "now".
        #
        #   The node must be RUNNING. If it is not, nothing opens: the ack
        #   watchdog below cannot see that, because there is no command of
        #   ours to be acked, so the log says so at release time instead.
        self.RELEASE_VIA_NODE = bool(self.declare_parameter(
            'release_via_servo_node', False).value)
        self.DROP_TRIGGER_TOPIC = str(self.declare_parameter(
            'drop_trigger_topic', '/servo/drop').value)
        self.SIM_DESCEND_SPEED = float(num('sim_descend_speed', 0.25))
        self.RETREAT_ALTITUDE = float(num('retreat_altitude', 1.2))
        self.RETREAT_RIGHT = float(num('retreat_right', 0.5))
        self.RETREAT_TIMEOUT = float(num('retreat_timeout', 30.0))

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
        self.hot_hits = collections.deque(maxlen=80)       # (t, target was hottest)
        self.rivals = collections.deque(maxlen=80)         # (t, xy, peak) of a
                                                           # blob that outranks it
        self.chosen = None          # {'xy', 'score'}
        self.frames_seen = 0
        self.verify_count = 0
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

        # PX4's verdict on the servo command. Without this the log can only
        # say "command sent", which is exactly the ambiguity that made a
        # non-moving servo impossible to diagnose from a flight log.
        self.create_subscription(VehicleCommandAck, '/fmu/out/vehicle_command_ack',
                                 self.command_ack_callback, sensor_qos,
                                 callback_group=self.sensor_cbg)
        self.led_pub = self.create_publisher(String, 'led/command', 10)
        self.ready_pub = self.create_publisher(Bool, 'thermal_drop/ready', 10)
        # RELIABLE and depth 10, not the best-effort QoS the sensor streams
        # use: this is a one-shot command, and a dropped one is a cone that
        # stays in the aircraft. servo_controller.py subscribes with the
        # matching profile.
        self.drop_trigger_pub = self.create_publisher(
            Bool, self.DROP_TRIGGER_TOPIC, 10)
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
        self.released_at = None
        self.servo_returned = False
        self.servo_ack_seen = False
        self._last_actuator_send = 0.0
        self.retreat_target = None
        self.led_mode = None
        self.outcome = 'not attempted'
        self._xy_counter = None

        self.sim_xy = np.zeros(2)
        self.sim_alt = self.SURVEY_ALTITUDE
        self._dryrun_done_logged = False

        if self.MODE == 'dryrun':
            self._start_dryrun()
            return

        if self.MODE == 'bench':
            self.stream_setpoints = False
            self.current_stage = self.BENCH
            self.get_logger().warning(
                "BENCH MODE: nothing is armed or published to PX4. Hold a hot "
                "object under the camera and move it to the drone's RIGHT and "
                "FORWARD -- the log must say RIGHT and FORWARD.")
            return

        self.get_logger().warning(
            f"Thermal drop: climb to {self.CRUISE_ALTITUDE:.2f} m, then "
            f"{self.SURVEY_ALTITUDE:.2f} m to survey for "
            f"{self.EXPECTED_BOXES} boxes, fly the drop point over the hottest, "
            f"descend to {self.DROP_ALTITUDE:.2f} m, align to "
            f"{self.ALIGN_TOLERANCE * 100:.0f} cm, hover {self.HOVER_SECONDS:.0f} s, "
            f"release the servo, climb to {self.RETREAT_ALTITUDE:.2f} m, step "
            f"{self.RETREAT_RIGHT:.2f} m right and "
            f"{'land' if self.LAND_AFTER_DROP else 'hold'}. Camera is "
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

    def command_ack_callback(self, msg):
        if msg.command not in (187, 310):
            return
        results = {0: 'ACCEPTED (if the servo did not move, the OUTPUT MAPPING '
                      'is wrong, not the command)',
                   1: 'TEMPORARILY_REJECTED (disarmed is the usual reason: PX4 '
                      'applies offboard actuator values only while ARMED)',
                   2: 'DENIED', 3: 'UNSUPPORTED by this firmware',
                   4: 'FAILED', 5: 'IN_PROGRESS', 6: 'CANCELLED'}
        name = 'DO_SET_ACTUATOR' if msg.command == 187 else 'ACTUATOR_TEST'
        self.servo_ack_seen = True
        level = (self.get_logger().warning if msg.result == 0
                 else self.get_logger().error)
        level(f"PX4 ack for {name}: "
              f"{results.get(msg.result, f'result {msg.result}')}",
              throttle_duration_sec=1.0)

    def local_position_callback(self, msg):
        if self.MODE == 'dryrun':
            # The simulated vehicle IS the vehicle here. If PX4 happens to be
            # connected, its real estimate -- sitting on the floor with
            # xy_valid false -- would otherwise overwrite the simulated pose
            # between ticks and every frame would be discarded as unlocalised.
            return
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
        blobs, ambient = find_blobs(grid.astype(float), self.MIN_CONTRAST,
                                    min_pixels=self.MIN_BLOB_PIXELS)
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
                if not near and blobs:
                    # The box we are tracking is not among this frame's blobs.
                    # That is evidence against it, and counting it is what
                    # stops the confidence sitting at "building" for ever.
                    self.hot_hits.append((now, False))
                if near:
                    xy, b = max(near, key=lambda f: f[1]['peak'])
                    self.track.append((now, xy, b['peak']))
                    # Is the box we are tracking still the hottest thing in the
                    # frame? One frame proves nothing -- a pixel of noise on a
                    # radiator, a person, a sunlit patch -- so this is counted
                    # over a window and read by APPROACH before it commits.
                    top = blobs[0]
                    is_hottest = top['peak'] <= b['peak'] + self.RETARGET_MARGIN
                    self.hot_hits.append((now, is_hottest))
                    if not is_hottest:
                        rival_xy = self.project(top, q, p_ned)
                        if rival_xy is not None:
                            self.rivals.append((now, rival_xy, top['peak']))

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

    def _hot_confidence(self):
        """Fraction of recent frames in which the target was the hottest blob.

        None until there is enough evidence to answer, which is what stops the
        descent starting on a single lucky frame.
        """
        now = time.monotonic()
        with self._lock:
            recent = [ok for t, ok in self.hot_hits if now - t <= self.VERIFY_WINDOW]
        self.verify_count = len(recent)
        if len(recent) < self.VERIFY_FRAMES:
            return None
        return sum(1 for ok in recent if ok) / len(recent)

    def _rival_target(self):
        """A consistently hotter blob somewhere else, or None.

        It has to be hotter by retarget_margin, in at least verify_frames
        frames, and in the SAME place each time -- a wandering rival is noise,
        and noise must not be able to drag the aircraft off the real box.
        """
        now = time.monotonic()
        with self._lock:
            fresh = [xy for t, xy, _ in self.rivals if now - t <= self.VERIFY_WINDOW]
        if len(fresh) < self.VERIFY_FRAMES:
            return None
        arr = np.array(fresh)
        med = np.median(arr, axis=0)
        if float(np.max(np.linalg.norm(arr - med, axis=1))) > 2.0 * self.CLUSTER_RADIUS:
            return None
        return med

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
        if self.MODE == 'dryrun':
            self._dryrun_tick()
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

        self._stage_handlers()[self.current_stage]()

    def _stage_handlers(self):
        """Stage name -> the method that flies it.

        A dict and not a chain of ifs so a subclass can add its own stages by
        extending it, which is how thermal_fsm.py bolts the marker hunt on
        the front of this mission and the precision landing on the back
        without reimplementing any of the middle.
        """
        return {self.SURVEY: self._handle_survey,
                self.APPROACH: self._handle_approach,
                self.DESCEND: self._handle_descend,
                self.HOVER: self._handle_hover,
                self.RETREAT: self._handle_retreat}

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
        self._begin_survey()

    def _begin_survey(self, centre=None):
        """Watch the boxes from ONE point and take the hottest.

        Split out of _handle_hold because the survey does not always follow
        the post-takeoff hover: thermal_fsm.py flies to a marker and steps
        sideways onto the boxes first, and then starts the survey from
        WHEREVER that left the aircraft. centre defaults to the current
        position hold, which is what the plain mission wants.

        THERE IS NO SEARCH PATTERN ANY MORE.

            This used to fly a five-point ring -- centre, then one step
            forward, right, back and left -- dwelling survey_dwell_seconds
            at each and moving on until it had mapped expected_boxes. The
            ring existed because the survey was flown at 1.50 m, where the
            MLX90640's footprint is about 4.3 m by 2.6 m and three boxes
            spread over 2.9 m of arena do not reliably all fall inside it.

            Going UP fixes that outright instead of flying around it. At
            survey_altitude 2.50 m the same footprint is 7.1 m by 3.8 m and
            all three boxes are in one frame, so the aircraft can simply
            stop, watch, and rank them. That removes four moves and four
            dwells -- most of a minute of flight time -- and, more
            importantly, it removes the failure they brought with them: each
            step is flown on flow-held position, every one of them drifts a
            little, and the map the ring builds is a map assembled from five
            slightly different guesses about where the aircraft was.

            One point, one frame of reference, one decision.

            survey_step > 0 puts the ring back for an arena where the boxes
            really do not fit in a frame.
        """
        lp = self.local_position
        if centre is None:
            centre = np.array([self.hold_x, self.hold_y])
        self.survey_centre = np.asarray(centre, dtype=float)
        self.survey_points = [self.survey_centre]
        if self.SURVEY_STEP > 0.0:
            c, s = math.cos(self.home_yaw), math.sin(self.home_yaw)
            d = self.SURVEY_STEP
            self.survey_points += [
                self.survey_centre + np.array([f * c - r * s, f * s + r * c])
                for f, r in ((d, 0.0), (0.0, d), (-d, 0.0), (0.0, -d))]
        self.survey_index = 0
        self.survey_arrived_since = None
        # The climb happens HERE, not at takeoff. Nothing is collected until
        # it is finished: a map built on the way up is a map of blobs seen
        # from heights the decision was not made at, and the whole point of
        # one observation is that it is ONE observation.
        self.survey_at_altitude = False
        self._set_altitude(self.SURVEY_ALTITUDE)
        with self._lock:
            self.clusters = []
            self.collecting = False
        self._enter_stage(self.SURVEY)
        self.get_logger().warning(
            f"SURVEY from ({lp.x:+.2f}, {lp.y:+.2f}): climbing "
            f"{self.relative_altitude() or 0.0:.2f} -> "
            f"{self.SURVEY_ALTITUDE:.2f} m first, then "
            + (f"holding still for {self.SURVEY_DWELL:.1f} s and ranking "
               "whatever is in the frame (no search pattern)"
               if len(self.survey_points) == 1 else
               f"then a {self.SURVEY_STEP:.2f} m ring of "
               f"{len(self.survey_points) - 1} more points")
            + f". {self.frames_seen} thermal frames received so far.")

    # --------------------------------------------------------------- SURVEY

    def _handle_survey(self):
        lp = self.local_position
        point = self.survey_points[self.survey_index]
        self._move_to(point)
        now = time.monotonic()

        # THE CLIMB, FIRST. Hold station over the boxes while it happens --
        # the aircraft is already where it wants to be horizontally, it is
        # only the height that is wrong. The dwell clock and the move
        # timeout are both restarted at the top, so the climb is not charged
        # to either of them, and nothing is collected on the way up.
        if not self.survey_at_altitude:
            self._set_altitude(self.SURVEY_ALTITUDE)
            alt = self.relative_altitude()
            if alt is None:
                return
            # Twice the usual band. The survey height is not a precision
            # requirement -- 16 cm at 2.50 m moves the thermal footprint by
            # about 6% and changes nothing about which box is hottest -- and
            # a tight gate here buys nothing but a chance of sitting in the
            # climb until the timeout below.
            if abs(alt - self.SURVEY_ALTITUDE) > 2.0 * self.ALTITUDE_TOLERANCE:
                if self._in_stage_for() > self.SURVEY_MOVE_TIMEOUT:
                    self.get_logger().error(
                        f"SURVEY: still {alt:.2f} m after "
                        f"{self.SURVEY_MOVE_TIMEOUT:.0f} s of climbing to "
                        f"{self.SURVEY_ALTITUDE:.2f} m. Surveying from here "
                        "instead -- the boxes may not all be in frame, so "
                        "the log below is what it actually saw.")
                else:
                    self.get_logger().info(
                        f"SURVEY: climbing {alt:.2f} -> "
                        f"{self.SURVEY_ALTITUDE:.2f} m before looking.",
                        throttle_duration_sec=1.0)
                    return
            self.survey_at_altitude = True
            self.survey_arrived_since = None
            self._restart_stage_clock()
            with self._lock:
                self.clusters = []
                self.collecting = True
            self.get_logger().warning(
                f"SURVEY: at {alt:.2f} m over the boxes. Holding still for "
                f"{self.SURVEY_DWELL:.1f} s and ranking what is in the frame.")
            return
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

        # Something else is consistently hotter, in a fixed place. Go there
        # instead -- the mission is the HOTTEST box, not the first one found.
        rival = self._rival_target()
        if rival is not None and np.linalg.norm(rival - box) > self.CLUSTER_RADIUS:
            with self._lock:
                self.chosen['xy'] = rival.copy()
                self.track.clear()
                self.hot_hits.clear()
                self.rivals.clear()
            self.in_band_since = None
            self._restart_stage_clock()
            self.get_logger().warning(
                f"RETARGET: a hotter blob sits at ({rival[0]:+.2f}, "
                f"{rival[1]:+.2f}) NED, more than {self.RETARGET_MARGIN:.1f} C "
                "above the one we were tracking. Going there instead.")
            return

        confidence = self._hot_confidence()
        confirmed = confidence is not None and confidence >= self.VERIFY_RATIO

        if err <= self.APPROACH_TOLERANCE and live and confirmed:
            if self.in_band_since is None:
                self.in_band_since = time.monotonic()
            elif time.monotonic() - self.in_band_since >= 1.0:
                self.desired_alt = self.relative_altitude()
                self.in_band_since = None
                self._set_led('blink_red')
                self._enter_stage(self.DESCEND)
                self.get_logger().warning(
                    f"DESCEND: over the box ({err * 100:.0f} cm), hottest in "
                    f"{confidence * 100:.0f}% of recent frames. Stepping down to "
                    f"{self.DROP_ALTITUDE:.2f} m, LED blinking red.")
                return
        else:
            self.in_band_since = None

        if self._in_stage_for() > self.STAGE_TIMEOUT:
            self._finish("could not settle over the box at survey altitude", land=True)
            return
        self.get_logger().info(
            f"APPROACH: {err:.2f} m to go, track {'LIVE' if live else 'lost'}, "
            f"hottest-in-frame "
            + (f"{self.verify_count}/{self.VERIFY_FRAMES} frames of evidence"
               if confidence is None else
               f"{confidence * 100:.0f}% (want {self.VERIFY_RATIO * 100:.0f}%)")
            + ".", throttle_duration_sec=1.0)

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
                "DESCEND: lost the box; climbing back to survey altitude to "
                "reacquire. LED off until the descent restarts.")
            self._set_led('off')
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
                self._set_led('off')
                with self._lock:
                    self.track.clear()
                self.track_lost_since = None
                self._enter_stage(self.APPROACH)
                return
            if live and err <= self.ALIGN_TOLERANCE:
                if self.in_band_since is None:
                    self.in_band_since = now
                elif now - self.in_band_since >= self.ALIGN_SETTLE:
                    self._release_payload(err)
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

        # Released. Hold the servo open long enough for the payload to clear,
        # put it back, then settle briefly before climbing away.
        elapsed = now - self.released_at
        if elapsed < self.SERVO_HOLD_SECONDS:
            self._send_actuator(self.SERVO_DROP_VALUE)
            self.get_logger().info(
                f"DROP: servo open, {self.SERVO_HOLD_SECONDS - elapsed:.1f} s left.",
                throttle_duration_sec=0.5)
            return
        if not self.servo_returned:
            self.servo_returned = True
            if self.RELEASE_ENABLED and not self.RELEASE_VIA_NODE \
                    and not self.servo_ack_seen:
                self.get_logger().error(
                    "PX4 never acknowledged the servo command. Nothing moved "
                    "because nothing arrived: check the DDS agent, and try "
                    "`ros2 run drone_testing servo_test` on the bench.")
            self.drop_trigger_pub.publish(Bool(data=False))
            self._send_actuator(self.SERVO_NEUTRAL_VALUE, force=True)
            self._set_led('off')
            self.get_logger().warning("Servo back to neutral, LED off.")
        if elapsed < self.SERVO_HOLD_SECONDS + self.HOVER_SECONDS:
            self.get_logger().info("Settling after the drop.", throttle_duration_sec=1.0)
            return
        self.retreat_target = None
        self._enter_stage(self.RETREAT)
        self.get_logger().warning(
            f"RETREAT: climbing back to {self.RETREAT_ALTITUDE:.2f} m, then "
            f"{self.RETREAT_RIGHT:.2f} m to the right before landing.")

    def _release_payload(self, err):
        """Commit: open the servo over the box.

        MAV_CMD_DO_SET_ACTUATOR, so it goes down the DDS link the rest of the
        mission already uses. The PX4 output must be assigned to "Offboard
        Actuator Set <servo_index>" in QGC's Actuators tab -- an output left on
        "RC AUX 1" is an RC passthrough and ignores this command entirely.
        """
        self.drop_ready = True
        self.released_at = time.monotonic()
        self.servo_returned = False
        self._restart_stage_clock()
        self.outcome = (f"DROPPED on the hot box, {err * 100:.0f} cm off centre "
                        f"at {self.relative_altitude():.2f} m")
        if not self.RELEASE_ENABLED:
            self.get_logger().warning(
                f"DROP POSITION CONFIRMED ({err * 100:.0f} cm) but "
                "release_enabled is false: NOT commanding the servo.")
            return
        self.servo_ack_seen = False
        self.drop_trigger_pub.publish(Bool(data=True))
        self._send_actuator(self.SERVO_DROP_VALUE, force=True)
        self._set_led('solid_red')
        if self.RELEASE_VIA_NODE:
            self.get_logger().warning(
                f"DROP: {self.outcome}. True published on "
                f"{self.DROP_TRIGGER_TOPIC}; servo_controller.py opens the "
                "servo. If the cone does not go, check that node is running "
                "(`ros2 node list`) before suspecting the linkage.")
        else:
            self.get_logger().warning(
                f"DROP: {self.outcome}. Servo set {self.SERVO_DROP_VALUE:+.2f} via "
                + (f"actuator_test function {self.SERVO_FUNCTION}."
                   if self.SERVO_COMMAND == 'actuator_test'
                   else f"DO_SET_ACTUATOR on offboard actuator set "
                        f"{self.SERVO_INDEX}.")
                + " Watching for PX4's ack.")

    def _send_actuator(self, value, force=False):
        """MAV_CMD_DO_SET_ACTUATOR. NaN leaves the other outputs alone."""
        if not self.RELEASE_ENABLED:
            return
        if self.RELEASE_VIA_NODE:
            # servo_controller.py owns the output. Two publishers sending
            # different values to one actuator at 20 Hz is a servo that
            # buzzes between them, so this node does not send at all.
            return
        now = time.monotonic()
        if not force and now - self._last_actuator_send < 0.25:
            return
        self._last_actuator_send = now
        nan = float('nan')
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        if self.SERVO_COMMAND == 'actuator_test':
            # What QGC's Actuators tab uses. This is the one that works while
            # DISARMED, which is the only way a dry run can move the servo --
            # but it needs the PX4 output FUNCTION number, not the offboard
            # set index, and PX4 stops the test when the timeout expires.
            msg.command = 310      # MAV_CMD_ACTUATOR_TEST
            msg.param1 = float(value)
            msg.param2 = float(self.SERVO_HOLD_SECONDS)
            msg.param3 = msg.param4 = 0.0
            msg.param5 = float(self.SERVO_FUNCTION)
            msg.param6 = msg.param7 = 0.0
        else:
            params = [nan] * 6
            params[max(1, min(6, self.SERVO_INDEX)) - 1] = float(value)
            msg.command = 187      # MAV_CMD_DO_SET_ACTUATOR
            (msg.param1, msg.param2, msg.param3,
             msg.param4, msg.param5, msg.param6) = params
            msg.param7 = 0.0       # actuator set index group
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

    def _set_led(self, mode):
        if mode == self.led_mode:
            return
        self.led_mode = mode
        self.led_pub.publish(String(data=mode))

    # -------------------------------------------------------------- RETREAT

    def _handle_retreat(self):
        """Up first, then sideways. Never sideways at drop height: the payload
        is on the floor there and so is everything else in the arena."""
        alt = self.relative_altitude()
        lp = self.local_position
        if alt is None or lp is None:
            return
        self._set_altitude(self.RETREAT_ALTITUDE)

        if self.retreat_target is None:
            self.moving = False
            if abs(alt - self.RETREAT_ALTITUDE) <= self.ALTITUDE_TOLERANCE:
                hdg = lp.heading
                right = np.array([-math.sin(hdg), math.cos(hdg)])
                self.retreat_target = np.array([lp.x, lp.y]) + right * self.RETREAT_RIGHT
                self._restart_stage_clock()
                self.get_logger().warning(
                    f"RETREAT: at {alt:.2f} m; stepping {self.RETREAT_RIGHT:.2f} m "
                    f"right to ({self.retreat_target[0]:+.2f}, "
                    f"{self.retreat_target[1]:+.2f}) NED.")
            elif self._in_stage_for() > self.RETREAT_TIMEOUT:
                self._finish("could not climb away after the drop", land=True)
            else:
                self.get_logger().info(
                    f"RETREAT: climbing {alt:.2f} -> {self.RETREAT_ALTITUDE:.2f} m.",
                    throttle_duration_sec=1.0)
            return

        self._move_to(self.retreat_target)
        dist = math.hypot(self.retreat_target[0] - lp.x, self.retreat_target[1] - lp.y)
        if dist <= self.MOVE_TOLERANCE or self._in_stage_for() > self.RETREAT_TIMEOUT:
            self._after_retreat()
            return
        self.get_logger().info(f"RETREAT: {dist:.2f} m to go.",
                               throttle_duration_sec=1.0)

    def _after_retreat(self):
        """The end of the mission, once the aircraft is clear of the box.

        A hook, because "clear of the box" is not always the end: thermal_fsm
        goes looking for a landing marker here instead of putting it down
        wherever the retreat happened to stop.
        """
        self._finish("payload dropped, clear of the box",
                     land=self.LAND_AFTER_DROP)
        if not self.LAND_AFTER_DROP:
            self.get_logger().info(
                "Clear of the box and holding (land_after_drop is false). "
                "q to descend.", throttle_duration_sec=5.0)

    def _finish(self, reason, land):
        if self.outcome == 'not attempted':
            self.outcome = f"ENDED: {reason}"
        self.drop_ready = False
        self._set_led('off')
        with self._lock:
            self.collecting = False
        self.moving = False
        if self.MODE == 'dryrun':
            self._enter_stage(self.DONE)
            return
        if land:
            self._begin_landing(reason)

    # -------------------------------------------------------------- dry run

    def _start_dryrun(self):
        """Run the whole mission with a SIMULATED vehicle.

        Nothing reaches PX4 except the servo command: no arming, no Offboard
        request, no heartbeat, no setpoint -- publish_vehicle_command is
        blocked below and stream_setpoints is off, so the motors cannot be
        commanded even by a bug in a stage handler. What IS real is the
        thermal camera, the blob detection, the survey, the hottest-box
        verification, the LED and the servo.

        How it closes the loop: the simulated vehicle never changes what the
        camera sees, so the horizontal error the mission measures is simply
        where the hot object sits relative to the drop point, at the simulated
        height. YOU are the position controller -- move the hot object (or the
        airframe) until the green box on :8082 sits on the crosshair, and the
        mission will descend, blink, and fire the servo exactly as it would in
        the air.
        """
        self.stream_setpoints = False
        self.home_z = 0.0
        self.home_yaw = 0.0
        self.hold_xy = True
        self.hold_x = self.hold_y = 0.0
        self.target_z = -self.SURVEY_ALTITUDE
        self.setpoint_z = -self.SURVEY_ALTITUDE
        self.commanded_altitude = self.SURVEY_ALTITUDE
        self.local_position = _SimPose(0.0, 0.0, self.sim_alt)
        self.survey_centre = np.zeros(2)
        # One survey point: the ring exists to see boxes from nearer nadir,
        # and a simulated move does not change the view.
        self.survey_points = [np.zeros(2)]
        self.survey_index = 0
        self.survey_arrived_since = None
        self.flight_start = time.monotonic()
        with self._lock:
            self.clusters = []
            self.collecting = True
        self._enter_stage(self.SURVEY)
        self.get_logger().warning(
            "DRY RUN: the full mission on a SIMULATED vehicle. Nothing is "
            "armed and no setpoint is sent -- the motors cannot spin. The "
            "thermal camera, the survey, the LED and the servo are REAL. "
            f"Simulated height starts at {self.SURVEY_ALTITUDE:.2f} m; put a "
            "hot object under the camera and centre it (watch the stream on "
            f":8082) to walk it down to {self.DROP_ALTITUDE:.2f} m and fire "
            "the servo. q aborts."
            + ("" if self.RELEASE_ENABLED else
               " release_enabled is FALSE: the servo will NOT move."))
        if self.SERVO_TEST_ON_START:
            self._servo_test_once()

    def _servo_test_once(self):
        """Open and close the servo once at startup, before anything else."""
        self.get_logger().warning(
            f"SERVO TEST: {self.SERVO_DROP_VALUE:+.2f} for "
            f"{self.SERVO_HOLD_SECONDS:.1f} s, then "
            f"{self.SERVO_NEUTRAL_VALUE:+.2f}.")
        self._send_actuator(self.SERVO_DROP_VALUE, force=True)
        time.sleep(self.SERVO_HOLD_SECONDS)
        self._send_actuator(self.SERVO_NEUTRAL_VALUE, force=True)
        self.get_logger().warning(
            "SERVO TEST done. If nothing moved, see the servo notes in the "
            "launch file: the output must be an 'Offboard Actuator Set', and "
            "while DISARMED PX4 may only accept servo_command:=actuator_test.")

    def _sim_step(self):
        """Advance the simulated vehicle one tick towards what was commanded."""
        dt = 0.05
        delta = self.commanded_altitude - self.sim_alt
        speed = self.CLIMB_SPEED if delta > 0 else self.SIM_DESCEND_SPEED
        step = speed * dt
        vz = 0.0
        if abs(delta) > step:
            self.sim_alt += math.copysign(step, delta)
            vz = -math.copysign(speed, delta)
        else:
            self.sim_alt = self.commanded_altitude

        if self.moving and self.move_target_x is not None:
            d = np.array([self.move_target_x, self.move_target_y]) - self.sim_xy
            remaining = float(np.linalg.norm(d))
            mstep = self.MOVE_SPEED * dt
            self.sim_xy = (self.sim_xy + d if remaining <= mstep
                           else self.sim_xy + d / remaining * mstep)
        self.hold_x, self.hold_y = float(self.sim_xy[0]), float(self.sim_xy[1])

        self.local_position = _SimPose(self.sim_xy[0], self.sim_xy[1],
                                       self.sim_alt, vz=vz)
        # A pose for the projection to use. Real attitude if PX4 happens to be
        # connected -- which makes the dry run a tilt-compensation test too --
        # and level if it is not.
        q = self.attitude_q if self.attitude_q is not None else [1.0, 0.0, 0.0, 0.0]
        with self._lock:
            self.pose_buffer.append(
                (time.time(), q, (float(self.sim_xy[0]), float(self.sim_xy[1]),
                                  -self.sim_alt)))

    def _dryrun_tick(self):
        self._sim_step()
        self.publish_status()
        self.ready_pub.publish(Bool(data=self.drop_ready))

        if self.kill_requested or self.abort_requested:
            self.abort_requested = False
            self.kill_requested = False
            self._finish("operator abort", land=False)
            return

        if self.current_stage not in self.DROP_STAGES:
            if not self._dryrun_done_logged:
                self._dryrun_done_logged = True
                self._set_led('off')
                self.get_logger().warning(
                    f"DRY RUN COMPLETE: {self.outcome}. Ctrl-C to quit.")
            return

        self.get_logger().info(
            f"SIM: {self.current_stage} at {self.sim_alt:.2f} m, "
            f"({self.sim_xy[0]:+.2f}, {self.sim_xy[1]:+.2f}) NED.",
            throttle_duration_sec=2.0)

        {self.SURVEY: self._handle_survey,
         self.APPROACH: self._handle_approach,
         self.DESCEND: self._handle_descend,
         self.HOVER: self._handle_hover,
         self.RETREAT: self._handle_retreat}[self.current_stage]()

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, force=False):
        """In a dry run, nothing that could move the aircraft leaves this node.

        The servo goes out through _send_actuator, which builds its own
        message and deliberately does not come through here.
        """
        if self.MODE in ('bench', 'dryrun'):
            self.get_logger().warning(
                f"{self.MODE}: suppressed vehicle command {command}.",
                throttle_duration_sec=5.0)
            return
        super().publish_vehicle_command(command, param1, param2, force)

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
            detail = 'DROPPED' if self.drop_ready else 'align'
        elif self.current_stage == self.RETREAT:
            detail = 'up' if self.retreat_target is None else 'right'
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
